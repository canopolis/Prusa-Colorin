///|/ Copyright (c) Prusa Research 2021 - 2023 Oleksandra Iushchenko @YuSanka, Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, Lukáš Hejl @hejllukas
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <boost/log/trivial.hpp>
#include <iomanip>
#include <sstream>
#include "ImageMap/VolumeData.hpp"
#include <fstream>
#include "SVG.hpp"
#include "Utils.hpp"
#include <numeric>
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/parallel_for.h>
#include <algorithm>
#include <atomic> // Prusa Colorin (FullSpectrum)
#include <optional> // Prusa Colorin (FullSpectrum)
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cassert>
#include <cstddef>

#include "ClipperUtils.hpp"
#include "ElephantFootCompensation.hpp"
#include "I18N.hpp"
#include "Layer.hpp"
#include "MultiMaterialSegmentation.hpp"
#include "Print.hpp"
#include "ShortestPath.hpp"
#include "admesh/stl.h"
#include "libslic3r/Feature/Interlocking/InterlockingGenerator.hpp"
#include "libslic3r/Feature/FullSpectrum/VirtualExtruder.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/libslic3r.h"


namespace Slic3r {

// Prusa Colorin S4 forward declarations (defined in the S4 block below).
static bool has_surface_emboss_mixed_volume(const PrintObject &print_object);
static std::string surface_emboss_mixed_debug_file_path(const PrintObject &print_object);
static void reset_surface_emboss_mixed_debug_file(const PrintObject &print_object);

LayerPtrs new_layers(
    PrintObject                 *print_object,
    // Object layers (pairs of bottom/top Z coordinate), without the raft.
    const std::vector<coordf_t> &object_layers)
{
    LayerPtrs out;
    out.reserve(object_layers.size());
    auto     id   = int(print_object->slicing_parameters().raft_layers());
    coordf_t zmin = print_object->slicing_parameters().object_print_z_min;
    Layer   *prev = nullptr;
    for (size_t i_layer = 0; i_layer < object_layers.size(); i_layer += 2) {
        coordf_t lo = object_layers[i_layer];
        coordf_t hi = object_layers[i_layer + 1];
        coordf_t slice_z = 0.5 * (lo + hi);
        Layer *layer = new Layer(id ++, print_object, hi - lo, hi + zmin, slice_z);
        out.emplace_back(layer);
        if (prev != nullptr) {
            prev->upper_layer = layer;
            layer->lower_layer = prev;
        }
        prev = layer;
    }
    return out;
}

// Slice single triangle mesh.
static std::vector<ExPolygons> slice_volume(
    const ModelVolume             &volume,
    const std::vector<float>      &zs, 
    const MeshSlicingParamsEx     &params,
    const std::function<void()>   &throw_on_cancel_callback)
{
    std::vector<ExPolygons> layers;
    if (! zs.empty()) {
        indexed_triangle_set its = volume.mesh().its;
        if (its.indices.size() > 0) {
            MeshSlicingParamsEx params2 { params };
            params2.trafo = params2.trafo * volume.get_matrix();
            if (params2.trafo.rotation().determinant() < 0.)
                its_flip_triangles(its);
            layers = slice_mesh_ex(its, zs, params2, throw_on_cancel_callback);
            throw_on_cancel_callback();
        }
    }
    return layers;
}

// Slice single triangle mesh.
// Filter the zs not inside the ranges. The ranges are closed at the bottom and open at the top, they are sorted lexicographically and non overlapping.
static std::vector<ExPolygons> slice_volume(
    const ModelVolume                           &volume,
    const std::vector<float>                    &z,
    const std::vector<t_layer_height_range>     &ranges,
    const MeshSlicingParamsEx                   &params,
    const std::function<void()>                 &throw_on_cancel_callback)
{
    std::vector<ExPolygons> out;
    if (! z.empty() && ! ranges.empty()) {
        if (ranges.size() == 1 && z.front() >= ranges.front().first && z.back() < ranges.front().second) {
            // All layers fit into a single range.
            out = slice_volume(volume, z, params, throw_on_cancel_callback);
        } else {
            std::vector<float>                     z_filtered;
            std::vector<std::pair<size_t, size_t>> n_filtered;
            z_filtered.reserve(z.size());
            n_filtered.reserve(2 * ranges.size());
            size_t i = 0;
            for (const t_layer_height_range &range : ranges) {
                for (; i < z.size() && z[i] < range.first; ++ i) ;
                size_t first = i;
                for (; i < z.size() && z[i] < range.second; ++ i)
                    z_filtered.emplace_back(z[i]);
                if (i > first)
                    n_filtered.emplace_back(std::make_pair(first, i));
            }
            if (! n_filtered.empty()) {
                std::vector<ExPolygons> layers = slice_volume(volume, z_filtered, params, throw_on_cancel_callback);
                out.assign(z.size(), ExPolygons());
                i = 0;
                for (const std::pair<size_t, size_t> &span : n_filtered)
                    for (size_t j = span.first; j < span.second; ++ j)
                        out[j] = std::move(layers[i ++]);
            }
        }
    }
    return out;
}

struct VolumeSlices
{
    ObjectID                volume_id;
    std::vector<ExPolygons> slices;
};

static inline bool model_volume_needs_slicing(const ModelVolume &mv)
{
    ModelVolumeType type = mv.type();
    return type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME || type == ModelVolumeType::PARAMETER_MODIFIER;
}

// Slice printable volumes, negative volumes and modifier volumes, sorted by ModelVolume::id().
// Apply closing radius.
// Apply positive XY compensation to ModelVolumeType::MODEL_PART and ModelVolumeType::PARAMETER_MODIFIER, not to ModelVolumeType::NEGATIVE_VOLUME.
// Apply contour simplification.
static std::vector<VolumeSlices> slice_volumes_inner(
    const PrintConfig                                        &print_config,
    const PrintObjectConfig                                  &print_object_config,
    const Transform3d                                        &object_trafo,
    ModelVolumePtrs                                           model_volumes,
    const std::vector<PrintObjectRegions::LayerRangeRegions> &layer_ranges,
    const std::vector<float>                                 &zs,
    const std::function<void()>                              &throw_on_cancel_callback)
{
    model_volumes_sort_by_id(model_volumes);

    std::vector<VolumeSlices> out;
    out.reserve(model_volumes.size());

    std::vector<t_layer_height_range> slicing_ranges;
    if (layer_ranges.size() > 1)
        slicing_ranges.reserve(layer_ranges.size());

    MeshSlicingParamsEx params_base;
    params_base.closing_radius = print_object_config.slice_closing_radius.value;
    params_base.extra_offset   = 0;
    params_base.trafo          = object_trafo;
    params_base.resolution     = print_config.resolution.value;

    switch (print_object_config.slicing_mode.value) {
    case SlicingMode::Regular:    params_base.mode = MeshSlicingParams::SlicingMode::Regular; break;
    case SlicingMode::EvenOdd:    params_base.mode = MeshSlicingParams::SlicingMode::EvenOdd; break;
    case SlicingMode::CloseHoles: params_base.mode = MeshSlicingParams::SlicingMode::Positive; break;
    }

    params_base.mode_below     = params_base.mode;

    const size_t num_extruders = print_config.nozzle_diameter.size();
    const bool   is_mm_painted = num_extruders > 1 && std::any_of(model_volumes.cbegin(), model_volumes.cend(), [](const ModelVolume *mv) { return mv->is_mm_painted(); });
    const auto   extra_offset  = is_mm_painted ? 0.f : std::max(0.f, float(print_object_config.xy_size_compensation.value));

    for (const ModelVolume *model_volume : model_volumes)
        if (model_volume_needs_slicing(*model_volume)) {
            MeshSlicingParamsEx params { params_base };
            if (! model_volume->is_negative_volume())
                params.extra_offset = extra_offset;
            if (layer_ranges.size() == 1) {
                if (const PrintObjectRegions::LayerRangeRegions &layer_range = layer_ranges.front(); layer_range.has_volume(model_volume->id())) {
                    if (model_volume->is_model_part() && print_config.spiral_vase) {
                        auto it = std::find_if(layer_range.volume_regions.begin(), layer_range.volume_regions.end(), 
                            [model_volume](const auto &slice){ return model_volume == slice.model_volume; });
                        params.mode = MeshSlicingParams::SlicingMode::PositiveLargestContour;
                        // Slice the bottom layers with SlicingMode::Regular.
                        // This needs to be in sync with LayerRegion::make_perimeters() spiral_vase!
                        const PrintRegionConfig &region_config = it->region->config();
                        params.slicing_mode_normal_below_layer = size_t(region_config.bottom_solid_layers.value);
                        for (; params.slicing_mode_normal_below_layer < zs.size() && zs[params.slicing_mode_normal_below_layer] < region_config.bottom_solid_min_thickness - EPSILON;
                            ++ params.slicing_mode_normal_below_layer);
                    }
                    out.push_back({
                        model_volume->id(), 
                        slice_volume(*model_volume, zs, params, throw_on_cancel_callback)
                    });
                }
            } else {
                assert(! print_config.spiral_vase);
                slicing_ranges.clear();
                for (const PrintObjectRegions::LayerRangeRegions &layer_range : layer_ranges)
                    if (layer_range.has_volume(model_volume->id()))
                        slicing_ranges.emplace_back(layer_range.layer_height_range);
                if (! slicing_ranges.empty())
                    out.push_back({ 
                        model_volume->id(), 
                        slice_volume(*model_volume, zs, slicing_ranges, params, throw_on_cancel_callback)
                    });
            }
            if (! out.empty() && out.back().slices.empty())
                out.pop_back();
        }

    return out;
}

static inline VolumeSlices& volume_slices_find_by_id(std::vector<VolumeSlices> &volume_slices, const ObjectID id)
{
    auto it = lower_bound_by_predicate(volume_slices.begin(), volume_slices.end(), [id](const VolumeSlices &vs) { return vs.volume_id < id; });
    assert(it != volume_slices.end() && it->volume_id == id);
    return *it;
}

static inline bool overlap_in_xy(const PrintObjectRegions::BoundingBox &l, const PrintObjectRegions::BoundingBox &r)
{
    return ! (l.max().x() < r.min().x() || l.min().x() > r.max().x() ||
              l.max().y() < r.min().y() || l.min().y() > r.max().y());
}

static std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator layer_range_first(const std::vector<PrintObjectRegions::LayerRangeRegions> &layer_ranges, double z)
{
    auto  it = lower_bound_by_predicate(layer_ranges.begin(), layer_ranges.end(),
        [z](const PrintObjectRegions::LayerRangeRegions &lr) { return lr.layer_height_range.second < z; });
    assert(it != layer_ranges.end() && it->layer_height_range.first <= z && z <= it->layer_height_range.second);
    if (z == it->layer_height_range.second)
        if (auto it_next = it; ++ it_next != layer_ranges.end() && it_next->layer_height_range.first == z)
            it = it_next;
    assert(it != layer_ranges.end() && it->layer_height_range.first <= z && z <= it->layer_height_range.second);
    return it;
}

static std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator layer_range_next(
    const std::vector<PrintObjectRegions::LayerRangeRegions>            &layer_ranges, 
    std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator   it,
    double                                                               z)
{
    for (; it->layer_height_range.second <= z; ++ it)
        assert(it != layer_ranges.end());
    assert(it != layer_ranges.end() && it->layer_height_range.first <= z && z < it->layer_height_range.second);
    return it;
}

static std::vector<std::vector<ExPolygons>> slices_to_regions(
    ModelVolumePtrs                                           model_volumes,
    const PrintObjectRegions                                 &print_object_regions,
    const std::vector<float>                                 &zs,
    std::vector<VolumeSlices>                               &&volume_slices,
    const std::function<void()>                              &throw_on_cancel_callback)
{
    model_volumes_sort_by_id(model_volumes);

    std::vector<std::vector<ExPolygons>> slices_by_region(print_object_regions.all_regions.size(), std::vector<ExPolygons>(zs.size(), ExPolygons()));

    // First shuffle slices into regions if there is no overlap with another region possible, collect zs of the complex cases.
    std::vector<std::pair<size_t, float>> zs_complex;
    {
        size_t z_idx = 0;
        for (const PrintObjectRegions::LayerRangeRegions &layer_range : print_object_regions.layer_ranges) {
            for (; z_idx < zs.size() && zs[z_idx] < layer_range.layer_height_range.first; ++ z_idx) ;
            if (layer_range.volume_regions.empty()) {
            } else if (layer_range.volume_regions.size() == 1) {
                const ModelVolume *model_volume = layer_range.volume_regions.front().model_volume;
                assert(model_volume != nullptr);
                if (model_volume->is_model_part()) {
                    VolumeSlices &slices_src = volume_slices_find_by_id(volume_slices, model_volume->id());
                    auto         &slices_dst = slices_by_region[layer_range.volume_regions.front().region->print_object_region_id()];
                    for (; z_idx < zs.size() && zs[z_idx] < layer_range.layer_height_range.second; ++ z_idx)
                        slices_dst[z_idx] = std::move(slices_src.slices[z_idx]);
                }
            } else {
                zs_complex.reserve(zs.size());
                for (; z_idx < zs.size() && zs[z_idx] < layer_range.layer_height_range.second; ++ z_idx) {
                    float z                          = zs[z_idx];
                    int   idx_first_printable_region = -1;
                    bool  complex                    = false;
                    for (int idx_region = 0; idx_region < int(layer_range.volume_regions.size()); ++ idx_region) {
                        const PrintObjectRegions::VolumeRegion &region = layer_range.volume_regions[idx_region];
                        if (region.bbox->min().z() <= z && region.bbox->max().z() >= z) {
                            if (idx_first_printable_region == -1 && region.model_volume->is_model_part())
                                idx_first_printable_region = idx_region;
                            else if (idx_first_printable_region != -1) {
                                // Test for overlap with some other region.
                                for (int idx_region2 = idx_first_printable_region; idx_region2 < idx_region; ++ idx_region2) {
                                    const PrintObjectRegions::VolumeRegion &region2 = layer_range.volume_regions[idx_region2];
                                    if (region2.bbox->min().z() <= z && region2.bbox->max().z() >= z && overlap_in_xy(*region.bbox, *region2.bbox)) {
                                        complex = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    if (complex)
                        zs_complex.push_back({ z_idx, z });
                    else if (idx_first_printable_region >= 0) {
                        const PrintObjectRegions::VolumeRegion &region = layer_range.volume_regions[idx_first_printable_region];
                        slices_by_region[region.region->print_object_region_id()][z_idx] = std::move(volume_slices_find_by_id(volume_slices, region.model_volume->id()).slices[z_idx]);
                    }
                }
            }
            throw_on_cancel_callback();
        }
    }

    // Second perform region clipping and assignment in parallel.
    if (! zs_complex.empty()) {
        std::vector<std::vector<VolumeSlices*>> layer_ranges_regions_to_slices(print_object_regions.layer_ranges.size(), std::vector<VolumeSlices*>());
        for (const PrintObjectRegions::LayerRangeRegions &layer_range : print_object_regions.layer_ranges) {
            std::vector<VolumeSlices*> &layer_range_regions_to_slices = layer_ranges_regions_to_slices[&layer_range - print_object_regions.layer_ranges.data()];
            layer_range_regions_to_slices.reserve(layer_range.volume_regions.size());
            for (const PrintObjectRegions::VolumeRegion &region : layer_range.volume_regions)
                layer_range_regions_to_slices.push_back(&volume_slices_find_by_id(volume_slices, region.model_volume->id()));
        }
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, zs_complex.size()),
            [&slices_by_region, &print_object_regions, &zs_complex, &layer_ranges_regions_to_slices, &throw_on_cancel_callback]
                (const tbb::blocked_range<size_t> &range) {
                float z              = zs_complex[range.begin()].second;
                auto  it_layer_range = layer_range_first(print_object_regions.layer_ranges, z);
                // Per volume_regions slices at this Z height.
                struct RegionSlice { 
                    ExPolygons  expolygons;
                    // Identifier of this region in PrintObjectRegions::all_regions
                    int         region_id;
                    ObjectID    volume_id;
                    bool operator<(const RegionSlice &rhs) const {
                        bool this_empty = this->region_id < 0 || this->expolygons.empty();
                        bool rhs_empty  = rhs.region_id < 0 || rhs.expolygons.empty();
                        // Sort the empty items to the end of the list.
                        // Sort by region_id & volume_id lexicographically.
                        return ! this_empty && (rhs_empty || (this->region_id < rhs.region_id || (this->region_id == rhs.region_id && volume_id < volume_id)));
                    }
                };
                std::vector<RegionSlice> temp_slices;
                for (size_t zs_complex_idx = range.begin(); zs_complex_idx < range.end(); ++ zs_complex_idx) {
                    auto [z_idx, z] = zs_complex[zs_complex_idx];
                    it_layer_range = layer_range_next(print_object_regions.layer_ranges, it_layer_range, z);
                    const PrintObjectRegions::LayerRangeRegions &layer_range = *it_layer_range;
                    {
                        std::vector<VolumeSlices*> &layer_range_regions_to_slices = layer_ranges_regions_to_slices[it_layer_range - print_object_regions.layer_ranges.begin()];
                        // Per volume_regions slices at thiz Z height.
                        temp_slices.clear();
                        temp_slices.reserve(layer_range.volume_regions.size());
                        for (VolumeSlices* &slices : layer_range_regions_to_slices) {
                            const PrintObjectRegions::VolumeRegion &volume_region = layer_range.volume_regions[&slices - layer_range_regions_to_slices.data()];
                            temp_slices.push_back({ std::move(slices->slices[z_idx]), volume_region.region ? volume_region.region->print_object_region_id() : -1, volume_region.model_volume->id() });
                        }
                    }
                    for (int idx_region = 0; idx_region < int(layer_range.volume_regions.size()); ++ idx_region)
                        if (! temp_slices[idx_region].expolygons.empty()) {
                            const PrintObjectRegions::VolumeRegion &region = layer_range.volume_regions[idx_region];
                            if (region.model_volume->is_modifier()) {
                                assert(region.parent > -1);
                                bool next_region_same_modifier = idx_region + 1 < int(temp_slices.size()) && layer_range.volume_regions[idx_region + 1].model_volume == region.model_volume;
                                RegionSlice &parent_slice = temp_slices[region.parent];
                                RegionSlice &this_slice   = temp_slices[idx_region];
                                ExPolygons   source       = std::move(this_slice.expolygons);
                                if (parent_slice.expolygons.empty()) {
                                    this_slice  .expolygons.clear();
                                } else {
                                    this_slice  .expolygons = intersection_ex(parent_slice.expolygons, source);
                                    parent_slice.expolygons = diff_ex        (parent_slice.expolygons, source);
                                }
                                if (next_region_same_modifier)
                                    // To be used in the following iteration.
                                    temp_slices[idx_region + 1].expolygons = std::move(source);
                            } else if (region.model_volume->is_model_part() || region.model_volume->is_negative_volume()) {
                                // Clip every non-zero region preceding it.
                                for (int idx_region2 = 0; idx_region2 < idx_region; ++ idx_region2)
                                    if (! temp_slices[idx_region2].expolygons.empty()) {
                                        if (const PrintObjectRegions::VolumeRegion &region2 = layer_range.volume_regions[idx_region2];
                                            ! region2.model_volume->is_negative_volume() && overlap_in_xy(*region.bbox, *region2.bbox))
                                            temp_slices[idx_region2].expolygons = diff_ex(temp_slices[idx_region2].expolygons, temp_slices[idx_region].expolygons);
                                    }
                            }
                        }
                    // Sort by region_id, push empty slices to the end.
                    std::sort(temp_slices.begin(), temp_slices.end());
                    // Remove the empty slices.
                    temp_slices.erase(std::find_if(temp_slices.begin(), temp_slices.end(), [](const auto &slice) { return slice.region_id == -1 || slice.expolygons.empty(); }), temp_slices.end());
                    // Merge slices and store them to the output.
                    for (int i = 0; i < int(temp_slices.size());) {
                        // Find a range of temp_slices with the same region_id.
                        int j = i;
                        bool merged = false;
                        ExPolygons &expolygons = temp_slices[i].expolygons;
                        for (++ j; j < int(temp_slices.size()) && temp_slices[i].region_id == temp_slices[j].region_id; ++ j)
                            if (ExPolygons &expolygons2 = temp_slices[j].expolygons; ! expolygons2.empty()) {
                                if (expolygons.empty()) {
                                    expolygons = std::move(expolygons2);
                                } else {
                                    append(expolygons, std::move(expolygons2));
                                    merged = true;
                                }
                            }
                        if (merged)
                            expolygons = closing_ex(expolygons, float(scale_(EPSILON)));
                        slices_by_region[temp_slices[i].region_id][z_idx] = std::move(expolygons);
                        i = j;
                    }
                    throw_on_cancel_callback();
                }
            });
    }

    return slices_by_region;
}

// Layer::slicing_errors is no more set since 1.41.1 or possibly earlier, thus this code
// was not really functional for a long day and nobody missed it.
// Could we reuse this fixing code one day?
/*
std::string fix_slicing_errors(LayerPtrs &layers, const std::function<void()> &throw_if_canceled)
{
    // Collect layers with slicing errors.
    // These layers will be fixed in parallel.
    std::vector<size_t> buggy_layers;
    buggy_layers.reserve(layers.size());
    for (size_t idx_layer = 0; idx_layer < layers.size(); ++ idx_layer)
        if (layers[idx_layer]->slicing_errors)
            buggy_layers.push_back(idx_layer);

    BOOST_LOG_TRIVIAL(debug) << "Slicing objects - fixing slicing errors in parallel - begin";
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, buggy_layers.size()),
        [&layers, &throw_if_canceled, &buggy_layers](const tbb::blocked_range<size_t>& range) {
            for (size_t buggy_layer_idx = range.begin(); buggy_layer_idx < range.end(); ++ buggy_layer_idx) {
                throw_if_canceled();
                size_t idx_layer = buggy_layers[buggy_layer_idx];
                Layer *layer     = layers[idx_layer];
                assert(layer->slicing_errors);
                // Try to repair the layer surfaces by merging all contours and all holes from neighbor layers.
                // BOOST_LOG_TRIVIAL(trace) << "Attempting to repair layer" << idx_layer;
                for (size_t region_id = 0; region_id < layer->region_count(); ++ region_id) {
                    LayerRegion *layerm = layer->get_region(region_id);
                    // Find the first valid layer below / above the current layer.
                    const Surfaces *upper_surfaces = nullptr;
                    const Surfaces *lower_surfaces = nullptr;
                    for (size_t j = idx_layer + 1; j < layers.size(); ++ j)
                        if (! layers[j]->slicing_errors) {
                            upper_surfaces = &layers[j]->regions()[region_id]->slices().surfaces;
                            break;
                        }
                    for (int j = int(idx_layer) - 1; j >= 0; -- j)
                        if (! layers[j]->slicing_errors) {
                            lower_surfaces = &layers[j]->regions()[region_id]->slices().surfaces;
                            break;
                        }
                    // Collect outer contours and holes from the valid layers above & below.
                    Polygons outer;
                    outer.reserve(
                        ((upper_surfaces == nullptr) ? 0 : upper_surfaces->size()) + 
                        ((lower_surfaces == nullptr) ? 0 : lower_surfaces->size()));
                    size_t num_holes = 0;
                    if (upper_surfaces)
                        for (const auto &surface : *upper_surfaces) {
                            outer.push_back(surface.expolygon.contour);
                            num_holes += surface.expolygon.holes.size();
                        }
                    if (lower_surfaces)
                        for (const auto &surface : *lower_surfaces) {
                            outer.push_back(surface.expolygon.contour);
                            num_holes += surface.expolygon.holes.size();
                        }
                    Polygons holes;
                    holes.reserve(num_holes);
                    if (upper_surfaces)
                        for (const auto &surface : *upper_surfaces)
                            polygons_append(holes, surface.expolygon.holes);
                    if (lower_surfaces)
                        for (const auto &surface : *lower_surfaces)
                            polygons_append(holes, surface.expolygon.holes);
                    layerm->m_slices.set(diff_ex(union_(outer), holes), stInternal);
                }
                // Update layer slices after repairing the single regions.
                layer->make_slices();
            }
        });
    throw_if_canceled();
    BOOST_LOG_TRIVIAL(debug) << "Slicing objects - fixing slicing errors in parallel - end";

    // remove empty layers from bottom
    while (! layers.empty() && (layers.front()->lslices.empty() || layers.front()->empty())) {
        delete layers.front();
        layers.erase(layers.begin());
        layers.front()->lower_layer = nullptr;
        for (size_t i = 0; i < layers.size(); ++ i)
            layers[i]->set_id(layers[i]->id() - 1);
    }

    return buggy_layers.empty() ? "" :
        "The model has overlapping or self-intersecting facets. I tried to repair it, "
        "however you might want to check the results or repair the input file and retry.\n";
}
*/

// Called by make_perimeters()
// 1) Decides Z positions of the layers,
// 2) Initializes layers and their regions
// 3) Slices the object meshes
// 4) Slices the modifier meshes and reclassifies the slices of the object meshes by the slices of the modifier meshes
// 5) Applies size compensation (offsets the slices in XY plane)
// 6) Replaces bad slices by the slices reconstructed from the upper/lower layer
// Resulting expolygons of layer regions are marked as Internal.
void PrintObject::slice()
{
    if (! this->set_started(posSlice))
        return;
    m_print->set_status(10, _u8L("Processing triangulated mesh"));
    std::vector<coordf_t> layer_height_profile;
    this->update_layer_height_profile(*this->model_object(), m_slicing_params, layer_height_profile);
    m_print->throw_if_canceled();
    m_typed_slices = false;
    this->clear_layers();
    m_layers = new_layers(this, generate_object_layers(m_slicing_params, layer_height_profile));
    if (has_surface_emboss_mixed_volume(*this)) {
        reset_surface_emboss_mixed_debug_file(*this);
        BOOST_LOG_TRIVIAL(warning) << "Surface emboss mixed debug enabled"
                                   << " object=" << (this->model_object() ? this->model_object()->name : std::string("<unknown>"))
                                   << " debug_file=" << surface_emboss_mixed_debug_file_path(*this);
    }
    this->slice_volumes();
    m_print->throw_if_canceled();
#if 0
    // Layer::slicing_errors is no more set since 1.41.1 or possibly earlier, thus this code
    // was not really functional for a long day and nobody missed it.
    // Could we reuse this fixing code one day?

    // Fix the model.
    //FIXME is this the right place to do? It is done repeateadly at the UI and now here at the backend.
    std::string warning = fix_slicing_errors(m_layers, [this](){ m_print->throw_if_canceled(); });
    m_print->throw_if_canceled();
    if (! warning.empty())
        BOOST_LOG_TRIVIAL(info) << warning;
#endif
    // Update bounding boxes, back up raw slices of complex models.
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, m_layers.size()),
        [this](const tbb::blocked_range<size_t> &range) {
            for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                m_print->throw_if_canceled();
                Layer &layer = *m_layers[layer_idx];
                layer.lslices_ex.clear();
                layer.lslices_ex.reserve(layer.lslices.size());
                for (const ExPolygon &expoly : layer.lslices)
                	layer.lslices_ex.push_back({ get_extents(expoly) });
                layer.backup_untyped_slices();
            }
        });
    // Interlink the lslices into a Z graph.
    tbb::parallel_for(
        tbb::blocked_range<size_t>(1, m_layers.size()),
        [this](const tbb::blocked_range<size_t> &range) {
            for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                m_print->throw_if_canceled();
                Layer::build_up_down_graph(*m_layers[layer_idx - 1], *m_layers[layer_idx]);
            }
        });
    if (m_layers.empty())
        throw Slic3r::SlicingError("No layers were detected. You might want to repair your STL file(s) or check their size or thickness and retry.\n");    
    this->set_done(posSlice);
}

// Prusa Colorin (FullSpectrum)
// Resolves every painted mixed (virtual) filament channel to the physical filament that prints it
// on each layer, using the FullSpectrum MixedFilamentManager cadence
// (MixedFilamentManager::effective_painted_region_filament_id), exactly as OrcaSlicer FullSpectrum
// does in apply_mm_segmentation() when "mixed_filament_region_collapse" is enabled (its default).
// PrusaSlicer segmentation slots are 0-based: slot i holds filament ID i + 1.
// ---- Prusa Colorin S3: mixed component bias / surface offsets / indentation (Orca FullSpectrum v0.9.18 PrintObjectSlice.cpp 895-1211, 2316-2468) ----
static inline unsigned int segmentation_channel_filament_id(size_t channel_idx)
{
    // MM segmentation reserves channel 0 for the parent/default region.
    // All remaining channels already line up with the 1-based filament IDs.
    return unsigned(channel_idx);
}

static float mixed_filament_reference_nozzle_mm(const MixedFilamentDefinition &definition, const ConfigOptionFloats &nozzle_diameters)
{
    std::vector<float> samples;
    samples.reserve(2);

    auto append_if_valid = [&samples, &nozzle_diameters](unsigned int component_id) {
        if (component_id >= 1 && component_id <= nozzle_diameters.size())
            samples.emplace_back(std::max(0.05f, float(nozzle_diameters.get_at(component_id - 1))));
    };

    const MixedFilamentPrimaryPairView pair = definition.recipe.blend.primary_pair_or();
    append_if_valid(pair.component_a.id);
    append_if_valid(pair.component_b.id);

    if (samples.empty())
        return 0.4f;
    return std::accumulate(samples.begin(), samples.end(), 0.0f) / float(samples.size());
}

static coordf_t clamped_mixed_component_surface_offset(const MixedFilamentManager &mixed_mgr,
                                                       const PrintConfig          &print_cfg,
                                                       unsigned int                filament_id,
                                                       size_t                      num_physical,
                                                       int                         layer_index,
                                                       float                       layer_print_z,
                                                       float                       layer_height,
                                                       bool                        force_height_weighted = false)
{
    const std::optional<MixedFilamentDefinition> definition = mixed_mgr.mixed_filament_definition_from_id(filament_id, num_physical);
    if (!definition)
        return 0.f;

    const coordf_t offset_mm = coordf_t(mixed_mgr.component_surface_offset(
        filament_id,
        num_physical,
        layer_index,
        layer_print_z,
        layer_height,
        force_height_weighted));
    if (std::abs(offset_mm) <= EPSILON)
        return 0.f;

    const float reference_nozzle = mixed_filament_reference_nozzle_mm(*definition, print_cfg.nozzle_diameter);
    const coordf_t limit_mm = coordf_t(MixedFilamentManager::max_component_surface_offset_mm(reference_nozzle));
    return std::clamp(offset_mm, -limit_mm, limit_mm);
}

static bool apply_mixed_surface_indentation(PrintObject &print_object, std::vector<std::vector<ExPolygons>> &segmentation)
{
    const Print *print = print_object.print();
    if (print == nullptr || segmentation.empty())
        return false;

    const PrintConfig        &print_cfg = print->config();
    coordf_t indentation_mm = coordf_t(print_cfg.mixed_filament_surface_indentation.value);
    indentation_mm = std::clamp(indentation_mm, coordf_t(-2.f), coordf_t(2.f));
    if (std::abs(indentation_mm) <= EPSILON)
        return false;

    const size_t num_physical = print_cfg.filament_colour.size();
    const size_t num_channels = segmentation.front().size();
    if (num_channels <= num_physical)
        return false;

    const MixedFilamentManager &mixed_mgr = print->mixed_filament_manager();
    const bool  expand_outward = indentation_mm < 0.f;
    const float delta_scaled = float(scale_(std::abs(double(indentation_mm))));
    if (delta_scaled <= float(EPSILON))
        return false;

    size_t changed_layers = 0;
    size_t changed_states = 0;
    size_t emptied_states = 0;
    size_t overlap_clipped_states = 0;
    size_t outside_trimmed_states = 0;

    for (size_t layer_id = 0; layer_id < segmentation.size(); ++layer_id) {
        if (segmentation[layer_id].size() != num_channels)
            continue;

        bool       layer_changed = false;
        ExPolygons outside_trim_band;
        ExPolygons occupied;
        if (expand_outward) {
            for (size_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
                const ExPolygons &state_masks = segmentation[layer_id][channel_idx];
                if (state_masks.empty())
                    continue;

                const unsigned int state_id = unsigned(channel_idx + 1);
                if (!mixed_mgr.is_mixed(state_id, num_physical))
                    append(occupied, state_masks);
            }
            if (occupied.size() > 1)
                occupied = union_ex(occupied);
        } else {
            ExPolygons layer_masks;
            for (size_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
                const ExPolygons &state_masks = segmentation[layer_id][channel_idx];
                if (!state_masks.empty())
                    append(layer_masks, state_masks);
            }
            if (!layer_masks.empty()) {
                if (layer_masks.size() > 1)
                    layer_masks = union_ex(layer_masks);

                ExPolygons layer_inner = offset_ex(layer_masks, -delta_scaled);
                if (!layer_inner.empty() && layer_inner.size() > 1)
                    layer_inner = union_ex(layer_inner);

                outside_trim_band = layer_inner.empty() ? layer_masks : diff_ex(layer_masks, layer_inner, ApplySafetyOffset::Yes);
                if (!outside_trim_band.empty() && outside_trim_band.size() > 1)
                    outside_trim_band = union_ex(outside_trim_band);
            }
        }

        for (size_t channel_idx = num_physical; channel_idx < num_channels; ++channel_idx) {
            ExPolygons &state_masks = segmentation[layer_id][channel_idx];
            if (state_masks.empty())
                continue;

            const unsigned int state_id = unsigned(channel_idx + 1);
            if (!mixed_mgr.is_mixed(state_id, num_physical))
                continue;

            ExPolygons adjusted;
            if (expand_outward) {
                adjusted = offset_ex(state_masks, delta_scaled);
                if (!adjusted.empty() && adjusted.size() > 1)
                    adjusted = union_ex(adjusted);

                if (!adjusted.empty() && !occupied.empty()) {
                    ExPolygons clipped = diff_ex(adjusted, occupied, ApplySafetyOffset::Yes);
                    if (std::abs(area(clipped)) + EPSILON < std::abs(area(adjusted)))
                        ++overlap_clipped_states;
                    adjusted = std::move(clipped);
                    if (!adjusted.empty() && adjusted.size() > 1)
                        adjusted = union_ex(adjusted);
                }
            } else {
                adjusted = outside_trim_band.empty() ? state_masks : diff_ex(state_masks, outside_trim_band, ApplySafetyOffset::Yes);
                if (std::abs(area(adjusted)) + EPSILON < std::abs(area(state_masks)))
                    ++outside_trimmed_states;
                if (!adjusted.empty() && adjusted.size() > 1)
                    adjusted = union_ex(adjusted);
            }

            state_masks = std::move(adjusted);
            if (state_masks.empty())
                ++emptied_states;
            ++changed_states;
            layer_changed = true;

            if (expand_outward && !state_masks.empty()) {
                append(occupied, state_masks);
                if (occupied.size() > 1)
                    occupied = union_ex(occupied);
            }
        }

        if (layer_changed)
            ++changed_layers;
    }

    if (changed_states == 0)
        return false;

    BOOST_LOG_TRIVIAL(warning) << "Mixed surface indentation applied"
                               << " object=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
                               << " indentation_mm=" << indentation_mm
                               << " direction=" << (expand_outward ? "outward" : "inward")
                               << " changed_layers=" << changed_layers
                               << " changed_states=" << changed_states
                               << " emptied_states=" << emptied_states
                               << " overlap_clipped_states=" << overlap_clipped_states
                               << " outside_trimmed_states=" << outside_trimmed_states;
    return true;
}

static bool apply_mixed_component_surface_offsets(PrintObject &print_object, std::vector<std::vector<ExPolygons>> &segmentation)
{
    const Print *print = print_object.print();
    if (print == nullptr || segmentation.empty())
        return false;

    const PrintConfig        &print_cfg = print->config();
    if (print_cfg.dithering_local_z_mode.value)
        return false;
    if (!print_cfg.mixed_filament_component_bias_enabled.value)
        return false;

    const size_t num_physical = print_cfg.filament_colour.size();
    const size_t num_channels = segmentation.front().size();
    if (num_channels <= num_physical + 1)
        return false;

    const MixedFilamentManager &mixed_mgr = print->mixed_filament_manager();
    bool has_component_offsets = false;
    for (const MixedFilamentDefinition &definition : mixed_mgr.mixed_filament_definitions(num_physical)) {
        if (definition.visibility.tombstoned || mixed_filament_definition_uses_local_z(definition, false) ||
            definition.behavior.surface_bias.perimeter_modulation)
            continue;
        const std::vector<float> component_offsets = mixed_filament_component_surface_offsets(definition);
        if (std::any_of(component_offsets.begin(), component_offsets.end(), [](float offset) {
                return std::abs(offset) > EPSILON;
            })) {
            has_component_offsets = true;
            break;
        }
    }
    if (!has_component_offsets)
        return false;

    size_t changed_layers = 0;
    size_t changed_states = 0;
    size_t emptied_states = 0;
    size_t expanded_states = 0;
    size_t contracted_states = 0;

    for (size_t layer_id = 0; layer_id < segmentation.size(); ++layer_id) {
        if (segmentation[layer_id].size() != num_channels)
            continue;

        const Layer *layer = layer_id < size_t(print_object.layer_count()) ? print_object.get_layer(int(layer_id)) : nullptr;
        const float layer_print_z = layer ? float(layer->print_z) : 0.f;
        const float layer_height  = layer ? float(layer->height) : 0.f;
        bool layer_changed = false;

        for (size_t channel_idx = 1; channel_idx < num_channels; ++channel_idx) {
            ExPolygons &state_masks = segmentation[layer_id][channel_idx];
            if (state_masks.empty())
                continue;

            const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
            if (!mixed_mgr.is_mixed(state_id, num_physical))
                continue;
            const std::optional<MixedFilamentDefinition> definition =
                mixed_mgr.mixed_filament_definition_from_id(state_id, num_physical);
            if (definition &&
                (definition->visibility.tombstoned || mixed_filament_definition_uses_local_z(*definition, false) ||
                 definition->behavior.surface_bias.perimeter_modulation))
                continue;

            const coordf_t offset_mm = clamped_mixed_component_surface_offset(mixed_mgr,
                                                                              print_cfg,
                                                                              state_id,
                                                                              num_physical,
                                                                              int(layer_id),
                                                                              layer_print_z,
                                                                              layer_height);
            if (std::abs(offset_mm) <= EPSILON)
                continue;

            const float delta_scaled = float(scale_(std::abs(double(offset_mm))));
            if (delta_scaled <= float(EPSILON))
                continue;

            ExPolygons adjusted = offset_mm > 0 ? offset_ex(state_masks, -delta_scaled) : offset_ex(state_masks, delta_scaled);
            if (!adjusted.empty() && adjusted.size() > 1)
                adjusted = union_ex(adjusted);

            if (offset_mm < 0 && !adjusted.empty()) {
                ExPolygons occupied_other;
                for (size_t other_idx = 0; other_idx < num_channels; ++other_idx) {
                    if (other_idx == channel_idx)
                        continue;
                    if (!segmentation[layer_id][other_idx].empty())
                        append(occupied_other, segmentation[layer_id][other_idx]);
                }
                if (occupied_other.size() > 1)
                    occupied_other = union_ex(occupied_other);
                if (!occupied_other.empty()) {
                    ExPolygons clipped = diff_ex(adjusted, occupied_other, ApplySafetyOffset::Yes);
                    adjusted = std::move(clipped);
                    if (!adjusted.empty() && adjusted.size() > 1)
                        adjusted = union_ex(adjusted);
                }
            }

            state_masks = std::move(adjusted);
            if (state_masks.empty())
                ++emptied_states;
            if (offset_mm < 0)
                ++expanded_states;
            else
                ++contracted_states;
            ++changed_states;
            layer_changed = true;
        }

        if (layer_changed)
            ++changed_layers;
    }

    if (changed_states == 0)
        return false;

    BOOST_LOG_TRIVIAL(warning) << "Mixed component surface offsets applied"
                               << " object=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
                               << " changed_layers=" << changed_layers
                               << " changed_states=" << changed_states
                               << " contracted_states=" << contracted_states
                               << " expanded_states=" << expanded_states
                               << " emptied_states=" << emptied_states;
    return true;
}

static ExPolygons collect_layer_region_slices(const Layer &layer)
{
    ExPolygons out;
    for (const LayerRegion *layerm : layer.regions())
        append(out, to_expolygons(layerm->slices().surfaces));
    if (!out.empty())
        out = union_ex(out);
    return out;
}

static bool apply_mixed_region_surface_offsets(PrintObject &print_object)
{
    const Print *print = print_object.print();
    if (print == nullptr || print_object.layer_count() == 0)
        return false;

    const PrintConfig        &print_cfg = print->config();
    if (print_cfg.dithering_local_z_mode.value)
        return false;
    if (!print_cfg.mixed_filament_component_bias_enabled.value)
        return false;

    const size_t num_physical = print_cfg.filament_diameter.size();
    if (num_physical == 0)
        return false;

    const MixedFilamentManager &mixed_mgr = print->mixed_filament_manager();
    bool has_component_offsets = false;
    for (const MixedFilamentDefinition &definition : mixed_mgr.mixed_filament_definitions(num_physical)) {
        if (definition.visibility.tombstoned || mixed_filament_definition_uses_local_z(definition, false) ||
            definition.behavior.surface_bias.perimeter_modulation)
            continue;
        const std::vector<float> component_offsets = mixed_filament_component_surface_offsets(definition);
        if (std::any_of(component_offsets.begin(), component_offsets.end(), [](float offset) {
                return std::abs(offset) > EPSILON;
            })) {
            has_component_offsets = true;
            break;
        }
    }
    if (!has_component_offsets)
        return false;

    size_t changed_layers    = 0;
    size_t changed_regions   = 0;
    size_t contracted_regions = 0;
    size_t expanded_regions  = 0;
    size_t stolen_regions    = 0;

    struct PendingRegionOffset {
        int        region_id { -1 };
        coordf_t   offset_mm { 0.f };
        ExPolygons adjusted;
    };

    for (size_t layer_id = 0; layer_id < print_object.layer_count(); ++layer_id) {
        Layer &layer = *print_object.get_layer(int(layer_id));
        std::vector<PendingRegionOffset> pending;
        pending.reserve(size_t(layer.region_count()));

        for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
            LayerRegion *layerm = layer.get_region(region_id);
            if (layerm == nullptr || layerm->slices().empty())
                continue;

            const unsigned int filament_id = unsigned(std::max(0, layerm->region().config().perimeter_extruder.value));
            if (!mixed_mgr.is_mixed(filament_id, num_physical))
                continue;
            const std::optional<MixedFilamentDefinition> definition =
                mixed_mgr.mixed_filament_definition_from_id(filament_id, num_physical);
            if (definition &&
                (definition->visibility.tombstoned || mixed_filament_definition_uses_local_z(*definition, false) ||
                 definition->behavior.surface_bias.perimeter_modulation))
                continue;

            const coordf_t offset_mm = clamped_mixed_component_surface_offset(mixed_mgr,
                                                                              print_cfg,
                                                                              filament_id,
                                                                              num_physical,
                                                                              int(layer_id),
                                                                              float(layer.print_z),
                                                                              float(layer.height));
            if (std::abs(offset_mm) <= EPSILON)
                continue;

            const float delta_scaled = float(scale_(std::abs(double(offset_mm))));
            if (delta_scaled <= float(EPSILON))
                continue;

            const ExPolygons original = to_expolygons(layerm->slices().surfaces);
            ExPolygons adjusted = offset_ex(original, offset_mm > 0 ? -delta_scaled : delta_scaled);
            if (!adjusted.empty() && adjusted.size() > 1)
                adjusted = union_ex(adjusted);

            pending.push_back({ region_id, offset_mm, std::move(adjusted) });
        }

        if (pending.empty())
            continue;

        bool layer_changed = false;
        for (const PendingRegionOffset &entry : pending) {
            LayerRegion *layerm = layer.get_region(entry.region_id);
            if (layerm == nullptr)
                continue;

            if (entry.offset_mm < 0 && !entry.adjusted.empty()) {
                for (int other_region_id = 0; other_region_id < layer.region_count(); ++other_region_id) {
                    if (other_region_id == entry.region_id)
                        continue;

                    LayerRegion *other = layer.get_region(other_region_id);
                    if (other == nullptr || other->slices().empty())
                        continue;

                    ExPolygons stolen = intersection_ex(other->slices().surfaces, entry.adjusted);
                    if (stolen.empty())
                        continue;

                    Polygons remaining = diff(to_polygons(other->slices().surfaces), entry.adjusted);
                    const_cast<SurfaceCollection&>(other->slices()).set(union_ex(remaining), stInternal);
                    ++stolen_regions;
                    layer_changed = true;
                }
            }

            const_cast<SurfaceCollection&>(layerm->slices()).set(entry.adjusted, stInternal);
            ++changed_regions;
            if (entry.offset_mm > 0)
                ++contracted_regions;
            else
                ++expanded_regions;
            layer_changed = true;
        }

        if (layer_changed)
            ++changed_layers;
    }

    if (changed_regions == 0)
        return false;

    BOOST_LOG_TRIVIAL(warning) << "Mixed region surface offsets applied"
                               << " object=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
                               << " changed_layers=" << changed_layers
                               << " changed_regions=" << changed_regions
                               << " contracted_regions=" << contracted_regions

                               << " expanded_regions=" << expanded_regions
                               << " stolen_regions=" << stolen_regions;
    return true;
}

// ---- end Prusa Colorin S3 ----

// ---- Prusa Colorin S4: surface emboss mixed (Orca FullSpectrum v0.9.18 PrintObjectSlice.cpp 4978-5382) ----
static float emboss_surface_mixed_shell_override_delta(const LayerRegion &layerm, const ModelVolume &volume);

struct SurfaceEmbossMixedDebugCandidate
{
    const ModelVolume *volume { nullptr };
    int                region_id { -1 };
};

static bool has_surface_emboss_mixed_volume(const PrintObject &print_object)
{
    const Print *print = print_object.print();
    if (print == nullptr)
        return false;

    const size_t                num_physical = print->config().filament_diameter.size();
    const MixedFilamentManager &mixed_mgr    = print->mixed_filament_manager();
    for (const ModelVolume *volume : print_object.model_object()->volumes)
        if (volume->is_model_part() &&
            volume->emboss_shape.has_value() &&
            volume->emboss_shape->projection.use_surface &&
            mixed_mgr.is_mixed(unsigned(std::max(0, volume->extruder_id())), num_physical))
            return true;
    return false;
}

static std::string surface_emboss_mixed_debug_file_path(const PrintObject &print_object)
{
    return debug_out_path("emboss-mixed/obj-%d-debug.txt", int(print_object.id().id));
}

static void reset_surface_emboss_mixed_debug_file(const PrintObject &print_object)
{
    std::ofstream out(surface_emboss_mixed_debug_file_path(print_object), std::ios::out | std::ios::trunc);
    out << "surface emboss mixed debug"
        << " object_id=" << int(print_object.id().id)
        << " object_name=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
        << "\n";
}

static void append_surface_emboss_mixed_debug_line(const PrintObject &print_object, const std::string &line)
{
    std::ofstream out(surface_emboss_mixed_debug_file_path(print_object), std::ios::out | std::ios::app);
    out << line << '\n';
}

static std::vector<SurfaceEmbossMixedDebugCandidate> collect_surface_emboss_mixed_debug_candidates(
    const Layer                                  &layer,
    const PrintObjectRegions::LayerRangeRegions  &layer_range,
    const MixedFilamentManager                   &mixed_mgr,
    size_t                                        num_physical)
{
    std::vector<SurfaceEmbossMixedDebugCandidate> out;
    std::vector<int>                              processed_region_ids;
    processed_region_ids.reserve(layer_range.volume_regions.size());

    for (const PrintObjectRegions::VolumeRegion &volume_region : layer_range.volume_regions) {
        const ModelVolume *volume = volume_region.model_volume;
        if (volume == nullptr || !volume->is_model_part() || !volume->emboss_shape.has_value() || !volume->emboss_shape->projection.use_surface)
            continue;
        if (volume_region.region == nullptr)
            continue;

        const int region_id = volume_region.region->print_object_region_id();
        if (region_id < 0 || region_id >= layer.region_count())
            continue;
        if (std::find(processed_region_ids.begin(), processed_region_ids.end(), region_id) != processed_region_ids.end())
            continue;
        processed_region_ids.emplace_back(region_id);

        if (!mixed_mgr.is_mixed(unsigned(std::max(0, volume_region.region->config().perimeter_extruder.value)), num_physical))
            continue;

        out.push_back({ volume, region_id });
    }

    return out;
}

static void export_surface_emboss_mixed_layer_svg(
    const char                                          *stage,
    const PrintObject                                   &print_object,
    size_t                                               layer_id,
    const Layer                                         &layer,
    const std::vector<SurfaceEmbossMixedDebugCandidate> &candidates,
    const ExPolygons                                    *overlay,
    const std::string                                   &overlay_legend)
{
    std::vector<std::pair<ExPolygons, SVG::ExPolygonAttributes>> items;
    items.reserve(size_t(layer.region_count()) + ((overlay != nullptr && !overlay->empty()) ? 1 : 0));

    for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
        const LayerRegion *layerm = layer.get_region(region_id);
        if (layerm == nullptr || layerm->slices().empty())
            continue;

        ExPolygons expolygons = to_expolygons(layerm->slices().surfaces);
        if (expolygons.empty())
            continue;

        const bool is_candidate = std::find_if(candidates.begin(), candidates.end(), [region_id](const auto &candidate) {
            return candidate.region_id == region_id;
        }) != candidates.end();

        SVG::ExPolygonAttributes attrs(
            "region " + std::to_string(region_id) + " wall=" + std::to_string(layerm->region().config().perimeter_extruder.value),
            is_candidate ? "#3b82f6" : "#bfc5cc",
            is_candidate ? 0.35f : 0.14f);
        attrs.outline_width = scale_(0.05f);
        attrs.color_contour = is_candidate ? "blue" : "black";
        attrs.color_holes   = attrs.color_contour;
        items.emplace_back(std::move(expolygons), std::move(attrs));
    }

    if (overlay != nullptr && !overlay->empty()) {
        SVG::ExPolygonAttributes attrs(overlay_legend, "#ef4444", 0.28f);
        attrs.outline_width = scale_(0.05f);
        attrs.color_contour = "red";
        attrs.color_holes   = "red";
        items.emplace_back(*overlay, std::move(attrs));
    }

    if (!items.empty())
        SVG::export_expolygons(debug_out_path("emboss-mixed/obj-%d-layer-%03d-%s.svg",
                                              int(print_object.id().id),
                                              int(layer_id),
                                              stage),
                               items);
}

static void dump_surface_emboss_mixed_layer_state(
    const char                                          *stage,
    const PrintObject                                   &print_object,
    size_t                                               layer_id,
    const Layer                                         &layer,
    const PrintObjectRegions::LayerRangeRegions         &layer_range,
    const std::vector<ExPolygons>                       *segmentation_layer = nullptr)
{
    const Print *print = print_object.print();
    if (print == nullptr)
        return;

    const size_t                num_physical = print->config().filament_diameter.size();
    const MixedFilamentManager &mixed_mgr    = print->mixed_filament_manager();
    const std::vector<SurfaceEmbossMixedDebugCandidate> candidates =
        collect_surface_emboss_mixed_debug_candidates(layer, layer_range, mixed_mgr, num_physical);
    if (candidates.empty())
        return;

    std::ostringstream header;
    header << std::fixed << std::setprecision(4)
           << "stage=" << stage
           << " layer=" << layer_id
           << " print_z=" << layer.print_z
           << " slice_z=" << layer.slice_z
           << " regions=" << layer.region_count()
           << " candidates=" << candidates.size();
    append_surface_emboss_mixed_debug_line(print_object, header.str());

    for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
        const LayerRegion *layerm = layer.get_region(region_id);
        if (layerm == nullptr)
            continue;
        const double slice_area = layerm->slices().empty() ? 0.0 : std::abs(area(to_expolygons(layerm->slices().surfaces)));
        std::ostringstream line;
        line << std::fixed << std::setprecision(4)
             << "  region=" << region_id
             << " wall=" << layerm->region().config().perimeter_extruder.value
             << " sparse=" << layerm->region().config().infill_extruder.value
             << " solid=" << layerm->region().config().solid_infill_extruder.value
             << " area=" << slice_area;
        append_surface_emboss_mixed_debug_line(print_object, line.str());
    }

    for (const SurfaceEmbossMixedDebugCandidate &candidate : candidates) {
        const LayerRegion *layerm = layer.get_region(candidate.region_id);
        if (layerm == nullptr)
            continue;

        const double slice_area = layerm->slices().empty() ? 0.0 : std::abs(area(to_expolygons(layerm->slices().surfaces)));
        const float  shell_delta_scaled = emboss_surface_mixed_shell_override_delta(*layerm, *candidate.volume);
        std::ostringstream line;
        line << std::fixed << std::setprecision(4)
             << "  candidate region=" << candidate.region_id
             << " volume_name=" << candidate.volume->name
             << " volume_extruder=" << candidate.volume->extruder_id()
             << " cfg_wall=" << layerm->region().config().perimeter_extruder.value
             << " depth=" << float(candidate.volume->emboss_shape->projection.depth)
             << " shell_delta_mm=" << unscale<double>(shell_delta_scaled)
             << " area=" << slice_area;
        append_surface_emboss_mixed_debug_line(print_object, line.str());

        if (segmentation_layer != nullptr) {
            const int cfg_wall = layerm->region().config().perimeter_extruder.value;
            if (cfg_wall >= 1 && cfg_wall <= int(segmentation_layer->size())) {
                const double seg_area = std::abs(area((*segmentation_layer)[size_t(cfg_wall - 1)]));
                std::ostringstream seg_line;
                seg_line << std::fixed << std::setprecision(4)
                         << "    segmentation channel=" << cfg_wall
                         << " area=" << seg_area;
                append_surface_emboss_mixed_debug_line(print_object, seg_line.str());
            }
        }
    }

    export_surface_emboss_mixed_layer_svg(stage, print_object, layer_id, layer, candidates, nullptr, "");
}

static float emboss_surface_mixed_shell_override_delta(const LayerRegion &layerm, const ModelVolume &volume)
{
    if (!volume.emboss_shape.has_value() || !volume.emboss_shape->projection.use_surface)
        return 0.f;

    const float depth_mm = std::max(0.f, float(volume.emboss_shape->projection.depth));
    if (depth_mm <= EPSILON)
        return 0.f;

    const PrintRegionConfig &config = layerm.region().config();
    if (config.perimeters.value <= 0)
        return 0.f;

    const Flow   ext_flow        = layerm.flow(frExternalPerimeter);
    const Flow   perimeter_flow  = layerm.flow(frPerimeter);
    const coord_t shell_scaled   = ext_flow.scaled_width() / 2 +
                                   ext_flow.scaled_spacing() / 2 +
                                   std::max(0, config.perimeters.value - 1) * perimeter_flow.scaled_spacing();
    const float  shell_depth_mm  = float(unscale<double>(shell_scaled));
    const float  delta_mm        = std::max(0.f, shell_depth_mm - depth_mm);
    return delta_mm <= EPSILON ? 0.f : scaled<float>(delta_mm);
}

template<typename ThrowOnCancel>
static bool apply_surface_emboss_mixed_region_override(PrintObject &print_object, ThrowOnCancel throw_on_cancel)
{
    const Print *print = print_object.print();
    if (print == nullptr || print_object.layer_count() == 0 || print_object.shared_regions() == nullptr)
        return false;

    const size_t                 num_physical = print->config().filament_diameter.size();
    const MixedFilamentManager  &mixed_mgr    = print->mixed_filament_manager();
    const auto                  &volumes      = print_object.model_object()->volumes;
    if (num_physical == 0 ||
        std::find_if(volumes.begin(), volumes.end(), [&mixed_mgr, num_physical](const ModelVolume *volume) {
            return volume->is_model_part() &&
                   volume->emboss_shape.has_value() &&
                   volume->emboss_shape->projection.use_surface &&
                   mixed_mgr.is_mixed(unsigned(std::max(0, volume->extruder_id())), num_physical);
        }) == volumes.end())
        return false;

    const auto &layer_ranges = print_object.shared_regions()->layer_ranges;
    auto        it_layer_range = layer_range_first(layer_ranges, print_object.get_layer(0)->slice_z);

    size_t changed_layers  = 0;
    size_t changed_regions = 0;
    size_t stolen_regions  = 0;

    for (size_t layer_id = 0; layer_id < print_object.layer_count(); ++layer_id) {
        throw_on_cancel();

        Layer &layer = *print_object.get_layer(int(layer_id));
        it_layer_range = layer_range_next(layer_ranges, it_layer_range, layer.slice_z);
        const PrintObjectRegions::LayerRangeRegions &layer_range = *it_layer_range;
        const std::vector<SurfaceEmbossMixedDebugCandidate> candidates =
            collect_surface_emboss_mixed_debug_candidates(layer, layer_range, mixed_mgr, num_physical);
        if (!candidates.empty())
            dump_surface_emboss_mixed_layer_state("pre-emboss-override", print_object, layer_id, layer, layer_range);

        bool              layer_changed = false;
        ExPolygons        layer_masks;
        std::vector<int>  processed_region_ids;
        processed_region_ids.reserve(layer_range.volume_regions.size());

        for (const PrintObjectRegions::VolumeRegion &volume_region : layer_range.volume_regions) {
            const ModelVolume *volume = volume_region.model_volume;
            if (volume == nullptr || !volume->is_model_part() || !volume->emboss_shape.has_value() || !volume->emboss_shape->projection.use_surface)
                continue;
            if (volume_region.region == nullptr)
                continue;

            const int region_id = volume_region.region->print_object_region_id();
            if (region_id < 0 || region_id >= layer.region_count())
                continue;
            if (std::find(processed_region_ids.begin(), processed_region_ids.end(), region_id) != processed_region_ids.end())
                continue;
            processed_region_ids.emplace_back(region_id);

            const unsigned int filament_id = unsigned(std::max(0, volume_region.region->config().perimeter_extruder.value));
            if (!mixed_mgr.is_mixed(filament_id, num_physical))
                continue;

            LayerRegion *emboss_layerm = layer.get_region(region_id);
            if (emboss_layerm == nullptr || emboss_layerm->slices().empty())
                continue;

            ExPolygons override_mask = to_expolygons(emboss_layerm->slices().surfaces);
            if (override_mask.empty())
                continue;

            if (const float delta_scaled = emboss_surface_mixed_shell_override_delta(*emboss_layerm, *volume);
                delta_scaled > float(EPSILON)) {
                override_mask = offset_ex(override_mask, delta_scaled);
                if (override_mask.empty())
                    continue;
                if (layer_masks.empty())
                    layer_masks = collect_layer_region_slices(layer);
                if (!layer_masks.empty())
                    override_mask = intersection_ex(override_mask, layer_masks);
                if (override_mask.empty())
                    continue;
            }

            {
                std::ostringstream line;
                line << std::fixed << std::setprecision(4)
                     << "stage=override-mask"
                     << " layer=" << layer_id
                     << " region=" << region_id
                     << " volume_name=" << volume->name
                     << " volume_extruder=" << volume->extruder_id()
                     << " cfg_wall=" << volume_region.region->config().perimeter_extruder.value
                     << " depth=" << float(volume->emboss_shape->projection.depth)
                     << " shell_delta_mm=" << unscale<double>(emboss_surface_mixed_shell_override_delta(*emboss_layerm, *volume))
                     << " mask_area=" << std::abs(area(override_mask));
                append_surface_emboss_mixed_debug_line(print_object, line.str());
            }
            const std::string overlay_stage = "override-mask-r" + std::to_string(region_id);
            export_surface_emboss_mixed_layer_svg(overlay_stage.c_str(),
                                                  print_object,
                                                  layer_id,
                                                  layer,
                                                  candidates,
                                                  &override_mask,
                                                  "override mask");

            ExPolygons emboss_slices = to_expolygons(emboss_layerm->slices().surfaces);
            bool       emboss_changed = false;

            for (int target_region_id = 0; target_region_id < layer.region_count(); ++target_region_id) {
                if (target_region_id == region_id)
                    continue;

                LayerRegion *target_layerm = layer.get_region(target_region_id);
                if (target_layerm == nullptr || target_layerm->slices().empty())
                    continue;
                if (target_layerm->region().config().perimeter_extruder.value == int(filament_id))
                    continue;

                ExPolygons stolen = intersection_ex(target_layerm->slices().surfaces, override_mask);
                if (stolen.empty())
                    continue;

                append(emboss_slices, stolen);
                emboss_changed = true;
                ++stolen_regions;

                std::ostringstream line;
                line << std::fixed << std::setprecision(4)
                     << "stage=override-steal"
                     << " layer=" << layer_id
                     << " emboss_region=" << region_id
                     << " from_region=" << target_region_id
                     << " from_wall=" << target_layerm->region().config().perimeter_extruder.value
                     << " stolen_area=" << std::abs(area(stolen));
                append_surface_emboss_mixed_debug_line(print_object, line.str());

                Polygons remaining = diff(to_polygons(target_layerm->slices().surfaces), override_mask);
                if (!remaining.empty())
                    remaining = opening(union_ex(remaining), scaled<float>(5. * EPSILON), scaled<float>(5. * EPSILON));
                const_cast<SurfaceCollection&>(target_layerm->slices()).set(union_ex(remaining), stInternal);
                layer_changed = true;
            }

            if (emboss_changed) {
                if (emboss_slices.size() > 1)
                    emboss_slices = closing_ex(to_polygons(emboss_slices), scaled<float>(10. * EPSILON));
                const_cast<SurfaceCollection&>(emboss_layerm->slices()).set(std::move(emboss_slices), stInternal);
                ++changed_regions;
                layer_changed = true;
            } else {
                std::ostringstream line;
                line << "stage=override-no-steal"
                     << " layer=" << layer_id
                     << " region=" << region_id
                     << " volume_name=" << volume->name;
                append_surface_emboss_mixed_debug_line(print_object, line.str());
            }
        }

        if (!candidates.empty())
            dump_surface_emboss_mixed_layer_state("post-emboss-override", print_object, layer_id, layer, layer_range);

        if (layer_changed)
            ++changed_layers;
    }

    if (changed_regions == 0)
        return false;

    BOOST_LOG_TRIVIAL(warning) << "Surface emboss mixed-region override applied"
                               << " object=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
                               << " changed_layers=" << changed_layers
                               << " changed_regions=" << changed_regions
                               << " stolen_regions=" << stolen_regions;
    return true;
}

// ---- end Prusa Colorin S4 ----

// ---- Prusa Colorin Local-Z parte 1: alturas de subcapa y pares activos (Orca FullSpectrum v0.9.18 PrintObjectSlice.cpp 1212-2315) ----
// Copia de ImageMap::allocate_adaptive_local_z_heights (Orca 0.9.18 ImageMap/AdaptiveLocalZRenderer.cpp 21-86) hasta portar el renderer.
static constexpr double COLORIN_ADAPTIVE_LOCAL_Z_MAX_HEIGHT_MM = 0.32;
static std::vector<double> colorin_allocate_adaptive_local_z_heights(const std::vector<double>& weights,
                                                      double                     total_height,
                                                      double                     configured_minimum)
{
    std::vector<double> heights(weights.size(), 0.0);
    if (weights.empty() || total_height <= EPSILON)
        return heights;

    // A valid print profile keeps the nominal layer height below this ceiling,
    // so an N-pass cadence always has enough total capacity. Preserve the
    // model envelope for a malformed profile instead of silently losing Z.
    const double maximum = std::max(COLORIN_ADAPTIVE_LOCAL_Z_MAX_HEIGHT_MM, total_height / double(weights.size()));
    const double minimum = std::min(maximum, std::max(0.01, configured_minimum));
    const double effective_minimum = minimum * double(weights.size()) <= total_height + EPSILON
        ? minimum
        : total_height / double(weights.size());

    const double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0, [](double sum, double weight) {
        return sum + std::max(0.0, weight);
    });
    for (size_t component_idx = 0; component_idx < weights.size(); ++component_idx) {
        const double desired = total_weight > EPSILON
            ? total_height * std::max(0.0, weights[component_idx]) / total_weight
            : total_height / double(weights.size());
        heights[component_idx] = std::clamp(desired, effective_minimum, maximum);
    }

    // Clamping one component changes the height still available to its
    // siblings. Redistribute that difference without changing the fixed cycle
    // top; zero-weight siblings share unavoidable excess equally.
    for (int pass = 0; pass < 2 * int(weights.size()) + 2; ++pass) {
        const double assigned = std::accumulate(heights.begin(), heights.end(), 0.0);
        const double delta    = total_height - assigned;
        if (std::abs(delta) <= 1e-10)
            break;

        const bool increase = delta > 0.0;
        std::vector<size_t> candidates;
        double              candidate_weight = 0.0;
        for (size_t component_idx = 0; component_idx < heights.size(); ++component_idx) {
            const double capacity = increase ? maximum - heights[component_idx]
                                             : heights[component_idx] - effective_minimum;
            if (capacity <= EPSILON)
                continue;
            candidates.emplace_back(component_idx);
            candidate_weight += std::max(0.0, weights[component_idx]);
        }
        if (candidates.empty())
            break;

        double applied = 0.0;
        for (size_t component_idx : candidates) {
            const double share = candidate_weight > EPSILON
                ? std::max(0.0, weights[component_idx]) / candidate_weight
                : 1.0 / double(candidates.size());
            const double capacity = increase ? maximum - heights[component_idx]
                                             : heights[component_idx] - effective_minimum;
            const double adjustment = std::min(std::abs(delta) * share, capacity);
            heights[component_idx] += increase ? adjustment : -adjustment;
            applied += adjustment;
        }
        if (applied <= 1e-10)
            break;
    }
    return heights;
}

static bool fit_pass_heights_to_interval(std::vector<double> &passes, double base_height, double lo, double hi)
{
    if (passes.empty() || base_height <= EPSILON)
        return false;

    double sum = std::accumulate(passes.begin(), passes.end(), 0.0);
    double delta = base_height - sum;

    auto within = [lo, hi](double h) { return h >= lo - EPSILON && h <= hi + EPSILON; };
    if (std::abs(delta) > EPSILON) {
        if (within(passes.back() + delta)) {
            passes.back() += delta;
            delta = 0.0;
        } else if (delta > 0.0) {
            for (size_t i = passes.size(); i > 0 && delta > EPSILON; --i) {
                double &h = passes[i - 1];
                const double room = hi - h;
                if (room <= EPSILON)
                    continue;
                const double take = std::min(room, delta);
                h += take;
                delta -= take;
            }
        } else {
            for (size_t i = passes.size(); i > 0 && delta < -EPSILON; --i) {
                double &h = passes[i - 1];
                const double room = h - lo;
                if (room <= EPSILON)
                    continue;
                const double take = std::min(room, -delta);
                h -= take;
                delta += take;
            }
        }
    }

    if (std::abs(delta) > 1e-6)
        return false;
    return std::all_of(passes.begin(), passes.end(), within);
}

static bool sanitize_local_z_pass_heights(std::vector<double> &passes, double base_height, double min_sublayer_height)
{
    if (passes.empty() || base_height <= EPSILON)
        return false;

    const double lo = std::max<double>(0.01, min_sublayer_height);
    const double hi = std::max<double>(lo, base_height);
    for (double &h : passes) {
        if (!std::isfinite(h))
            h = lo;
        h = std::clamp(h, lo, hi);
    }
    return fit_pass_heights_to_interval(passes, base_height, lo, hi);
}

static std::vector<double> build_uniform_local_z_pass_heights(double base_height,
                                                              double min_sublayer_height,
                                                              size_t max_passes_limit = 0)
{
    std::vector<double> out;
    if (base_height <= EPSILON)
        return out;

    const double lo = std::max<double>(0.01, min_sublayer_height);
    const double hi = std::max<double>(lo, base_height);
    size_t min_passes = size_t(std::max<double>(1.0, std::ceil((base_height - EPSILON) / hi)));
    size_t max_passes = size_t(std::max<double>(1.0, std::floor((base_height + EPSILON) / lo)));
    size_t pass_count = min_passes;

    if (max_passes >= min_passes) {
        const double target_step = 0.5 * (lo + hi);
        const size_t target_passes =
            size_t(std::max<double>(1.0, std::llround(base_height / std::max<double>(target_step, EPSILON))));
        pass_count = std::clamp(target_passes, min_passes, max_passes);
    }

    if (max_passes_limit > 0) {
        const size_t capped_limit = std::max<size_t>(1, max_passes_limit);
        if (pass_count > capped_limit)
            pass_count = capped_limit;
    }

    if (pass_count == 1 && base_height >= 2.0 * lo - EPSILON && max_passes >= 2)
        pass_count = 2;

    if (pass_count <= 1) {
        out.emplace_back(base_height);
        return out;
    }

    const double uniform_height = base_height / double(pass_count);
    out.assign(pass_count, uniform_height);

    // Keep the accumulated numeric error at the very top of the interval.
    double accumulated = 0.0;
    for (size_t i = 0; i + 1 < out.size(); ++i)
        accumulated += out[i];
    out.back() = std::max<double>(EPSILON, base_height - accumulated);
    return out;
}

static std::vector<double> build_uniform_local_z_pass_heights_exact(double base_height,
                                                                    double min_sublayer_height,
                                                                    size_t pass_count)
{
    if (base_height <= EPSILON || pass_count == 0)
        return {};

    const double lo = std::max<double>(0.01, min_sublayer_height);
    const double hi = std::max<double>(lo, base_height);
    if (pass_count == 1)
        return { base_height };

    if (double(pass_count) * lo > base_height + EPSILON || double(pass_count) * hi < base_height - EPSILON)
        return {};

    std::vector<double> out(pass_count, base_height / double(pass_count));
    if (!fit_pass_heights_to_interval(out, base_height, lo, hi))
        return {};
    return out;
}

static inline void compute_local_z_component_heights(int mix_b_percent,
                                                     double base_height,
                                                     double min_sublayer_height,
                                                     double &h_a,
                                                     double &h_b)
{
    const auto heights = mixed_filament_local_z_pair_heights(base_height, min_sublayer_height, mix_b_percent);
    h_a               = heights.first;
    h_b               = heights.second;
}

static bool choose_local_z_start_with_component_a(const std::vector<double> &pass_heights,
                                                  double                     expected_h_a,
                                                  double                     expected_h_b,
                                                  size_t                     cadence_index)
{
    double err_ab = 0.0;
    double err_ba = 0.0;
    for (size_t pass_i = 0; pass_i < pass_heights.size(); ++pass_i) {
        const double expected_ab = (pass_i % 2) == 0 ? expected_h_a : expected_h_b;
        const double expected_ba = (pass_i % 2) == 0 ? expected_h_b : expected_h_a;
        err_ab += std::abs(pass_heights[pass_i] - expected_ab);
        err_ba += std::abs(pass_heights[pass_i] - expected_ba);
    }

    if (err_ab + 1e-6 < err_ba)
        return true;
    if (err_ba + 1e-6 < err_ab)
        return false;

    // When the requested component heights are equal (for example 50/50),
    // either A/B or B/A is numerically identical. Preserve the existing
    // row cadence so equal-split layers keep the normal local-Z A/B/A/B
    // sequence instead of flipping AB|BA between nominal layers.
    if (std::abs(expected_h_a - expected_h_b) <= 1e-6) {
        return (cadence_index % 2) == 0;
    }

    return expected_h_a >= expected_h_b;
}

static std::vector<double> build_local_z_alternating_pass_heights(double base_height,
                                                                   double min_sublayer_height,
                                                                   double gradient_h_a,
                                                                   double gradient_h_b,
                                                                   size_t max_passes_limit = 0)
{
    if (base_height <= EPSILON)
        return {};

    const double minimum = std::max<double>(0.01, min_sublayer_height);
    if (gradient_h_a <= EPSILON || gradient_h_b <= EPSILON ||
        base_height < 2.0 * minimum - EPSILON ||
        (max_passes_limit > 0 && max_passes_limit < 2))
        return { base_height };

    const double cycle_h = std::max<double>(EPSILON, gradient_h_a + gradient_h_b);
    const double ratio_a = std::clamp(gradient_h_a / cycle_h, 0.0, 1.0);
    const double height_a = std::clamp(base_height * ratio_a, minimum, base_height - minimum);
    return {height_a, base_height - height_a};
}

static std::vector<double> build_local_z_two_pass_heights(double base_height,
                                                          double min_sublayer_height,
                                                          double gradient_h_a,
                                                          double gradient_h_b)
{
    if (base_height <= EPSILON)
        return {};

    const double lo = std::max<double>(0.01, min_sublayer_height);
    const double hi = std::max<double>(lo, base_height);
    if (gradient_h_a <= EPSILON || gradient_h_b <= EPSILON || base_height < 2.0 * lo - EPSILON)
        return { base_height };

    const double cycle_h = std::max<double>(EPSILON, gradient_h_a + gradient_h_b);
    const double ratio_a = std::clamp(gradient_h_a / cycle_h, 0.0, 1.0);

    const double h_a_min = std::max(lo, base_height - hi);
    const double h_a_max = std::min(hi, base_height - lo);
    if (h_a_min > h_a_max + EPSILON)
        return { base_height };

    const double h_a = std::clamp(base_height * ratio_a, h_a_min, h_a_max);
    const double h_b = base_height - h_a;

    std::vector<double> out { h_a, h_b };
    if (!fit_pass_heights_to_interval(out, base_height, lo, hi))
        return { base_height };
    return out;
}

static std::vector<double> build_local_z_shared_pass_heights(double base_height, double min_sublayer_height)
{
    if (base_height <= EPSILON)
        return {};

    const double lo = std::max<double>(0.01, min_sublayer_height);
    const double hi = std::max<double>(lo, base_height);
    if (base_height < 2.0 * lo - EPSILON)
        return { base_height };

    // In shared (dense multi-zone) mode keep a single pair of pass planes for
    // the whole nominal layer, anchored to the configured minimum height.
    double h_small = lo;
    double h_large = base_height - h_small;
    if (h_large > hi + EPSILON) {
        h_large = hi;
        h_small = base_height - h_large;
    }
    if (h_small < lo - EPSILON || h_small > hi + EPSILON ||
        h_large < lo - EPSILON || h_large > hi + EPSILON)
        return build_uniform_local_z_pass_heights(base_height, lo);

    std::vector<double> out { h_small, h_large };
    if (!fit_pass_heights_to_interval(out, base_height, lo, hi))
        return build_uniform_local_z_pass_heights(base_height, lo);
    if (out.size() == 2 && out[0] > out[1])
        std::swap(out[0], out[1]);
    return out;
}

static std::vector<double> build_local_z_pass_heights(double base_height,
                                                      double min_sublayer_height,
                                                      double preferred_a,
                                                      double preferred_b,
                                                      size_t max_passes_limit = 0)
{
    if (base_height <= EPSILON)
        return {};

    const double lo = std::max<double>(0.01, min_sublayer_height);
    const double hi = std::max<double>(lo, base_height);

    std::vector<double> cadence_unit;
    if (preferred_a > EPSILON)
        cadence_unit.push_back(std::clamp(preferred_a, lo, hi));
    if (preferred_b > EPSILON)
        cadence_unit.push_back(std::clamp(preferred_b, lo, hi));

    if (!cadence_unit.empty()) {
        std::vector<double> out;
        out.reserve(size_t(std::ceil(base_height / lo)) + 2);

        double z_used = 0.0;
        size_t idx = 0;
        size_t guard = 0;
        while (z_used + cadence_unit[idx] < base_height - EPSILON && guard++ < 100000) {
            out.push_back(cadence_unit[idx]);
            z_used += cadence_unit[idx];
            idx = (idx + 1) % cadence_unit.size();
        }

        const double remainder = base_height - z_used;
        if (remainder > EPSILON)
            out.push_back(remainder);

        if (fit_pass_heights_to_interval(out, base_height, lo, hi) &&
            (max_passes_limit == 0 || out.size() <= max_passes_limit))
            return out;

        if (max_passes_limit > 0 && preferred_a > EPSILON && preferred_b > EPSILON)
            return build_local_z_alternating_pass_heights(base_height,
                                                          min_sublayer_height,
                                                          preferred_a,
                                                          preferred_b,
                                                          max_passes_limit);
    }

    return build_uniform_local_z_pass_heights(base_height, lo, max_passes_limit);
}

static std::vector<unsigned int> decode_manual_pattern_sequence(const MixedFilamentDefinition &definition, size_t num_physical)
{
    return mixed_filament_manual_pattern_sequence(definition, num_physical);
}

static std::vector<unsigned int> decode_blend_component_ids(const MixedFilamentDefinition &definition, size_t num_physical)
{
    return definition.recipe.blend.component_ids(num_physical);
}

static std::vector<int> decode_blend_component_weights(const MixedFilamentDefinition &definition,
                                                       size_t                         num_physical,
                                                       size_t                         expected_components)
{
    if (expected_components == 0)
        return {};

    std::vector<int> out = definition.recipe.blend.component_percents(num_physical);
    if (out.size() != expected_components)
        return {};

    int sum = 0;
    for (const int v : out)
        sum += std::max(0, v);
    if (sum <= 0)
        return {};
    return out;
}

static void reduce_weight_counts_to_cycle_limit(std::vector<int> &counts, size_t cycle_limit)
{
    if (counts.empty() || cycle_limit == 0)
        return;

    int total = std::accumulate(counts.begin(), counts.end(), 0);
    if (total <= 0 || size_t(total) <= cycle_limit)
        return;

    std::vector<size_t> positive_indices;
    positive_indices.reserve(counts.size());
    for (size_t i = 0; i < counts.size(); ++i)
        if (counts[i] > 0)
            positive_indices.emplace_back(i);

    if (positive_indices.empty()) {
        counts.assign(counts.size(), 0);
        return;
    }

    std::vector<int> reduced(counts.size(), 0);
    if (cycle_limit < positive_indices.size()) {
        std::sort(positive_indices.begin(), positive_indices.end(), [&counts](size_t lhs, size_t rhs) {
            if (counts[lhs] != counts[rhs])
                return counts[lhs] > counts[rhs];
            return lhs < rhs;
        });
        for (size_t i = 0; i < cycle_limit; ++i)
            reduced[positive_indices[i]] = 1;
        counts = std::move(reduced);
        return;
    }

    size_t remaining_slots = cycle_limit;
    for (const size_t idx : positive_indices) {
        reduced[idx] = 1;
        --remaining_slots;
    }

    int total_extras = 0;
    std::vector<int> extra_counts(counts.size(), 0);
    for (const size_t idx : positive_indices) {
        extra_counts[idx] = std::max(0, counts[idx] - 1);
        total_extras += extra_counts[idx];
    }
    if (remaining_slots == 0 || total_extras <= 0) {
        counts = std::move(reduced);
        return;
    }

    std::vector<double> remainders(counts.size(), -1.0);
    size_t assigned_slots = 0;
    for (const size_t idx : positive_indices) {
        if (extra_counts[idx] == 0)
            continue;
        const double exact = double(remaining_slots) * double(extra_counts[idx]) / double(total_extras);
        const int assigned = int(std::floor(exact));
        reduced[idx] += assigned;
        assigned_slots += size_t(assigned);
        remainders[idx] = exact - double(assigned);
    }

    size_t missing_slots = remaining_slots > assigned_slots ? (remaining_slots - assigned_slots) : size_t(0);
    while (missing_slots > 0) {
        size_t best_idx = size_t(-1);
        double best_remainder = -1.0;
        int    best_extra = -1;
        for (const size_t idx : positive_indices) {
            if (extra_counts[idx] == 0)
                continue;
            if (remainders[idx] > best_remainder ||
                (std::abs(remainders[idx] - best_remainder) <= 1e-9 && extra_counts[idx] > best_extra) ||
                (std::abs(remainders[idx] - best_remainder) <= 1e-9 && extra_counts[idx] == best_extra && idx < best_idx)) {
                best_idx = idx;
                best_remainder = remainders[idx];
                best_extra = extra_counts[idx];
            }
        }
        if (best_idx == size_t(-1))
            break;
        ++reduced[best_idx];
        remainders[best_idx] = -1.0;
        --missing_slots;
    }

    counts = std::move(reduced);
}

static std::vector<unsigned int> build_weighted_gradient_sequence(const std::vector<unsigned int> &ids,
                                                                  const std::vector<int>          &weights,
                                                                  size_t                           max_cycle_limit = 0)
{
    if (ids.empty())
        return {};

    std::vector<unsigned int> filtered_ids;
    std::vector<int>          counts;
    filtered_ids.reserve(ids.size());
    counts.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        const int w = (i < weights.size()) ? std::max(0, weights[i]) : 0;
        if (w <= 0)
            continue;
        filtered_ids.emplace_back(ids[i]);
        counts.emplace_back(w);
    }
    if (filtered_ids.empty()) {
        filtered_ids = ids;
        counts.assign(ids.size(), 1);
    }

    int g = 0;
    for (const int c : counts)
        g = std::gcd(g, std::max(1, c));
    if (g > 1) {
        for (int &c : counts)
            c = std::max(1, c / g);
    }

    constexpr size_t k_max_cycle = 48;
    const size_t effective_cycle_limit =
        max_cycle_limit > 0 ? std::min(k_max_cycle, std::max<size_t>(1, max_cycle_limit)) : k_max_cycle;
    reduce_weight_counts_to_cycle_limit(counts, effective_cycle_limit);

    std::vector<unsigned int> reduced_ids;
    std::vector<int>          reduced_counts;
    reduced_ids.reserve(filtered_ids.size());
    reduced_counts.reserve(counts.size());
    for (size_t i = 0; i < counts.size(); ++i) {
        if (counts[i] <= 0)
            continue;
        reduced_ids.emplace_back(filtered_ids[i]);
        reduced_counts.emplace_back(counts[i]);
    }
    if (reduced_ids.empty())
        return {};
    filtered_ids = std::move(reduced_ids);
    counts = std::move(reduced_counts);

    const int total = std::accumulate(counts.begin(), counts.end(), 0);
    if (total <= 0)
        return {};

    const size_t cycle = size_t(total);

    std::vector<unsigned int> sequence;
    sequence.reserve(cycle);
    std::vector<int> emitted(counts.size(), 0);
    for (size_t pos = 0; pos < cycle; ++pos) {
        size_t best_idx = 0;
        double best_score = -1e9;
        for (size_t i = 0; i < counts.size(); ++i) {
            const double target = double(pos + 1) * double(counts[i]) / double(total);
            const double score = target - double(emitted[i]);
            if (score > best_score) {
                best_score = score;
                best_idx = i;
            }
        }
        ++emitted[best_idx];
        sequence.emplace_back(filtered_ids[best_idx]);
    }
    return sequence;
}

static bool local_z_eligible_mixed_definition(const MixedFilamentDefinition &definition)
{
    // Local-Z flow-height modulation should apply to all mixed rows that are
    // resolved as A/B blends on model surfaces, not only custom rows.
    return !definition.visibility.tombstoned && !definition.recipe.manual_pattern;
}

static bool local_z_gradient_mixed_definition(const MixedFilamentDefinition &definition)
{
    if (!local_z_eligible_mixed_definition(definition) || !definition.behavior.gradient.enabled)
        return false;

    const std::optional<MixedFilamentPrimaryPairView> pair = definition.recipe.blend.primary_pair();
    return pair && pair->component_a.id != pair->component_b.id;
}

static std::vector<int> fill_continuous_layer_range(std::vector<int> layers)
{
    if (layers.empty())
        return {};

    std::sort(layers.begin(), layers.end());
    layers.erase(std::unique(layers.begin(), layers.end()), layers.end());

    std::vector<int> out;
    out.reserve(size_t(layers.back() - layers.front() + 1));
    for (int layer = layers.front(); layer <= layers.back(); ++layer)
        out.emplace_back(layer);
    return out;
}

static std::vector<double> build_local_z_gradient_stop_positions(const MixedFilamentDefinition &definition,
                                                                 size_t                         num_physical,
                                                                 size_t                         component_count)
{
    const size_t expected_stops = component_count >= 2 ? 2 * component_count - 1 : size_t(0);
    if (expected_stops < 3)
        return {};

    std::vector<double> out;
    const std::vector<float> &explicit_positions = definition.behavior.gradient.stop_positions;
    if (explicit_positions.size() == expected_stops) {
        out.reserve(expected_stops);
        for (const float position : explicit_positions) {
            if (!std::isfinite(position)) {
                out.clear();
                break;
            }
            out.emplace_back(std::clamp(double(position), 0.0, 1.0));
        }
    }

    if (out.empty() && component_count >= 3) {
        const std::vector<int> weights = definition.recipe.blend.component_percents(num_physical);
        if (weights.size() == component_count) {
            double total = 0.0;
            for (const int weight : weights)
                total += double(std::max(0, weight));
            if (total > EPSILON) {
                std::vector<double> boundaries(component_count + 1, 0.0);
                for (size_t i = 0; i < component_count; ++i)
                    boundaries[i + 1] = boundaries[i] + double(std::max(0, weights[i])) / total;
                boundaries.front() = 0.0;
                boundaries.back() = 1.0;

                out.assign(expected_stops, 0.0);
                out.front() = 0.0;
                out.back() = 1.0;
                for (size_t i = 0; i + 1 < component_count; ++i)
                    out[2 * i + 1] = std::clamp(boundaries[i + 1], 0.0, 1.0);
                for (size_t i = 1; i + 1 < component_count; ++i)
                    out[2 * i] = std::clamp((boundaries[i] + boundaries[i + 1]) * 0.5, 0.0, 1.0);
            }
        }
    }

    if (out.size() != expected_stops)
        return {};

    out.front() = 0.0;
    out.back() = 1.0;
    constexpr double min_gap = 1e-6;
    for (size_t i = 1; i < out.size(); ++i)
        out[i] = std::max(out[i], out[i - 1] + min_gap);
    out.back() = 1.0;
    for (size_t i = out.size() - 1; i > 0; --i)
        out[i - 1] = std::min(out[i - 1], out[i] - min_gap);
    out.front() = 0.0;
    for (double &position : out)
        position = std::clamp(position, 0.0, 1.0);
    return out;
}

static bool local_z_direct_multicolor_definition(const MixedFilamentDefinition &definition,
                                                 size_t                         num_physical,
                                                 std::vector<unsigned int>     *component_ids = nullptr,
                                                 std::vector<int>              *component_weights = nullptr)
{
    if (!local_z_eligible_mixed_definition(definition) || definition.behavior.gradient.enabled)
        return false;

    const std::vector<unsigned int> ids = decode_blend_component_ids(definition, num_physical);
    if (ids.size() < 3)
        return false;

    if (component_ids != nullptr)
        *component_ids = ids;
    if (component_weights != nullptr) {
        std::vector<int> weights = decode_blend_component_weights(definition, num_physical, ids.size());
        if (weights.empty())
            weights.assign(ids.size(), 1);
        *component_weights = std::move(weights);
    }
    return true;
}

struct LocalZActivePair
{
    unsigned int component_a = 0;
    unsigned int component_b = 0;
    int          mix_b_percent = 50;
    bool         uses_layer_cycle_sequence = false;
    bool         single_component = false;

    bool valid_pair(size_t num_physical) const
    {
        return component_a > 0 && component_a <= num_physical &&
               component_b > 0 && component_b <= num_physical &&
               (component_a != component_b || single_component);
    }
};

static int local_z_mix_b_percent_from_fraction(double b_fraction)
{
    return std::clamp(int(std::lround(100.0 * std::clamp(b_fraction, 0.0, 1.0))), 1, 99);
}

static void local_z_orient_pair_to_follow_previous(LocalZActivePair &pair, unsigned int previous_extruder)
{
    if (pair.single_component)
        return;

    // Gradient Local-Z rows use B/A pass order. If the previous emitted pass
    // was B, flip the pair so the next nominal layer starts with the other
    // component and does not create a doubled physical layer at the boundary.
    if (previous_extruder != 0 && previous_extruder == pair.component_b && previous_extruder != pair.component_a) {
        std::swap(pair.component_a, pair.component_b);
        pair.mix_b_percent = std::clamp(100 - pair.mix_b_percent, 1, 99);
    }
}

static bool local_z_gradient_active_pair_for_progress(const std::vector<unsigned int>& ids,
                                                      const std::vector<double>&       stop_positions,
                                                      double                           progress,
                                                      double                           middle_window_fraction,
                                                      LocalZActivePair&                pair_out,
                                                      const std::vector<float>&        solid_widths)
{
    if (ids.size() < 2 || (ids.size() == 2 && stop_positions.size() != 3))
        return false;
    MixedFilamentDefinition definition;
    definition.behavior.gradient.enabled = true;
    definition.behavior.gradient.stop_positions.assign(stop_positions.begin(), stop_positions.end());
    definition.behavior.gradient.solid_widths = solid_widths;
    for (unsigned int id : ids)
        definition.recipe.blend.components.push_back({{id}, 1});
    const auto sample         = sample_mixed_gradient(definition, 0, progress, middle_window_fraction);
    pair_out = LocalZActivePair{};
    pair_out.component_a      = sample.component_a;
    pair_out.component_b      = sample.component_b;
    pair_out.mix_b_percent    = sample.mix_b_percent;
    pair_out.single_component = sample.component_a == sample.component_b;
    return sample.component_a != 0 && sample.component_b != 0;
}

static void append_local_z_pair_option(std::vector<LocalZActivePair> &out,
                                       unsigned int                   component_a,
                                       unsigned int                   component_b,
                                       int                            weight_a,
                                       int                            weight_b)
{
    if (component_a == 0 || component_b == 0 || component_a == component_b)
        return;

    LocalZActivePair pair;
    pair.component_a = component_a;
    pair.component_b = component_b;
    pair.uses_layer_cycle_sequence = true;

    const int safe_weight_a = std::max(0, weight_a);
    const int safe_weight_b = std::max(0, weight_b);
    const int pair_total    = std::max(1, safe_weight_a + safe_weight_b);
    pair.mix_b_percent =
        std::clamp(int(std::lround(100.0 * double(safe_weight_b) / double(pair_total))), 1, 99);
    out.emplace_back(pair);
}

static std::vector<LocalZActivePair> build_local_z_pair_cycle_for_definition(const MixedFilamentDefinition &definition, size_t num_physical)
{
    std::vector<LocalZActivePair> pair_options;
    if (definition.visibility.tombstoned || num_physical == 0 || definition.behavior.gradient.enabled)
        return pair_options;

    const std::vector<unsigned int> blend_ids = definition.recipe.blend.component_ids(num_physical);
    if (blend_ids.size() < 3)
        return pair_options;

    std::vector<int> gradient_weights = decode_blend_component_weights(definition, num_physical, blend_ids.size());
    if (gradient_weights.empty())
        gradient_weights.assign(blend_ids.size(), 1);

    std::vector<int> pair_weights;
    if (blend_ids.size() >= 4) {
        append_local_z_pair_option(pair_options, blend_ids[0], blend_ids[1], gradient_weights[0], gradient_weights[1]);
        append_local_z_pair_option(pair_options, blend_ids[2], blend_ids[3], gradient_weights[2], gradient_weights[3]);
        pair_weights.emplace_back(std::max(1, gradient_weights[0] + gradient_weights[1]));
        pair_weights.emplace_back(std::max(1, gradient_weights[2] + gradient_weights[3]));
    } else {
        append_local_z_pair_option(pair_options, blend_ids[0], blend_ids[1], gradient_weights[0], gradient_weights[1]);
        append_local_z_pair_option(pair_options, blend_ids[0], blend_ids[2], gradient_weights[0], gradient_weights[2]);
        append_local_z_pair_option(pair_options, blend_ids[1], blend_ids[2], gradient_weights[1], gradient_weights[2]);
        pair_weights.emplace_back(std::max(1, gradient_weights[0] + gradient_weights[1]));
        pair_weights.emplace_back(std::max(1, gradient_weights[0] + gradient_weights[2]));
        pair_weights.emplace_back(std::max(1, gradient_weights[1] + gradient_weights[2]));
    }

    if (pair_options.size() < 2 || pair_options.size() != pair_weights.size())
        return {};

    std::vector<unsigned int> pair_ids(pair_options.size(), 0);
    for (size_t idx = 0; idx < pair_ids.size(); ++idx)
        pair_ids[idx] = unsigned(idx + 1);

    const size_t max_pair_layers =
        definition.behavior.local_z.max_sublayers >= 2 ?
            std::max<size_t>(1, size_t(definition.behavior.local_z.max_sublayers) / 2) :
            size_t(0);
    const std::vector<unsigned int> pair_sequence = build_weighted_gradient_sequence(pair_ids, pair_weights, max_pair_layers);
    if (pair_sequence.empty())
        return {};

    std::vector<LocalZActivePair> out;
    out.reserve(pair_sequence.size());
    for (const unsigned int pair_token : pair_sequence) {
        if (pair_token < 1 || pair_token > pair_options.size())
            continue;
        out.emplace_back(pair_options[size_t(pair_token - 1)]);
    }
    return out;
}

static std::vector<double> build_local_z_direct_multicolor_pass_heights(const MixedFilamentDefinition &definition,
                                                                        const std::vector<int>       &component_weights,
                                                                        double                        base_height,
                                                                        double                        min_sublayer_height,
                                                                        size_t                        component_count)
{
    if (base_height <= EPSILON || component_count == 0)
        return {};

    const double lo = std::max<double>(0.01, min_sublayer_height);
    const double hi = std::max<double>(lo, base_height);
    const size_t min_passes = size_t(std::max<double>(1.0, std::ceil((base_height - EPSILON) / hi)));
    const size_t max_passes = size_t(std::max<double>(1.0, std::floor((base_height + EPSILON) / lo)));
    if (max_passes == 0)
        return { base_height };

    size_t pass_limit = max_passes;
    if (definition.behavior.local_z.max_sublayers >= 2)
        pass_limit = std::min(pass_limit, size_t(std::max(2, definition.behavior.local_z.max_sublayers)));
    pass_limit = std::max(pass_limit, min_passes);

    std::vector<int> positive_weights;
    positive_weights.reserve(component_weights.size());
    for (const int weight : component_weights)
        if (weight > 0)
            positive_weights.emplace_back(weight);
    if (positive_weights.empty())
        positive_weights.assign(component_count, 1);

    const int total_weight = std::max(1, std::accumulate(positive_weights.begin(), positive_weights.end(), 0));
    std::vector<double> component_targets;
    component_targets.reserve(positive_weights.size());
    size_t ideal_passes = 0;
    for (const int weight : positive_weights) {
        const double target = base_height * double(weight) / double(total_weight);
        component_targets.emplace_back(target);
        ideal_passes += size_t(std::max<double>(1.0, std::ceil((target - EPSILON) / hi)));
    }

    const size_t preferred_passes = std::max(component_targets.size(), ideal_passes);
    const size_t desired_passes   = std::min(std::max(preferred_passes, min_passes), pass_limit);

    std::vector<double> bins = component_targets;
    while (bins.size() > desired_passes) {
        std::sort(bins.begin(), bins.end());
        const double merged = bins[0] + bins[1];
        bins.erase(bins.begin(), bins.begin() + 2);
        bins.emplace_back(merged);
    }

    while (bins.size() < desired_passes) {
        auto it = std::max_element(bins.begin(), bins.end());
        if (it == bins.end())
            break;
        const double value = *it;
        if (value < 2.0 * lo - EPSILON)
            break;
        double first = std::clamp(value * 0.5, lo, value - lo);
        double second = value - first;
        if (first < lo - EPSILON || second < lo - EPSILON)
            break;
        *it = first;
        bins.emplace_back(second);
    }

    if (bins.empty())
        return build_uniform_local_z_pass_heights(base_height, lo, desired_passes);

    std::sort(bins.begin(), bins.end(), std::greater<double>());
    for (double &value : bins)
        value = std::clamp(value, lo, hi);
    if (fit_pass_heights_to_interval(bins, base_height, lo, hi))
        return bins;

    for (size_t pass_count = desired_passes; pass_count >= min_passes; --pass_count) {
        std::vector<double> exact = build_uniform_local_z_pass_heights_exact(base_height, min_sublayer_height, pass_count);
        if (!exact.empty())
            return exact;
        if (pass_count == min_passes)
            break;
    }

    return build_uniform_local_z_pass_heights(base_height, lo, desired_passes);
}

static std::vector<unsigned int> build_local_z_direct_multicolor_sequence(const std::vector<unsigned int> &component_ids,
                                                                          const std::vector<int>          &component_weights,
                                                                          const std::vector<double>       &pass_heights,
                                                                          std::vector<double>            &carry_error_mm)
{
    if (component_ids.empty() || pass_heights.empty())
        return {};

    std::vector<unsigned int> filtered_ids;
    std::vector<int>          filtered_weights;
    filtered_ids.reserve(component_ids.size());
    filtered_weights.reserve(component_ids.size());
    for (size_t idx = 0; idx < component_ids.size(); ++idx) {
        const int weight = idx < component_weights.size() ? std::max(0, component_weights[idx]) : 0;
        if (weight <= 0)
            continue;
        filtered_ids.emplace_back(component_ids[idx]);
        filtered_weights.emplace_back(weight);
    }
    if (filtered_ids.empty()) {
        filtered_ids = component_ids;
        filtered_weights.assign(component_ids.size(), 1);
    }
    if (filtered_ids.empty())
        return {};

    if (carry_error_mm.size() != filtered_ids.size())
        carry_error_mm.assign(filtered_ids.size(), 0.0);

    const double total_height = std::accumulate(pass_heights.begin(), pass_heights.end(), 0.0);
    const int total_weight = std::max(1, std::accumulate(filtered_weights.begin(), filtered_weights.end(), 0));

    std::vector<double> desired_heights(filtered_ids.size(), 0.0);
    for (size_t idx = 0; idx < filtered_ids.size(); ++idx)
        desired_heights[idx] = total_height * double(filtered_weights[idx]) / double(total_weight) + carry_error_mm[idx];

    std::vector<double>       assigned_heights(filtered_ids.size(), 0.0);
    std::vector<unsigned int> sequence;
    sequence.reserve(pass_heights.size());
    int previous_choice = -1;

    for (const double pass_height : pass_heights) {
        size_t best_idx   = 0;
        double best_score = -std::numeric_limits<double>::infinity();
        double best_need  = -std::numeric_limits<double>::infinity();
        for (size_t idx = 0; idx < filtered_ids.size(); ++idx) {
            const double remaining_need = desired_heights[idx] - assigned_heights[idx];
            double       score          = remaining_need;
            if (int(idx) == previous_choice)
                score -= 0.35 * pass_height;

            if (score > best_score + 1e-9 ||
                (std::abs(score - best_score) <= 1e-9 &&
                 (remaining_need > best_need + 1e-9 ||
                  (std::abs(remaining_need - best_need) <= 1e-9 && filtered_ids[idx] < filtered_ids[best_idx])))) {
                best_idx   = idx;
                best_score = score;
                best_need  = remaining_need;
            }
        }

        assigned_heights[best_idx] += pass_height;
        previous_choice = int(best_idx);
        sequence.emplace_back(filtered_ids[best_idx]);
    }

    for (size_t idx = 0; idx < filtered_ids.size(); ++idx)
        carry_error_mm[idx] = desired_heights[idx] - assigned_heights[idx];

    const double error_sum = std::accumulate(carry_error_mm.begin(), carry_error_mm.end(), 0.0);
    if (!carry_error_mm.empty() && std::abs(error_sum) > 1e-9) {
        const double correction = error_sum / double(carry_error_mm.size());
        for (double &value : carry_error_mm)
            value -= correction;
    }

    return sequence;
}

struct LocalZIndependentDirectPass
{
    unsigned int extruder_id { 0 };
    double       height { 0.0 };
};

static double local_z_max_layer_height_for_extruder(const PrintConfig &print_config, unsigned int extruder_id)
{
    const size_t extruder_idx = extruder_id > 0 ? size_t(extruder_id - 1) : 0;
    const double configured_max =
        print_config.max_layer_height.values.empty() ? 0.0 : print_config.max_layer_height.get_at(extruder_idx);
    const double nozzle_diameter =
        print_config.nozzle_diameter.values.empty() ? 0.4 : print_config.nozzle_diameter.get_at(extruder_idx);
    return std::max(0.01, configured_max > EPSILON ? configured_max : 0.75 * nozzle_diameter);
}

static std::vector<LocalZIndependentDirectPass> build_local_z_independent_direct_cadence(
    const std::vector<unsigned int> &component_ids,
    const std::vector<int>          &component_weights,
    double                           min_sublayer_height,
    const PrintConfig               &print_config)
{
    std::vector<LocalZIndependentDirectPass> cadence;
    if (component_ids.empty())
        return cadence;

    int minimum_positive_weight = std::numeric_limits<int>::max();
    for (size_t idx = 0; idx < component_ids.size(); ++idx) {
        const int weight = idx < component_weights.size() ? component_weights[idx] : 0;
        if (component_ids[idx] > 0 && weight > 0)
            minimum_positive_weight = std::min(minimum_positive_weight, weight);
    }
    if (minimum_positive_weight == std::numeric_limits<int>::max())
        return cadence;

    const double minimum = std::max<double>(0.01, min_sublayer_height);
    for (size_t idx = 0; idx < component_ids.size(); ++idx) {
        const int weight = idx < component_weights.size() ? component_weights[idx] : 0;
        if (component_ids[idx] == 0 || weight <= 0)
            continue;

        const double total_component_height =
            minimum * double(weight) / double(minimum_positive_weight);
        const double max_layer_height =
            local_z_max_layer_height_for_extruder(print_config, component_ids[idx]);
        const size_t split_count = std::max<size_t>(
            1,
            size_t(std::ceil(total_component_height / max_layer_height - EPSILON)));
        const double split_height = total_component_height / double(split_count);
        for (size_t split_idx = 0; split_idx < split_count; ++split_idx)
            cadence.push_back(LocalZIndependentDirectPass{component_ids[idx], split_height});
    }
    return cadence;
}

static double adaptive_local_z_full_resolution_height(const std::vector<unsigned int> &component_ids,
                                                      const std::vector<int>          &component_weights,
                                                      double                           nominal_layer_height)
{
    if (component_ids.size() < 2 || component_weights.size() != component_ids.size())
        return 0.0;

    int total_weight    = 0;
    int smallest_weight = std::numeric_limits<int>::max();
    for (size_t component_idx = 0; component_idx < component_ids.size(); ++component_idx) {
        const int weight = component_weights[component_idx];
        if (component_ids[component_idx] == 0 || weight <= 0)
            continue;
        total_weight    += weight;
        smallest_weight = std::min(smallest_weight, weight);
    }
    if (total_weight <= 0 || smallest_weight == std::numeric_limits<int>::max())
        return 0.0;

    return nominal_layer_height * double(smallest_weight) / double(total_weight);
}

static std::vector<double> adaptive_local_z_bounded_heights(const std::vector<unsigned int> &component_ids,
                                                            const std::vector<int>          &component_weights,
                                                            double                           nominal_layer_height,
                                                            double                           min_sublayer_height)
{
    std::vector<double> heights(component_ids.size(), 0.0);
    if (component_ids.empty() || component_weights.size() != component_ids.size() || nominal_layer_height <= EPSILON)
        return heights;

    std::vector<size_t> active;
    active.reserve(component_ids.size());
    for (size_t component_idx = 0; component_idx < component_ids.size(); ++component_idx)
        if (component_ids[component_idx] != 0 && component_weights[component_idx] > 0)
            active.emplace_back(component_idx);
    if (active.empty())
        return heights;

    std::vector<double> active_weights;
    active_weights.reserve(active.size());
    for (size_t component_idx : active)
        active_weights.emplace_back(double(component_weights[component_idx]));
    const std::vector<double> active_heights =
        colorin_allocate_adaptive_local_z_heights(active_weights, nominal_layer_height, min_sublayer_height);
    for (size_t active_idx = 0; active_idx < active.size(); ++active_idx)
        heights[active[active_idx]] = active_heights[active_idx];
    return heights;
}

static std::vector<LocalZIndependentDirectPass> build_adaptive_local_z_modulated_cadence(
    const std::vector<unsigned int> &component_ids,
    const std::vector<int>          &component_weights,
    double                           nominal_layer_height,
    double                           min_sublayer_height,
    const PrintConfig               &print_config)
{
    std::vector<LocalZIndependentDirectPass> cadence;
    if (component_ids.size() < 2 || component_weights.size() != component_ids.size())
        return cadence;

    const std::vector<double> component_heights = adaptive_local_z_bounded_heights(component_ids,
                                                                                    component_weights,
                                                                                    nominal_layer_height,
                                                                                    min_sublayer_height);
    for (size_t component_idx = 0; component_idx < component_ids.size(); ++component_idx) {
        const unsigned int component_id = component_ids[component_idx];
        const double       component_height = component_heights[component_idx];
        if (component_id == 0 || component_height <= EPSILON)
            continue;
        cadence.push_back(LocalZIndependentDirectPass{component_id, component_height});
    }
    (void)print_config;
    return cadence;
}

static void append_local_z_independent_component_height(std::vector<LocalZIndependentDirectPass> &cadence,
                                                        unsigned int                               component_id,
                                                        double                                     total_height,
                                                        const PrintConfig                         &print_config)
{
    if (component_id == 0 || total_height <= EPSILON)
        return;

    const double max_layer_height = local_z_max_layer_height_for_extruder(print_config, component_id);
    const size_t split_count = std::max<size_t>(
        1,
        size_t(std::ceil(total_height / max_layer_height - EPSILON)));
    const double split_height = total_height / double(split_count);
    for (size_t split_idx = 0; split_idx < split_count; ++split_idx)
        cadence.push_back(LocalZIndependentDirectPass{component_id, split_height});
}

static std::vector<LocalZIndependentDirectPass> build_local_z_independent_gradient_cadence(const MixedGradientLocalZSample& sample,
                                                                                           double             available_height,
                                                                                           double             min_sublayer_height,
                                                                                           unsigned int       previous_extruder,
                                                                                           const PrintConfig& print_config)
{
    std::vector<LocalZIndependentDirectPass> cadence;
    if (available_height <= EPSILON || sample.mix.component_a == 0 || sample.mix.component_b == 0)
        return cadence;

    LocalZActivePair pair;
    pair.component_a      = sample.mix.component_a;
    pair.component_b      = sample.mix.component_b;
    pair.mix_b_percent    = sample.mix.mix_b_percent;
    pair.single_component = pair.component_a == pair.component_b;
    double height_a       = sample.height_a;
    double height_b       = sample.height_b;
    if (height_a + height_b > available_height) {
        const auto clipped = mixed_filament_local_z_pair_heights(available_height, min_sublayer_height, pair.mix_b_percent);
        height_a           = clipped.first;
        height_b           = clipped.second;
    }
    local_z_orient_pair_to_follow_previous(pair, previous_extruder);
    if (pair.component_a != sample.mix.component_a)
        std::swap(height_a, height_b);
    // Retain the established B/A order and per-extruder maximum-height splitting.
    append_local_z_independent_component_height(cadence, pair.component_b, height_b, print_config);
    append_local_z_independent_component_height(cadence, pair.component_a, height_a, print_config);
    return cadence;
}

static LocalZActivePair derive_local_z_active_pair(const MixedFilamentDefinition     &definition,
                                                   const std::vector<LocalZActivePair> &pair_cycle,
                                                   size_t                              num_physical,
                                                   int                                 cadence_index)
{
    LocalZActivePair out;

    if (!pair_cycle.empty()) {
        const int cycle_i = int(pair_cycle.size());
        const size_t pos  = size_t(((cadence_index % cycle_i) + cycle_i) % cycle_i);
        return pair_cycle[pos];
    }

    const MixedFilamentPrimaryPairView pair = definition.recipe.blend.primary_pair_or(0, 0);
    out.component_a = pair.component_a.id;
    out.component_b = pair.component_b.id;
    out.mix_b_percent = pair.component_b_percent;
    out.uses_layer_cycle_sequence = false;
    return out;
}

// ---- end Prusa Colorin Local-Z parte 1 ----

// ---- Prusa Colorin Local-Z parte 2: plan Local-Z por capa (Orca FullSpectrum v0.9.18 PrintObjectSlice.cpp 2469-4579) ----
// En Prusa la PrintConfig ya contiene estas opciones aplicadas desde el config completo: se usa el valor de print_cfg.
static bool bool_from_full_config(const PrintConfig &, const char *, bool fallback) { return fallback; }
static coordf_t float_from_full_config(const PrintConfig &, const char *, coordf_t fallback) { return fallback; }

static void export_local_z_plan_debug(const PrintObject &print_object, coordf_t min_sublayer_height)
{
    const std::vector<LocalZInterval> &intervals = print_object.local_z_intervals();
    const std::vector<SubLayerPlan>   &plans     = print_object.local_z_sublayer_plan();
    if (intervals.empty() || plans.empty())
        return;

    const int object_id = int(print_object.id().id);
    std::ofstream json(debug_out_path("local-z-plan-obj-%d.json", object_id), std::ios::out | std::ios::trunc);
    if (json.good()) {
        json << std::fixed << std::setprecision(6);
        json << "{\n";
        json << "  \"object_id\": " << object_id << ",\n";
        json << "  \"min_sublayer_height\": " << min_sublayer_height << ",\n";
        json << "  \"interval_count\": " << intervals.size() << ",\n";
        json << "  \"sublayer_count\": " << plans.size() << ",\n";
        json << "  \"intervals\": [\n";
        for (size_t i = 0; i < intervals.size(); ++i) {
            const LocalZInterval &interval = intervals[i];
            json << "    {\"layer_id\": " << interval.layer_id
                 << ", \"z_lo\": " << interval.z_lo
                 << ", \"z_hi\": " << interval.z_hi
                 << ", \"base_height\": " << interval.base_height
                 << ", \"sublayer_height\": " << interval.sublayer_height
                 << ", \"has_mixed_paint\": " << (interval.has_mixed_paint ? "true" : "false")
                 << ", \"sublayer_count\": " << interval.sublayer_count << "}";
            if (i + 1 < intervals.size())
                json << ",";
            json << "\n";
        }
        json << "  ],\n";
        json << "  \"sublayers\": [\n";
        for (size_t i = 0; i < plans.size(); ++i) {
            const SubLayerPlan &plan = plans[i];
            json << "    {\"layer_id\": " << plan.layer_id
                 << ", \"pass_index\": " << plan.pass_index
                 << ", \"split_interval\": " << (plan.split_interval ? "true" : "false")
                 << ", \"z_lo\": " << plan.z_lo
                 << ", \"z_hi\": " << plan.z_hi
                 << ", \"print_z\": " << plan.print_z
                 << ", \"flow_height\": " << plan.flow_height
                 << ", \"base_mask_count\": " << plan.base_masks.size()
                 << ", \"painted_mask_counts\": [";
            for (size_t eidx = 0; eidx < plan.painted_masks_by_extruder.size(); ++eidx) {
                json << plan.painted_masks_by_extruder[eidx].size();
                if (eidx + 1 < plan.painted_masks_by_extruder.size())
                    json << ", ";
            }
            json << "]}";
            if (i + 1 < plans.size())
                json << ",";
            json << "\n";
        }
        json << "  ]\n";
        json << "}\n";
    }

    static const std::array<const char *, 10> colors {
        "#E53935", "#1E88E5", "#43A047", "#FB8C00", "#8E24AA",
        "#00897B", "#6D4C41", "#3949AB", "#C0CA33", "#F4511E"
    };
    for (const SubLayerPlan &plan : plans) {
        bool has_painted = std::any_of(plan.painted_masks_by_extruder.begin(), plan.painted_masks_by_extruder.end(),
                                       [](const ExPolygons &masks) { return !masks.empty(); });
        if (!plan.split_interval && !has_painted)
            continue;
        if (!has_painted && plan.base_masks.empty())
            continue;

        std::vector<std::pair<ExPolygons, SVG::ExPolygonAttributes>> layers;
        if (!plan.base_masks.empty()) {
            layers.emplace_back(plan.base_masks, SVG::ExPolygonAttributes("base", "#D6D6D6", "#6A6A6A", "#6A6A6A", scale_(0.03), 0.45f));
        }
        for (size_t eidx = 0; eidx < plan.painted_masks_by_extruder.size(); ++eidx) {
            if (plan.painted_masks_by_extruder[eidx].empty())
                continue;
            const char *color = colors[eidx % colors.size()];
            layers.emplace_back(plan.painted_masks_by_extruder[eidx],
                                SVG::ExPolygonAttributes("E" + std::to_string(eidx + 1), color, color, color, scale_(0.03), 0.55f));
        }
        if (!layers.empty()) {
            SVG::export_expolygons(debug_out_path("local-z-plan-obj-%d-layer-%d-pass-%d.svg", object_id, int(plan.layer_id), int(plan.pass_index)), layers);
        }
    }
}

static std::vector<uint8_t> local_z_enabled_mixed_rows(const PrintObject &print_object,
                                                       std::vector<uint8_t> &adaptive_independent_rows,
                                                       double &adaptive_full_resolution_height_mm)
{
    const Print              &print     = *print_object.print();
    const PrintConfig        &print_cfg = print.config();
    const PrintConfig &full_cfg  = print.config();
    const bool global_local_z =
        bool_from_full_config(full_cfg, "dithering_local_z_mode", print_cfg.dithering_local_z_mode.value);
    const size_t num_physical = print_cfg.filament_colour.size();
    const std::vector<MixedFilamentDefinition> definitions =
        print.mixed_filament_manager().mixed_filament_definitions(num_physical);

    std::vector<uint8_t> enabled_rows(definitions.size(), uint8_t(0));
    adaptive_independent_rows.assign(definitions.size(), uint8_t(0));
    adaptive_full_resolution_height_mm = 0.;
    for (size_t row_idx = 0; row_idx < definitions.size(); ++row_idx) {
        if (mixed_filament_definition_uses_local_z(definitions[row_idx], global_local_z))
            enabled_rows[row_idx] = uint8_t(1);
    }

    const double nominal_layer_height = std::max(0.01, print_object.config().layer_height.value);
    const double minimum_sublayer_height = std::max(
        0.01,
        double(float_from_full_config(full_cfg,
                                      "mixed_filament_height_lower_bound",
                                      coordf_t(print_cfg.mixed_filament_height_lower_bound.value))));
    const MixedFilamentManager &mixed_mgr = print.mixed_filament_manager();
    const ModelObject *model_object = print_object.model_object();
    if (model_object == nullptr)
        return enabled_rows;
    for (const ModelVolume *volume : model_object->volumes) {
        if (volume == nullptr || !volume->is_model_part())
            continue;
        const std::shared_ptr<const ImageMap::VolumeData> data = volume->image_map_data();
        if (!data || data->empty())
            continue;

        std::vector<uint8_t> referenced_zones(data->zones.size(), uint8_t(0));
        for (const ImageMap::TriangleBinding &binding : data->triangle_bindings)
            if (binding.zone_index < referenced_zones.size())
                referenced_zones[binding.zone_index] = uint8_t(1);

        for (size_t zone_idx = 0; zone_idx < data->zones.size(); ++zone_idx) {
            const ImageMap::Zone &zone = data->zones[zone_idx];
            if (referenced_zones[zone_idx] == 0 || !zone.enabled ||
                zone.render_mode != ImageMap::RenderMode::AdaptiveLocalizedCycles ||
                zone.adaptive_modulation_mode != ImageMap::AdaptiveModulationMode::LocalZHeight)
                continue;

            for (const ImageMap::PaletteEntry &entry : zone.palette) {
                unsigned int filament_id = 0;
                if (entry.mixed_filament_stable_id != 0) {
                    const std::optional<unsigned int> resolved =
                        mixed_mgr.filament_id_from_stable_id(entry.mixed_filament_stable_id, num_physical);
                    if (resolved)
                        filament_id = *resolved;
                }
                if (filament_id == 0)
                    filament_id = entry.fallback_filament_id;

                const int mixed_idx = mixed_mgr.mixed_index_from_filament_id(filament_id, num_physical);
                if (mixed_idx < 0 || size_t(mixed_idx) >= definitions.size())
                    continue;
                const size_t row_idx = size_t(mixed_idx);
                enabled_rows[row_idx]              = uint8_t(1);
                adaptive_independent_rows[row_idx] = uint8_t(1);

                const std::vector<unsigned int> component_ids =
                    definitions[row_idx].recipe.blend.component_ids(num_physical);
                const std::vector<int> component_weights =
                    definitions[row_idx].recipe.blend.component_percents(num_physical);
                const double full_resolution_height =
                    adaptive_local_z_full_resolution_height(component_ids,
                                                            component_weights,
                                                            nominal_layer_height * double(component_ids.size()));
                if (full_resolution_height <= EPSILON)
                    continue;
                if (full_resolution_height + EPSILON < minimum_sublayer_height &&
                    (adaptive_full_resolution_height_mm <= EPSILON ||
                     full_resolution_height < adaptive_full_resolution_height_mm)) {
                    adaptive_full_resolution_height_mm = full_resolution_height;
                }
            }
        }
    }

    return enabled_rows;
}

static std::vector<std::vector<ExPolygons>> whole_object_local_z_segmentation_by_mixed_wall(
    const PrintObject &print_object, const std::vector<uint8_t> &local_z_enabled_rows)
{
    std::vector<std::vector<ExPolygons>> segmentation;

    const Print *print = print_object.print();
    if (print == nullptr || print_object.layer_count() == 0)
        return segmentation;

    const size_t num_physical = print->config().filament_colour.size();
    const size_t num_total    = print->mixed_filament_manager().total_filaments(num_physical);
    if (num_total <= num_physical)
        return segmentation;

    segmentation.assign(print_object.layer_count(), std::vector<ExPolygons>(num_total + 1));
    const MixedFilamentManager &mixed_mgr = print->mixed_filament_manager();
    size_t mixed_region_layers = 0;
    size_t mixed_region_count  = 0;

    for (size_t layer_id = 0; layer_id < print_object.layer_count(); ++layer_id) {
        const Layer &layer = *print_object.get_layer(int(layer_id));
        bool layer_has_mixed_region = false;
        for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
            const LayerRegion *layerm = layer.get_region(region_id);
            if (layerm == nullptr || layerm->slices().empty())
                continue;

            const unsigned int filament_id = unsigned(std::max(0, layerm->region().config().perimeter_extruder.value));
            if (!mixed_mgr.is_mixed(filament_id, num_physical))
                continue;
            const int mixed_idx = mixed_mgr.mixed_index_from_filament_id(filament_id, num_physical);
            if (mixed_idx < 0 || size_t(mixed_idx) >= local_z_enabled_rows.size() ||
                local_z_enabled_rows[size_t(mixed_idx)] == 0)
                continue;
            if (filament_id >= segmentation[layer_id].size())
                continue;

            ExPolygons state_masks = to_expolygons(layerm->slices().surfaces);
            if (state_masks.empty())
                continue;

            append(segmentation[layer_id][filament_id], std::move(state_masks));
            layer_has_mixed_region = true;
            ++mixed_region_count;
        }

        if (layer_has_mixed_region) {
            ++mixed_region_layers;
            for (size_t channel_idx = num_physical + 1; channel_idx < segmentation[layer_id].size(); ++channel_idx) {
                ExPolygons &state_masks = segmentation[layer_id][channel_idx];
                if (state_masks.size() > 1)
                    state_masks = union_ex(state_masks);
            }
        }
    }

    if (mixed_region_count == 0)
        return {};

    BOOST_LOG_TRIVIAL(info) << "Local-Z whole-object wall segmentation prepared"
                            << " object=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
                            << " mixed_region_layers=" << mixed_region_layers
                            << " mixed_region_count=" << mixed_region_count
                            << " physical_filaments=" << num_physical
                            << " total_filaments=" << num_total;
    return segmentation;
}

static std::vector<std::vector<ExPolygons>> local_z_planner_segmentation_with_whole_object_mixed_wall(
    const PrintObject                          &print_object,
    const std::vector<std::vector<ExPolygons>> &paint_segmentation,
    const std::vector<uint8_t>                 &local_z_enabled_rows,
    bool                                       &has_whole_object_scope)
{
    has_whole_object_scope = false;
    const Print *print = print_object.print();
    if (print == nullptr || paint_segmentation.empty())
        return paint_segmentation;

    const size_t num_physical = print->config().filament_colour.size();
    const MixedFilamentManager &mixed_mgr = print->mixed_filament_manager();
    size_t excluded_physical_channels = 0;
    size_t excluded_mixed_channels    = 0;
    std::vector<std::vector<ExPolygons>> planner_paint_segmentation(paint_segmentation.size());
    for (size_t layer_id = 0; layer_id < paint_segmentation.size(); ++layer_id) {
        planner_paint_segmentation[layer_id].resize(paint_segmentation[layer_id].size());
        for (size_t channel_idx = 1; channel_idx < paint_segmentation[layer_id].size(); ++channel_idx) {
            const ExPolygons &state_masks = paint_segmentation[layer_id][channel_idx];
            if (state_masks.empty())
                continue;

            const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
            const bool state_is_mixed = mixed_mgr.is_mixed(state_id, num_physical);
            bool mixed_row_uses_local_z = false;
            if (state_is_mixed) {
                const int mixed_idx = mixed_mgr.mixed_index_from_filament_id(state_id, num_physical);
                mixed_row_uses_local_z = mixed_idx >= 0 && size_t(mixed_idx) < local_z_enabled_rows.size() &&
                                         local_z_enabled_rows[size_t(mixed_idx)] != 0;
            }
            if (!mixed_filament_local_z_painted_override_uses_planner(state_is_mixed, mixed_row_uses_local_z)) {
                if (state_is_mixed)
                    ++excluded_mixed_channels;
                else
                    ++excluded_physical_channels;
                continue;
            }
            planner_paint_segmentation[layer_id][channel_idx] = state_masks;
        }
    }

    std::vector<std::vector<ExPolygons>> augmented =
        whole_object_local_z_segmentation_by_mixed_wall(print_object, local_z_enabled_rows);
    if (augmented.empty())
        return planner_paint_segmentation;
    has_whole_object_scope = true;

    size_t overlay_layers              = 0;
    size_t overlay_mixed_channels      = 0;
    size_t physical_override_layers    = 0;

    for (size_t layer_id = 0; layer_id < augmented.size() && layer_id < paint_segmentation.size(); ++layer_id) {
        if (augmented[layer_id].size() < paint_segmentation[layer_id].size())
            augmented[layer_id].resize(paint_segmentation[layer_id].size());

        ExPolygons painted_overrides;
        for (size_t channel_idx = 1; channel_idx < paint_segmentation[layer_id].size(); ++channel_idx) {
            const ExPolygons &state_masks = paint_segmentation[layer_id][channel_idx];
            if (!state_masks.empty())
                append(painted_overrides, state_masks);
        }
        if (painted_overrides.size() > 1)
            painted_overrides = union_ex(painted_overrides);

        bool layer_has_overlay = false;
        if (!painted_overrides.empty()) {
            bool clipped_for_physical_override = false;
            for (size_t channel_idx = num_physical + 1; channel_idx < augmented[layer_id].size(); ++channel_idx) {
                ExPolygons &state_masks = augmented[layer_id][channel_idx];
                if (state_masks.empty())
                    continue;
                const ExPolygons clipped_masks = diff_ex(state_masks, painted_overrides);
                if (clipped_masks.size() != state_masks.size())
                    clipped_for_physical_override = true;
                state_masks = clipped_masks;
            }
            if (clipped_for_physical_override)
                ++physical_override_layers;
            layer_has_overlay = true;
        }

        for (size_t channel_idx = 1; channel_idx < planner_paint_segmentation[layer_id].size(); ++channel_idx) {
            const ExPolygons &state_masks = planner_paint_segmentation[layer_id][channel_idx];
            if (state_masks.empty())
                continue;

            const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
            if (channel_idx >= augmented[layer_id].size())
                augmented[layer_id].resize(channel_idx + 1);

            append(augmented[layer_id][channel_idx], state_masks);
            layer_has_overlay = true;
            if (mixed_mgr.is_mixed(state_id, num_physical))
                ++overlay_mixed_channels;
        }

        for (size_t channel_idx = num_physical + 1; channel_idx < augmented[layer_id].size(); ++channel_idx) {
            ExPolygons &state_masks = augmented[layer_id][channel_idx];
            if (state_masks.size() > 1)
                state_masks = union_ex(state_masks);
        }
        if (layer_has_overlay)
            ++overlay_layers;
    }

    if (overlay_layers > 0) {
        BOOST_LOG_TRIVIAL(info) << "Local-Z planner merged whole-object mixed wall masks with painted overrides"
                                << " object=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
                                << " overlay_layers=" << overlay_layers
                                << " overlay_mixed_channels=" << overlay_mixed_channels
                                << " physical_override_layers=" << physical_override_layers
                                << " excluded_physical_channels=" << excluded_physical_channels
                                << " excluded_mixed_channels=" << excluded_mixed_channels;
    }
    return augmented;
}

static std::vector<ExPolygons> collect_local_z_fixed_state_masks_by_extruder(const std::vector<ExPolygons> &layer_segmentation,
                                                                             const size_t                   num_physical)
{
    std::vector<ExPolygons> masks_by_extruder(num_physical);
    for (size_t channel_idx = 1; channel_idx < layer_segmentation.size(); ++channel_idx) {
        const ExPolygons &state_masks = layer_segmentation[channel_idx];
        if (state_masks.empty())
            continue;

        const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
        if (state_id == 0 || state_id > num_physical)
            continue;
        append(masks_by_extruder[state_id - 1], state_masks);
    }
    for (ExPolygons &masks : masks_by_extruder)
        if (masks.size() > 1)
            masks = union_ex(masks);
    return masks_by_extruder;
}

static std::vector<ExPolygons> build_local_z_transition_fixed_masks_for_pass(
    const std::vector<ExPolygons> &current_masks_by_extruder,
    const std::vector<ExPolygons> &prev_masks_by_extruder,
    const std::vector<ExPolygons> &next_masks_by_extruder,
    const size_t                   pass_idx,
    const size_t                   pass_count)
{
    if (pass_count <= 1)
        return current_masks_by_extruder;

    std::vector<ExPolygons> pass_masks_by_extruder(current_masks_by_extruder.size());
    const bool is_lowest_pass = pass_idx == 0;
    const bool is_highest_pass = pass_idx + 1 >= pass_count;

    for (size_t extruder_idx = 0; extruder_idx < current_masks_by_extruder.size(); ++extruder_idx) {
        const ExPolygons &current_masks = current_masks_by_extruder[extruder_idx];
        if (current_masks.empty())
            continue;

        const ExPolygons prev_masks = extruder_idx < prev_masks_by_extruder.size() ? prev_masks_by_extruder[extruder_idx] : ExPolygons();
        const ExPolygons next_masks = extruder_idx < next_masks_by_extruder.size() ? next_masks_by_extruder[extruder_idx] : ExPolygons();

        const ExPolygons current_and_prev = prev_masks.empty() ? ExPolygons() : intersection_ex(current_masks, prev_masks);
        const ExPolygons current_and_next = next_masks.empty() ? ExPolygons() : intersection_ex(current_masks, next_masks);
        const ExPolygons persistent =
            current_and_prev.empty() || current_and_next.empty() ? ExPolygons() : intersection_ex(current_and_prev, current_and_next);

        ExPolygons entering = current_and_next;
        if (!entering.empty() && !current_and_prev.empty())
            entering = diff_ex(entering, current_and_prev);

        ExPolygons exiting = current_and_prev;
        if (!exiting.empty() && !current_and_next.empty())
            exiting = diff_ex(exiting, current_and_next);

        ExPolygons covered;
        if (!persistent.empty())
            append(covered, persistent);
        if (!entering.empty())
            append(covered, entering);
        if (!exiting.empty())
            append(covered, exiting);
        if (covered.size() > 1)
            covered = union_ex(covered);

        const ExPolygons isolated = covered.empty() ? current_masks : diff_ex(current_masks, covered);

        ExPolygons assigned;
        if (!persistent.empty())
            append(assigned, persistent);
        if (is_lowest_pass && !exiting.empty())
            append(assigned, exiting);
        if (is_highest_pass) {
            if (!entering.empty())
                append(assigned, entering);
            if (!isolated.empty())
                append(assigned, isolated);
        }
        if (assigned.size() > 1)
            assigned = union_ex(assigned);
        pass_masks_by_extruder[extruder_idx] = std::move(assigned);
    }

    return pass_masks_by_extruder;
}

template<typename ThrowOnCancel>
static void build_local_z_plan(PrintObject &print_object,
                               const std::vector<std::vector<ExPolygons>> &segmentation,
                               bool whole_object_scope_active,
                               const std::vector<uint8_t> &enabled_rows,
                               const std::vector<uint8_t> &adaptive_independent_rows,
                               ThrowOnCancel throw_on_cancel)
{
    print_object.clear_local_z_plan();

    const Print *print = print_object.print();
    const std::string object_name = print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>");
    if (print == nullptr || print_object.layer_count() == 0 || segmentation.size() != print_object.layer_count()) {
        BOOST_LOG_TRIVIAL(debug) << "Local-Z plan skipped: invalid preconditions"
                                 << " object=" << object_name
                                 << " print_ptr=" << (print != nullptr)
                                 << " layer_count=" << print_object.layer_count()
                                 << " segmentation_layers=" << segmentation.size();
        return;
    }

    const PrintConfig &full_cfg  = print->config();
    const PrintConfig        &print_cfg = print->config();
    const bool local_z_mode = bool_from_full_config(full_cfg, "dithering_local_z_mode", print_cfg.dithering_local_z_mode.value);
    const bool global_full_domain =
        bool_from_full_config(full_cfg, "dithering_local_z_whole_objects", print_cfg.dithering_local_z_whole_objects.value);
    const bool local_z_whole_objects =
        mixed_filament_local_z_uses_full_domain(global_full_domain, whole_object_scope_active);
    const bool local_z_preserve_first_layer =
        bool_from_full_config(full_cfg,
                              "dithering_local_z_preserve_first_layer",
                              print_cfg.dithering_local_z_preserve_first_layer.value);
    const bool local_z_direct_multicolor =
        bool_from_full_config(full_cfg, "dithering_local_z_direct_multicolor",
                              print_cfg.dithering_local_z_direct_multicolor.value);
    const bool local_z_independent_layer_height =
        local_z_direct_multicolor &&
        bool_from_full_config(full_cfg, "dithering_local_z_independent_layer_height",
                              print_cfg.dithering_local_z_independent_layer_height.value);
    coordf_t min_sublayer_height = float_from_full_config(full_cfg, "mixed_filament_height_lower_bound",
                                                          coordf_t(print_cfg.mixed_filament_height_lower_bound.value));
    coordf_t preferred_a = float_from_full_config(full_cfg, "mixed_color_layer_height_a",
                                                  coordf_t(print_cfg.mixed_color_layer_height_a.value));
    coordf_t preferred_b = float_from_full_config(full_cfg, "mixed_color_layer_height_b",
                                                  coordf_t(print_cfg.mixed_color_layer_height_b.value));
    coordf_t gradient_nominal_height = float_from_full_config(
        full_cfg,
        "dithering_local_z_gradient_layer_height",
        coordf_t(print_cfg.dithering_local_z_gradient_layer_height.value));
    coordf_t gradient_middle_filament_window = float_from_full_config(
        full_cfg,
        "dithering_local_z_gradient_middle_filament_window",
        coordf_t(print_cfg.dithering_local_z_gradient_middle_filament_window.value));
    min_sublayer_height = std::max<coordf_t>(0.01f, min_sublayer_height);
    preferred_a = std::max<coordf_t>(0.f, preferred_a);
    preferred_b = std::max<coordf_t>(0.f, preferred_b);
    gradient_nominal_height = std::max<coordf_t>(gradient_nominal_height, 2.f * min_sublayer_height);
    gradient_middle_filament_window = std::clamp<coordf_t>(gradient_middle_filament_window, 0.f, 100.f);
    const double gradient_middle_filament_fraction = double(gradient_middle_filament_window) / 100.0;

    const size_t num_physical = print_cfg.filament_colour.size();
    if (num_physical == 0) {
        BOOST_LOG_TRIVIAL(warning) << "Local-Z plan skipped: no physical filaments"
                                   << " object=" << object_name;
        return;
    }

    std::vector<double> gradient_max_layer_heights(num_physical);
    for (size_t i = 0; i < num_physical; ++i)
        gradient_max_layer_heights[i] = local_z_max_layer_height_for_extruder(print_cfg, unsigned(i + 1));

    const MixedFilamentManager &mixed_mgr = print->mixed_filament_manager();
    const double adaptive_nominal_layer_height = std::max(0.01, print_object.config().layer_height.value);
    const std::vector<MixedFilamentDefinition> mixed_definitions = mixed_mgr.mixed_filament_definitions(num_physical);
    std::vector<uint8_t> row_uses_local_z(mixed_definitions.size(), uint8_t(0));
    size_t enabled_row_count = 0;
    for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
        const bool enabled_by_caller = row_idx < enabled_rows.size() && enabled_rows[row_idx] != 0;
        if (!enabled_by_caller && !mixed_filament_definition_uses_local_z(mixed_definitions[row_idx], local_z_mode))
            continue;
        row_uses_local_z[row_idx] = uint8_t(1);
        ++enabled_row_count;
    }
    if (enabled_row_count == 0) {
        BOOST_LOG_TRIVIAL(debug) << "Local-Z plan skipped: no enabled mixed rows"
                                 << " object=" << object_name;
        return;
    }

    std::vector<uint8_t> row_is_gradient_definition(mixed_definitions.size(), uint8_t(0));
    for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
        if (row_uses_local_z[row_idx] != 0 && local_z_gradient_mixed_definition(mixed_definitions[row_idx]))
            row_is_gradient_definition[row_idx] = uint8_t(1);
    }
    std::vector<std::vector<unsigned int>> row_gradient_component_ids(mixed_definitions.size());
    std::vector<std::vector<double>>       row_gradient_stop_positions(mixed_definitions.size());
    for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
        if (row_is_gradient_definition[row_idx] == 0)
            continue;
        row_gradient_component_ids[row_idx] = decode_blend_component_ids(mixed_definitions[row_idx], num_physical);
        row_gradient_stop_positions[row_idx] = build_local_z_gradient_stop_positions(mixed_definitions[row_idx],
                                                                                    num_physical,
                                                                                    row_gradient_component_ids[row_idx].size());
    }
    std::vector<std::vector<unsigned int>> row_direct_component_ids(mixed_definitions.size());
    std::vector<std::vector<int>>          row_direct_component_weights(mixed_definitions.size());
    std::vector<std::vector<double>>       row_direct_component_error_mm(mixed_definitions.size());
    std::vector<std::vector<LocalZIndependentDirectPass>> row_independent_direct_cadences(mixed_definitions.size());
    std::vector<uint8_t>                   row_uses_direct_multicolor_solver(mixed_definitions.size(), uint8_t(0));
    if (preferred_a <= EPSILON && preferred_b <= EPSILON) {
        for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
            const bool adaptive_direct = row_idx < adaptive_independent_rows.size() && adaptive_independent_rows[row_idx] != 0;
            if (!local_z_direct_multicolor && !adaptive_direct)
                continue;
            if (adaptive_direct) {
                row_direct_component_ids[row_idx] = decode_blend_component_ids(mixed_definitions[row_idx], num_physical);
                if (row_direct_component_ids[row_idx].size() >= 2) {
                    row_direct_component_weights[row_idx] =
                        decode_blend_component_weights(mixed_definitions[row_idx],
                                                       num_physical,
                                                       row_direct_component_ids[row_idx].size());
                    if (row_direct_component_weights[row_idx].size() != row_direct_component_ids[row_idx].size())
                        row_direct_component_weights[row_idx].assign(row_direct_component_ids[row_idx].size(), 1);
                    row_uses_direct_multicolor_solver[row_idx] = uint8_t(1);
                    row_direct_component_error_mm[row_idx].assign(row_direct_component_ids[row_idx].size(), 0.0);
                    row_independent_direct_cadences[row_idx] = build_adaptive_local_z_modulated_cadence(
                        row_direct_component_ids[row_idx],
                        row_direct_component_weights[row_idx],
                        adaptive_nominal_layer_height * double(row_direct_component_ids[row_idx].size()),
                        min_sublayer_height,
                        print_cfg);
                }
                continue;
            }
            if (local_z_direct_multicolor_definition(mixed_definitions[row_idx],
                                                     num_physical,
                                                     &row_direct_component_ids[row_idx],
                                                     &row_direct_component_weights[row_idx])) {
                row_uses_direct_multicolor_solver[row_idx] = uint8_t(1);
                row_direct_component_error_mm[row_idx].assign(row_direct_component_ids[row_idx].size(), 0.0);
                row_independent_direct_cadences[row_idx] =
                    build_local_z_independent_direct_cadence(row_direct_component_ids[row_idx],
                                                             row_direct_component_weights[row_idx],
                                                             min_sublayer_height,
                                                             print_cfg);
            }
        }
    }
    if (preferred_a <= EPSILON && preferred_b <= EPSILON) {
        for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
            const bool adaptive_independent =
                row_idx < adaptive_independent_rows.size() && adaptive_independent_rows[row_idx] != 0;
            if (!local_z_independent_layer_height && !adaptive_independent)
                continue;
            if (!row_independent_direct_cadences[row_idx].empty() ||
                row_uses_local_z[row_idx] == 0 ||
                !local_z_eligible_mixed_definition(mixed_definitions[row_idx]) ||
                mixed_definitions[row_idx].behavior.gradient.enabled) {
                continue;
            }

            const std::vector<unsigned int> component_ids =
                decode_blend_component_ids(mixed_definitions[row_idx], num_physical);
            if (component_ids.size() < 2)
                continue;
            std::vector<int> component_weights =
                decode_blend_component_weights(mixed_definitions[row_idx], num_physical, component_ids.size());
            if (component_weights.empty())
                component_weights.assign(component_ids.size(), 1);
            row_independent_direct_cadences[row_idx] =
                build_local_z_independent_direct_cadence(component_ids,
                                                         component_weights,
                                                         min_sublayer_height,
                                                         print_cfg);
        }
    }
    std::vector<std::vector<LocalZActivePair>> row_pair_cycles(mixed_definitions.size());
    std::vector<uint8_t>                       row_uses_layer_cycle_pair(mixed_definitions.size(), uint8_t(0));
    for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
        if (row_uses_direct_multicolor_solver[row_idx] != 0)
            continue;
        row_pair_cycles[row_idx] = build_local_z_pair_cycle_for_definition(mixed_definitions[row_idx], num_physical);
        if (!row_pair_cycles[row_idx].empty())
            row_uses_layer_cycle_pair[row_idx] = uint8_t(1);
    }

    BOOST_LOG_TRIVIAL(debug) << "Local-Z plan start"
                             << " object=" << object_name
                             << " layers=" << print_object.layer_count()
                             << " min_sublayer_height=" << min_sublayer_height
                             << " preferred_a=" << preferred_a
                             << " preferred_b=" << preferred_b
                             << " gradient_nominal_height=" << gradient_nominal_height
                             << " direct_multicolor=" << (local_z_direct_multicolor ? 1 : 0)
                             << " independent_layer_height=" << (local_z_independent_layer_height ? 1 : 0)
                             << " global_mode=" << (local_z_mode ? 1 : 0)
                             << " enabled_rows=" << enabled_row_count
                             << " full_domain=" << (local_z_whole_objects ? 1 : 0)
                             << " preserve_first_layer=" << (local_z_preserve_first_layer ? 1 : 0)
                             << " gradient_middle_filament_window=" << gradient_middle_filament_window
                             << " physical_filaments=" << num_physical;

    std::vector<LocalZInterval> intervals;
    std::vector<SubLayerPlan>   plans;
    intervals.reserve(print_object.layer_count());
    size_t mixed_intervals              = 0;
    size_t split_intervals              = 0;
    size_t non_split_mixed_intervals    = 0;
    size_t total_generated_sublayer_cnt = 0;
    size_t total_mixed_state_layers     = 0;
    size_t forced_height_resolve_calls  = 0;
    size_t forced_height_resolve_invalid_target   = 0;
    size_t split_passes_total                     = 0;
    size_t split_passes_with_painted_masks        = 0;
    size_t split_intervals_without_painted_masks  = 0;
    size_t strict_ab_assignments                  = 0;
    size_t alternating_height_intervals           = 0;
    size_t shared_multi_row_fallback_intervals    = 0;
    constexpr size_t LOCAL_Z_MAX_ISOLATED_ACTIVE_ROWS = 2;
    constexpr size_t LOCAL_Z_MAX_ISOLATED_MASK_COMPONENTS = 24;
    constexpr size_t LOCAL_Z_MAX_ISOLATED_MASK_VERTICES = 1200;
    constexpr bool   LOCAL_Z_SHARED_FALLBACK_ENABLED = false;
    struct IndependentCadenceState {
        bool                                      active { false };
        double                                    z_cursor { 0.0 };
        size_t                                    cadence_index { 0 };
        size_t                                    emitted_passes { 0 };
        std::vector<LocalZIndependentDirectPass> gradient_cadence;
        size_t                                    gradient_pass_index { 0 };
        unsigned int                              last_extruder { 0 };
        double                                    adaptive_cycle_z_lo { 0.0 };
        double                                    adaptive_cycle_z_hi { 0.0 };
        double                                    adaptive_sample_z { 0.0 };
        size_t                                    adaptive_cycle_end_layer { 0 };
    };
    std::vector<IndependentCadenceState> row_independent_states(mixed_definitions.size());
    // Keep local-Z cadence isolated per mixed row so independent painted zones
    // do not influence each other when resolving fallback cadence.
    std::vector<int> row_cadence_index(mixed_definitions.size(), 0);
    // Multi-color layer-cycle rows choose a pair once per nominal layer/zone
    // and rotate that pair independently from per-subpass A/B cadence.
    std::vector<int> row_layer_cycle_index(mixed_definitions.size(), 0);
    // Painted-only Local-Z keeps cadence isolated per zone. Whole-object Local-Z
    // instead syncs newly introduced painted rows to the dominant mixed cadence
    // so painted islands do not restart their phase at the boundary.
    std::vector<uint8_t> row_active_prev_layer(mixed_definitions.size(), uint8_t(0));
    std::vector<unsigned int> row_last_gradient_extruder(mixed_definitions.size(), 0);

    std::vector<std::vector<int>> per_row_gradient_layers(mixed_definitions.size());
    for (size_t scan_layer = 0; scan_layer < segmentation.size(); ++scan_layer) {
        std::vector<uint8_t> row_active_scan(mixed_definitions.size(), uint8_t(0));
        for (size_t channel_idx = 0; channel_idx < segmentation[scan_layer].size(); ++channel_idx) {
            if (segmentation[scan_layer][channel_idx].empty())
                continue;
            const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
            if (!mixed_mgr.is_mixed(state_id, num_physical))
                continue;
            const int mixed_idx = mixed_mgr.mixed_index_from_filament_id(state_id, num_physical);
            if (mixed_idx < 0 || size_t(mixed_idx) >= mixed_definitions.size())
                continue;
            if (row_uses_local_z[size_t(mixed_idx)] == 0)
                continue;
            row_active_scan[size_t(mixed_idx)] = uint8_t(1);
        }
        for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
            if (row_is_gradient_definition[row_idx] == 0 || row_active_scan[row_idx] == 0)
                continue;
            per_row_gradient_layers[row_idx].push_back(int(scan_layer));
        }
    }

    for (size_t row_idx = 0; row_idx < mixed_definitions.size(); ++row_idx) {
        if (row_is_gradient_definition[row_idx] == 0)
            continue;
        per_row_gradient_layers[row_idx] = fill_continuous_layer_range(std::move(per_row_gradient_layers[row_idx]));
    }

    std::vector<std::pair<double, double>> per_row_gradient_z_bounds(mixed_definitions.size(), {0.0, 0.0});
    for (size_t row_idx = 0; row_idx < per_row_gradient_layers.size(); ++row_idx) {
        const std::vector<int> &gradient_layers = per_row_gradient_layers[row_idx];
        if (gradient_layers.empty())
            continue;
        const Layer *first_layer = print_object.get_layer(gradient_layers.front());
        const Layer *last_layer  = print_object.get_layer(gradient_layers.back());
        if (first_layer == nullptr || last_layer == nullptr)
            continue;
        per_row_gradient_z_bounds[row_idx] = {
            first_layer->print_z - first_layer->height,
            last_layer->print_z
        };
    }

    auto gradient_progress_for_definition = [&](size_t row_idx, size_t layer_id) -> std::optional<double> {
        if (row_idx >= mixed_definitions.size() || row_is_gradient_definition[row_idx] == 0)
            return std::nullopt;
        const auto &layers = per_row_gradient_layers[row_idx];
        if (layers.empty())
            return std::nullopt;

        const int li = int(layer_id);
        if (li < layers.front() || li > layers.back())
            return std::nullopt;

        const int    idx = li - layers.front();
        const int    count = layers.back() - layers.front() + 1;
        return count > 0 ? std::optional<double>((2.0 * double(idx) + 1.0) / (2.0 * double(count))) : std::optional<double>(0.5);
    };

    auto effective_gradient_active_pair_for_definition = [&](size_t row_idx, size_t layer_id, LocalZActivePair &pair_out) -> bool {
        if (row_idx >= row_gradient_component_ids.size())
            return false;

        const std::vector<unsigned int> &ids = row_gradient_component_ids[row_idx];
        const std::optional<double> progress = gradient_progress_for_definition(row_idx, layer_id);
        static const std::vector<double> empty_stop_positions;
        const std::vector<double> &stop_positions =
            row_idx < row_gradient_stop_positions.size() ? row_gradient_stop_positions[row_idx] : empty_stop_positions;
        if (!progress)
            return false;

        if (!local_z_gradient_active_pair_for_progress(ids, stop_positions, *progress, gradient_middle_filament_fraction, pair_out,
                                                       mixed_definitions[row_idx].behavior.gradient.solid_widths))
            return false;
        if (row_idx < row_last_gradient_extruder.size())
            local_z_orient_pair_to_follow_previous(pair_out, row_last_gradient_extruder[row_idx]);
        return pair_out.valid_pair(num_physical);
    };

    auto effective_gradient_active_pair_for_z = [&](size_t          row_idx,
                                                    double          sample_z,
                                                    unsigned int    previous_extruder,
                                                    LocalZActivePair &pair_out) -> bool {
        if (row_idx >= row_gradient_component_ids.size() || row_is_gradient_definition[row_idx] == 0)
            return false;

        const auto [domain_lo, domain_hi] = per_row_gradient_z_bounds[row_idx];
        const double domain_height = domain_hi - domain_lo;
        if (domain_height <= EPSILON)
            return false;

        const double progress = std::clamp((sample_z - domain_lo) / domain_height, 0.0, 1.0);
        static const std::vector<double> empty_stop_positions;
        const std::vector<double> &stop_positions =
            row_idx < row_gradient_stop_positions.size() ? row_gradient_stop_positions[row_idx] : empty_stop_positions;
        const std::vector<unsigned int> &ids = row_gradient_component_ids[row_idx];

        bool resolved = local_z_gradient_active_pair_for_progress(ids, stop_positions, progress, gradient_middle_filament_fraction,
                                                                  pair_out, mixed_definitions[row_idx].behavior.gradient.solid_widths);
        if (!resolved) {
            const std::optional<MixedFilamentPrimaryPairView> primary_pair =
                mixed_definitions[row_idx].recipe.blend.primary_pair();
            if (!primary_pair || primary_pair->component_a.id == 0 || primary_pair->component_b.id == 0)
                return false;

            const MixedFilamentGradientBehavior &gradient = mixed_definitions[row_idx].behavior.gradient;
            double component_a_fraction = double(gradient.component_a_start) +
                (double(gradient.component_a_end) - double(gradient.component_a_start)) * progress;
            component_a_fraction = std::clamp(component_a_fraction, 0.0, 1.0);
            pair_out = LocalZActivePair{};
            pair_out.component_a = primary_pair->component_a.id;
            pair_out.component_b = primary_pair->component_b.id;
            pair_out.mix_b_percent = local_z_mix_b_percent_from_fraction(1.0 - component_a_fraction);
        }

        local_z_orient_pair_to_follow_previous(pair_out, previous_extruder);
        return pair_out.valid_pair(num_physical);
    };

    auto effective_gradient_heights_for_definition = [&](size_t row_idx, size_t layer_id, double base_height,
                                                         double &h_a_out, double &h_b_out) -> bool {
        if (row_idx >= mixed_definitions.size() || row_is_gradient_definition[row_idx] == 0 || base_height <= EPSILON)
            return false;

        LocalZActivePair active_pair;
        if (effective_gradient_active_pair_for_definition(row_idx, layer_id, active_pair)) {
            const double r_a = std::clamp(1.0 - double(active_pair.mix_b_percent) / 100.0, 0.01, 0.99);
            h_a_out = r_a * base_height;
            h_b_out = (1.0 - r_a) * base_height;
            return true;
        }

        const std::optional<double> progress = gradient_progress_for_definition(row_idx, layer_id);
        if (!progress)
            return false;

        const MixedFilamentGradientBehavior &gradient = mixed_definitions[row_idx].behavior.gradient;
        double r_a = double(gradient.component_a_start) +
                     (double(gradient.component_a_end) - double(gradient.component_a_start)) * *progress;
        r_a = std::clamp(r_a, 0.01, 0.99);
        h_a_out = r_a * base_height;
        h_b_out = (1.0 - r_a) * base_height;
        return true;
    };

    auto build_direct_pass_heights_for_definition = [&](size_t row_idx,
                                                        size_t layer_id,
                                                        double base_height) -> std::vector<double> {
        if (row_idx >= mixed_definitions.size() || row_idx >= row_direct_component_ids.size())
            return {};

        (void)layer_id;
        return build_local_z_direct_multicolor_pass_heights(mixed_definitions[row_idx],
                                                             row_direct_component_weights[row_idx],
                                                             base_height,
                                                             min_sublayer_height,
                                                             row_direct_component_ids[row_idx].size());
    };

    auto build_direct_sequence_for_definition = [&](size_t row_idx,
                                                   size_t layer_id,
                                                   const std::vector<double> &pass_heights) -> std::vector<unsigned int> {
        if (row_idx >= row_direct_component_ids.size() || pass_heights.empty())
            return {};

        (void)layer_id;
        return build_local_z_direct_multicolor_sequence(row_direct_component_ids[row_idx],
                                                        row_direct_component_weights[row_idx],
                                                        pass_heights,
                                                        row_direct_component_error_mm[row_idx]);
    };

    auto row_is_active_on_layer = [&](size_t row_idx, size_t layer_id) {
        if (row_idx >= mixed_definitions.size() || layer_id >= segmentation.size())
            return false;
        for (size_t channel_idx = 0; channel_idx < segmentation[layer_id].size(); ++channel_idx) {
            if (segmentation[layer_id][channel_idx].empty())
                continue;
            const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
            if (!mixed_mgr.is_mixed(state_id, num_physical))
                continue;
            const int mixed_idx = mixed_mgr.mixed_index_from_filament_id(state_id, num_physical);
            if (mixed_idx >= 0 && size_t(mixed_idx) == row_idx)
                return true;
        }
        return false;
    };

    for (size_t layer_id = 0; layer_id < print_object.layer_count(); ++layer_id) {
        throw_on_cancel();

        const Layer &layer = *print_object.get_layer(int(layer_id));
        LocalZInterval interval;
        interval.layer_id           = layer_id;
        interval.z_lo               = layer.print_z - layer.height;
        interval.z_hi               = layer.print_z;
        interval.base_height        = layer.height;
        interval.sublayer_height    = layer.height;
        interval.first_sublayer_idx = plans.size();
        const bool subdivide_this_layer = mixed_filament_local_z_should_subdivide_layer(
            layer_id,
            local_z_whole_objects,
            local_z_preserve_first_layer);

        ExPolygons mixed_masks;
        size_t     mixed_state_count = 0;
        std::vector<uint8_t> row_active_this_layer(mixed_definitions.size(), uint8_t(0));
        size_t     dominant_mixed_idx = size_t(-1);
        double     dominant_mixed_area = -1.0;
        double     dominant_gradient_h_a = 0.0;
        double     dominant_gradient_h_b = 0.0;
        bool       dominant_gradient_valid = false;
        bool       dominant_is_gradient = false;
        for (size_t channel_idx = 0; channel_idx < segmentation[layer_id].size(); ++channel_idx) {
            const ExPolygons &state_masks = segmentation[layer_id][channel_idx];
            if (state_masks.empty())
                continue;
            const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
            if (!mixed_mgr.is_mixed(state_id, num_physical))
                continue;

            const int mixed_idx = mixed_mgr.mixed_index_from_filament_id(state_id, num_physical);
            if (mixed_idx < 0 || size_t(mixed_idx) >= mixed_definitions.size())
                continue;
            if (row_uses_local_z[size_t(mixed_idx)] == 0)
                continue;
            const MixedFilamentDefinition &definition = mixed_definitions[size_t(mixed_idx)];
            if (!local_z_eligible_mixed_definition(definition))
                continue;
            interval.has_mixed_paint = true;
            row_active_this_layer[size_t(mixed_idx)] = uint8_t(1);
            ++mixed_state_count;
            append(mixed_masks, state_masks);

            const double mixed_area = std::abs(area(state_masks));
            if (mixed_area > dominant_mixed_area) {
                dominant_mixed_area = mixed_area;
                dominant_mixed_idx  = size_t(mixed_idx);
            }
        }
        for (size_t row_idx = 0; row_idx < row_active_this_layer.size(); ++row_idx) {
            if (row_active_this_layer[row_idx] == 0) {
                if (row_idx < row_last_gradient_extruder.size())
                    row_last_gradient_extruder[row_idx] = 0;
                continue;
            }
            if (row_active_this_layer[row_idx] != 0 && row_active_prev_layer[row_idx] == 0) {
                if (row_idx < row_last_gradient_extruder.size())
                    row_last_gradient_extruder[row_idx] = 0;
                if (row_uses_direct_multicolor_solver[row_idx] != 0 &&
                    row_direct_component_error_mm[row_idx].size() == row_direct_component_ids[row_idx].size()) {
                    std::fill(row_direct_component_error_mm[row_idx].begin(), row_direct_component_error_mm[row_idx].end(), 0.0);
                }
                const bool can_sync_to_dominant =
                    local_z_whole_objects &&
                    dominant_mixed_idx < mixed_definitions.size() &&
                    dominant_mixed_idx != row_idx &&
                    row_active_this_layer[dominant_mixed_idx] != 0;
                if (can_sync_to_dominant) {
                    row_cadence_index[row_idx]     = row_cadence_index[dominant_mixed_idx];
                    row_layer_cycle_index[row_idx] = row_layer_cycle_index[dominant_mixed_idx];
                } else {
                    row_cadence_index[row_idx]     = 0;
                    row_layer_cycle_index[row_idx] = 0;
                }
            }
        }
        std::vector<LocalZActivePair> row_active_pairs(mixed_definitions.size());
        for (size_t row_idx = 0; row_idx < row_active_this_layer.size(); ++row_idx) {
            if (row_active_this_layer[row_idx] == 0 || !local_z_eligible_mixed_definition(mixed_definitions[row_idx]))
                continue;
            if (row_uses_direct_multicolor_solver[row_idx] != 0)
                continue;

            LocalZActivePair gradient_pair;
            if (effective_gradient_active_pair_for_definition(row_idx, layer_id, gradient_pair)) {
                row_active_pairs[row_idx] = gradient_pair;
                continue;
            }

            const int cadence_index = row_uses_layer_cycle_pair[row_idx] != 0
                ? row_layer_cycle_index[row_idx]
                : row_cadence_index[row_idx];
            row_active_pairs[row_idx] =
                derive_local_z_active_pair(mixed_definitions[row_idx], row_pair_cycles[row_idx], num_physical, cadence_index);
        }
        if (dominant_mixed_idx < mixed_definitions.size()) {
            const LocalZActivePair &dominant_pair = row_active_pairs[dominant_mixed_idx];
            const int dominant_mix_b_percent =
                dominant_pair.valid_pair(num_physical) ? dominant_pair.mix_b_percent :
                                                          mixed_definitions[dominant_mixed_idx]
                                                              .recipe.blend.primary_pair_or().component_b_percent;
            if (row_uses_direct_multicolor_solver[dominant_mixed_idx] == 0) {
                dominant_is_gradient = effective_gradient_heights_for_definition(dominant_mixed_idx,
                                                                                layer_id,
                                                                                interval.base_height,
                                                                                dominant_gradient_h_a,
                                                                                dominant_gradient_h_b);
                if (!dominant_is_gradient)
                    compute_local_z_component_heights(dominant_mix_b_percent,
                                                      interval.base_height,
                                                      min_sublayer_height,
                                                      dominant_gradient_h_a,
                                                      dominant_gradient_h_b);
                dominant_gradient_valid = true;
            }
        }
        total_mixed_state_layers += mixed_state_count;
        if (!mixed_masks.empty())
            mixed_masks = union_ex(mixed_masks);
        if (interval.has_mixed_paint)
            ++mixed_intervals;

        const ExPolygons layer_masks = collect_layer_region_slices(layer);
        ExPolygons       base_masks  = layer_masks;
        if (interval.has_mixed_paint && !base_masks.empty() && !mixed_masks.empty()) {
            base_masks = diff_ex(base_masks, mixed_masks);
            if (!base_masks.empty()) {
                const Polygons filtered = opening(to_polygons(base_masks), scaled<float>(5. * EPSILON), scaled<float>(5. * EPSILON));
                base_masks = union_ex(filtered);
            }
        }

        const size_t active_mixed_definitions = size_t(std::count(row_active_this_layer.begin(), row_active_this_layer.end(), uint8_t(1)));
        std::vector<ExPolygons> row_state_masks(mixed_definitions.size());
        std::vector<unsigned int> row_state_ids(mixed_definitions.size(), 0);
        std::vector<ExPolygons> fixed_state_masks_by_extruder(num_physical);
        for (size_t channel_idx = 0; channel_idx < segmentation[layer_id].size(); ++channel_idx) {
            const ExPolygons &state_masks = segmentation[layer_id][channel_idx];
            if (state_masks.empty())
                continue;
            const unsigned int state_id = segmentation_channel_filament_id(channel_idx);
            if (state_id >= 1 && state_id <= num_physical) {
                append(fixed_state_masks_by_extruder[state_id - 1], state_masks);
                continue;
            }
            if (!mixed_mgr.is_mixed(state_id, num_physical))
                continue;
            const int mixed_idx = mixed_mgr.mixed_index_from_filament_id(state_id, num_physical);
            if (mixed_idx < 0 || size_t(mixed_idx) >= mixed_definitions.size())
                continue;
            const size_t row_idx = size_t(mixed_idx);
            if (row_active_this_layer[row_idx] == 0 || !local_z_eligible_mixed_definition(mixed_definitions[row_idx]))
                continue;
            row_state_ids[row_idx] = state_id;
            append(row_state_masks[row_idx], state_masks);
        }
        for (ExPolygons &state_masks : row_state_masks)
            if (state_masks.size() > 1)
                state_masks = union_ex(state_masks);
        for (ExPolygons &state_masks : fixed_state_masks_by_extruder)
            if (state_masks.size() > 1)
                state_masks = union_ex(state_masks);
        const std::vector<ExPolygons> prev_fixed_state_masks_by_extruder =
            layer_id > 0 ? collect_local_z_fixed_state_masks_by_extruder(segmentation[layer_id - 1], num_physical)
                         : std::vector<ExPolygons>(num_physical);
        const std::vector<ExPolygons> next_fixed_state_masks_by_extruder =
            layer_id + 1 < segmentation.size() ? collect_local_z_fixed_state_masks_by_extruder(segmentation[layer_id + 1], num_physical)
                                               : std::vector<ExPolygons>(num_physical);

        ExPolygons fixed_state_masks_union;
        for (const ExPolygons &state_masks : fixed_state_masks_by_extruder)
            if (!state_masks.empty())
                append(fixed_state_masks_union, state_masks);
        if (fixed_state_masks_union.size() > 1)
            fixed_state_masks_union = union_ex(fixed_state_masks_union);
        if (interval.has_mixed_paint && local_z_whole_objects && !fixed_state_masks_union.empty()) {
            if (!base_masks.empty()) {
                base_masks = diff_ex(base_masks, fixed_state_masks_union);
                if (!base_masks.empty()) {
                    const Polygons filtered = opening(to_polygons(base_masks), scaled<float>(5. * EPSILON), scaled<float>(5. * EPSILON));
                    base_masks = union_ex(filtered);
                }
            }
            append(mixed_masks, fixed_state_masks_union);
            if (mixed_masks.size() > 1)
                mixed_masks = union_ex(mixed_masks);
        }
        if (local_z_whole_objects && !fixed_state_masks_union.empty()) {
            constexpr double LOCAL_Z_WHOLE_OBJECT_FIXED_GUARD_MM = 0.10;
            ExPolygons fixed_state_guard_masks = offset_ex(fixed_state_masks_union, float(scale_(LOCAL_Z_WHOLE_OBJECT_FIXED_GUARD_MM)));
            if (fixed_state_guard_masks.empty())
                fixed_state_guard_masks = fixed_state_masks_union;
            else if (fixed_state_guard_masks.size() > 1)
                fixed_state_guard_masks = union_ex(fixed_state_guard_masks);

            for (ExPolygons &state_masks : row_state_masks) {
                if (state_masks.empty())
                    continue;
                state_masks = diff_ex(state_masks, fixed_state_guard_masks);
                if (state_masks.size() > 1)
                    state_masks = union_ex(state_masks);
            }
        }

        size_t active_row_mask_components = 0;
        size_t active_row_mask_vertices   = 0;
        for (size_t row_idx = 0; row_idx < row_state_masks.size(); ++row_idx)
            if (row_active_this_layer[row_idx] != 0) {
                active_row_mask_components += row_state_masks[row_idx].size();
                for (const ExPolygon &expoly : row_state_masks[row_idx]) {
                    active_row_mask_vertices += expoly.contour.points.size();
                    for (const Polygon &hole : expoly.holes)
                        active_row_mask_vertices += hole.points.size();
                }
            }

        bool independent_rows_eligible = active_mixed_definitions > 0;
        for (size_t row_idx = 0; row_idx < row_active_this_layer.size(); ++row_idx) {
            if (row_active_this_layer[row_idx] == 0)
                continue;
            const bool gradient_row = row_idx < row_is_gradient_definition.size() &&
                                      row_is_gradient_definition[row_idx] != 0;
            const bool adaptive_independent =
                row_idx < adaptive_independent_rows.size() && adaptive_independent_rows[row_idx] != 0;
            const bool direct_row = (local_z_independent_layer_height || adaptive_independent) &&
                                    row_idx < row_independent_direct_cadences.size() &&
                                    !row_independent_direct_cadences[row_idx].empty();
            independent_rows_eligible = independent_rows_eligible && (gradient_row || direct_row);
        }
        const bool independent_interval =
            interval.has_mixed_paint &&
            subdivide_this_layer &&
            independent_rows_eligible;

        if (independent_interval) {
            interval.independent_layer_height = true;
            interval.managed_masks            = mixed_masks;
            ++split_intervals;

            std::vector<SubLayerPlan> independent_plans;
            independent_plans.reserve(std::max<size_t>(2, active_mixed_definitions * 2));
            for (size_t row_idx = 0; row_idx < row_active_this_layer.size(); ++row_idx) {
                if (row_active_this_layer[row_idx] == 0 || row_state_masks[row_idx].empty())
                    continue;

                IndependentCadenceState &state = row_independent_states[row_idx];
                const bool gradient_row = row_idx < row_is_gradient_definition.size() &&
                                          row_is_gradient_definition[row_idx] != 0;
                const bool adaptive_independent =
                    row_idx < adaptive_independent_rows.size() && adaptive_independent_rows[row_idx] != 0;
                const std::vector<LocalZIndependentDirectPass> &direct_cadence = row_independent_direct_cadences[row_idx];

                if (adaptive_independent) {
                    const std::vector<unsigned int> &component_ids = row_direct_component_ids[row_idx];
                    const std::vector<int>          &component_weights = row_direct_component_weights[row_idx];
                    const size_t component_count = component_ids.size();
                    if (component_count < 2 || direct_cadence.size() != component_count)
                        continue;

                    // Strict 1:1 / 1:1:1 / 1:1:1:1 cadence means one
                    // component pass per nominal layer. The complete cadence
                    // owns N nominal layer heights, which leaves a full layer
                    // of average height for every component instead of
                    // squeezing all N components into one 0.20 mm interval.
                    if (!state.active || row_active_prev_layer[row_idx] == 0 ||
                        state.cadence_index >= component_count || layer_id > state.adaptive_cycle_end_layer) {
                        state = IndependentCadenceState{};
                        state.active                   = true;
                        state.adaptive_cycle_z_lo      = interval.z_lo;
                        state.adaptive_cycle_end_layer = layer_id;
                        for (size_t offset = 1; offset < component_count && layer_id + offset < print_object.layer_count(); ++offset) {
                            const size_t candidate_layer = layer_id + offset;
                            if (!mixed_filament_local_z_should_subdivide_layer(candidate_layer,
                                                                              local_z_whole_objects,
                                                                              local_z_preserve_first_layer) ||
                                !row_is_active_on_layer(row_idx, candidate_layer))
                                break;
                            state.adaptive_cycle_end_layer = candidate_layer;
                        }
                        const Layer *cycle_end_layer = print_object.get_layer(int(state.adaptive_cycle_end_layer));
                        state.adaptive_cycle_z_hi = cycle_end_layer != nullptr ? cycle_end_layer->print_z : interval.z_hi;
                        state.adaptive_sample_z   = layer.slice_z;
                    }

                    const double cycle_height = state.adaptive_cycle_z_hi - state.adaptive_cycle_z_lo;
                    const std::vector<double> component_heights = adaptive_local_z_bounded_heights(component_ids,
                                                                                                    component_weights,
                                                                                                    cycle_height,
                                                                                                    min_sublayer_height);
                    if (component_heights.size() != component_count)
                        continue;

                    // A short final/isolated run still completes the strict
                    // order on its last available layer so the perimeter
                    // reaches the model top without leaving a partial stack.
                    const size_t pass_count = layer_id == state.adaptive_cycle_end_layer
                        ? component_count - state.cadence_index
                        : size_t(1);
                    for (size_t pass_offset = 0; pass_offset < pass_count; ++pass_offset) {
                        const size_t component_idx = state.cadence_index;
                        if (component_idx >= component_count)
                            break;
                        const unsigned int component_id = component_ids[component_idx];
                        const double component_height = component_heights[component_idx];
                        if (component_id == 0 || component_id > num_physical || component_height <= EPSILON) {
                            ++state.cadence_index;
                            continue;
                        }

                        const double z_next = component_idx + 1 == component_count
                            ? state.adaptive_cycle_z_hi
                            : state.adaptive_cycle_z_lo +
                                  std::accumulate(component_heights.begin(),
                                                  component_heights.begin() + component_idx + 1,
                                                  0.0);
                        SubLayerPlan plan;
                        plan.layer_id                    = layer_id;
                        plan.pass_index                  = independent_plans.size();
                        plan.split_interval              = true;
                        plan.external_perimeters_only    = true;
                        plan.z_lo                        = interval.z_lo;
                        plan.z_hi                        = interval.z_hi;
                        plan.print_z                     = z_next;
                        plan.flow_height                 = component_height;
                        plan.dependency_group            = row_idx + 1;
                        plan.dependency_order            = state.emitted_passes++;
                        plan.adaptive_nonplanar          = true;
                        plan.adaptive_component_index    = component_idx;
                        plan.adaptive_component_ids      = component_ids;
                        plan.adaptive_component_weights  = component_weights;
                        plan.adaptive_cycle_z_lo         = state.adaptive_cycle_z_lo;
                        plan.adaptive_cycle_z_hi         = state.adaptive_cycle_z_hi;
                        plan.adaptive_sample_z           = state.adaptive_sample_z;
                        plan.painted_masks_by_extruder.assign(num_physical, ExPolygons());
                        plan.fixed_painted_masks_by_extruder.assign(num_physical, ExPolygons());
                        append(plan.painted_masks_by_extruder[component_id - 1], row_state_masks[row_idx]);
                        independent_plans.emplace_back(std::move(plan));
                        ++state.cadence_index;
                        ++split_passes_total;
                        ++split_passes_with_painted_masks;
                        ++forced_height_resolve_calls;
                    }
                    if (state.cadence_index >= component_count)
                        state.active = false;
                    continue;
                }

                if (!state.active || row_active_prev_layer[row_idx] == 0 ||
                    state.z_cursor > interval.z_hi + EPSILON) {
                    state = IndependentCadenceState{};
                    state.active   = true;
                    state.z_cursor = interval.z_lo;
                }

                auto append_independent_pass = [&](double z_hi, double flow_height, unsigned int target_extruder) {
                    if (flow_height <= EPSILON || target_extruder == 0 || target_extruder > num_physical)
                        return;

                    SubLayerPlan plan;
                    plan.layer_id         = layer_id;
                    plan.pass_index       = independent_plans.size();
                    plan.split_interval   = true;
                    plan.external_perimeters_only = adaptive_independent;
                    plan.z_lo             = z_hi - flow_height;
                    plan.z_hi             = z_hi;
                    plan.print_z          = z_hi;
                    plan.flow_height      = flow_height;
                    plan.dependency_group = row_idx + 1;
                    plan.dependency_order = state.emitted_passes++;
                    plan.painted_masks_by_extruder.assign(num_physical, ExPolygons());
                    plan.fixed_painted_masks_by_extruder.assign(num_physical, ExPolygons());
                    append(plan.painted_masks_by_extruder[target_extruder - 1], row_state_masks[row_idx]);
                    independent_plans.emplace_back(std::move(plan));
                    ++split_passes_total;
                    ++split_passes_with_painted_masks;
                    ++forced_height_resolve_calls;
                };

                auto ensure_gradient_cadence = [&]() {
                    if (!gradient_row)
                        return false;
                    if (state.gradient_pass_index < state.gradient_cadence.size())
                        return true;

                    const auto [domain_lo, domain_hi] = per_row_gradient_z_bounds[row_idx];
                    const double remaining_domain_height = domain_hi - state.z_cursor;
                    if (remaining_domain_height <= EPSILON)
                        return false;

                    const double cycle_height = std::min<double>(gradient_nominal_height, remaining_domain_height);
                    // Sample at the nominal midpoint; only the dominant solid-zone pass grows.
                    const double progress = std::clamp((state.z_cursor + .5 * cycle_height - domain_lo) / (domain_hi - domain_lo), 0.0, 1.0);
                    auto sample = sample_mixed_gradient_local_z(mixed_definitions[row_idx], num_physical, progress,
                                                                gradient_middle_filament_fraction, gradient_nominal_height,
                                                                min_sublayer_height, gradient_max_layer_heights);
                    // Preserve legacy two-color recipes that specify an endpoint range instead of explicit stops.
                    if (row_gradient_component_ids[row_idx].size() == 2 && row_gradient_stop_positions[row_idx].empty()) {
                        LocalZActivePair pair;
                        if (!effective_gradient_active_pair_for_z(row_idx, state.z_cursor + .5 * cycle_height, 0, pair))
                            return false;
                        sample.mix         = {pair.component_a, pair.component_b, pair.mix_b_percent};
                        const auto heights = mixed_filament_local_z_pair_heights(gradient_nominal_height, min_sublayer_height,
                                                                                 pair.mix_b_percent);
                        sample.height_a    = heights.first;
                        sample.height_b    = heights.second;
                    }
                    state.gradient_cadence    = build_local_z_independent_gradient_cadence(sample, remaining_domain_height,
                                                                                           min_sublayer_height, state.last_extruder,
                                                                                           print_cfg);
                    state.gradient_pass_index = 0;
                    return !state.gradient_cadence.empty();
                };

                auto current_cadence_pass = [&]() -> const LocalZIndependentDirectPass* {
                    if (gradient_row) {
                        if (!ensure_gradient_cadence())
                            return nullptr;
                        return &state.gradient_cadence[state.gradient_pass_index];
                    }
                    if (direct_cadence.empty())
                        return nullptr;
                    return &direct_cadence[state.cadence_index % direct_cadence.size()];
                };

                while (true) {
                    const LocalZIndependentDirectPass *cadence_pass = current_cadence_pass();
                    if (cadence_pass == nullptr)
                        break;
                    const double next_z = state.z_cursor + cadence_pass->height;
                    if (next_z > interval.z_hi + EPSILON)
                        break;

                    append_independent_pass(std::min(next_z, interval.z_hi),
                                            cadence_pass->height,
                                            cadence_pass->extruder_id);
                    state.z_cursor = next_z;
                    state.last_extruder = cadence_pass->extruder_id;
                    if (gradient_row)
                        ++state.gradient_pass_index;
                    else
                        ++state.cadence_index;
                    if (state.z_cursor >= interval.z_hi - EPSILON)
                        break;
                }

                const bool continues_next_layer =
                    layer_id + 1 < print_object.layer_count() &&
                    mixed_filament_local_z_should_subdivide_layer(layer_id + 1,
                                                                  local_z_whole_objects,
                                                                  local_z_preserve_first_layer) &&
                    row_is_active_on_layer(row_idx, layer_id + 1);
                if (!continues_next_layer) {
                    const double remaining_height = interval.z_hi - state.z_cursor;
                    if (remaining_height > EPSILON) {
                        const LocalZIndependentDirectPass *cadence_pass = current_cadence_pass();
                        if (cadence_pass != nullptr) {
                            bool merged_short_remainder = false;
                            if (adaptive_independent && remaining_height < min_sublayer_height - EPSILON) {
                                const size_t dependency_group = row_idx + 1;
                                const auto previous = std::find_if(
                                    independent_plans.rbegin(),
                                    independent_plans.rend(),
                                    [dependency_group](const SubLayerPlan &plan) {
                                        return plan.dependency_group == dependency_group;
                                    });
                                if (previous != independent_plans.rend()) {
                                    previous->z_hi        = interval.z_hi;
                                    previous->print_z     = interval.z_hi;
                                    previous->flow_height += remaining_height;
                                    merged_short_remainder = true;
                                }
                            }
                            if (!merged_short_remainder) {
                                append_independent_pass(interval.z_hi, remaining_height, cadence_pass->extruder_id);
                                state.last_extruder = cadence_pass->extruder_id;
                            }
                        }
                    }
                    state.active = false;
                }
            }

            if (local_z_whole_objects && !fixed_state_masks_union.empty()) {
                const size_t fixed_dependency_group = mixed_definitions.size() + 1;
                for (size_t extruder_idx = 0; extruder_idx < fixed_state_masks_by_extruder.size(); ++extruder_idx) {
                    if (fixed_state_masks_by_extruder[extruder_idx].empty())
                        continue;
                    SubLayerPlan plan;
                    plan.layer_id         = layer_id;
                    plan.pass_index       = independent_plans.size();
                    plan.split_interval   = true;
                    plan.z_lo             = interval.z_lo;
                    plan.z_hi             = interval.z_hi;
                    plan.print_z          = interval.z_hi;
                    plan.flow_height      = interval.base_height;
                    plan.dependency_group = fixed_dependency_group;
                    plan.dependency_order = extruder_idx;
                    plan.painted_masks_by_extruder.assign(num_physical, ExPolygons());
                    plan.fixed_painted_masks_by_extruder.assign(num_physical, ExPolygons());
                    append(plan.fixed_painted_masks_by_extruder[extruder_idx], fixed_state_masks_by_extruder[extruder_idx]);
                    independent_plans.emplace_back(std::move(plan));
                    ++split_passes_total;
                    ++split_passes_with_painted_masks;
                }
            }

            std::sort(independent_plans.begin(), independent_plans.end(), [](const SubLayerPlan &lhs, const SubLayerPlan &rhs) {
                if (std::abs(lhs.print_z - rhs.print_z) > EPSILON)
                    return lhs.print_z < rhs.print_z;
                if (lhs.dependency_group != rhs.dependency_group)
                    return lhs.dependency_group < rhs.dependency_group;
                return lhs.dependency_order < rhs.dependency_order;
            });

            double min_pass_height = interval.base_height;
            double max_pass_height = 0.0;
            for (size_t pass_idx = 0; pass_idx < independent_plans.size(); ++pass_idx) {
                SubLayerPlan &plan = independent_plans[pass_idx];
                plan.pass_index = pass_idx;
                min_pass_height = std::min(min_pass_height, plan.flow_height);
                max_pass_height = std::max(max_pass_height, plan.flow_height);
                if (pass_idx + 1 == independent_plans.size())
                    plan.base_masks = base_masks;
                plans.emplace_back(std::move(plan));
                ++interval.sublayer_count;
                ++total_generated_sublayer_cnt;
            }
            interval.sublayer_height = independent_plans.empty() ? interval.base_height : min_pass_height;

            BOOST_LOG_TRIVIAL(debug) << "Local-Z independent interval"
                                     << " object=" << object_name
                                     << " layer_id=" << layer_id
                                     << " base_height=" << interval.base_height
                                     << " pass_count=" << interval.sublayer_count
                                     << " pass_min_height=" << (interval.sublayer_count == 0 ? 0.0 : min_pass_height)
                                     << " pass_max_height=" << max_pass_height
                                     << " active_mixed_definitions=" << active_mixed_definitions
                                     << " managed_mask_count=" << interval.managed_masks.size();

            row_active_prev_layer = row_active_this_layer;
            intervals.emplace_back(std::move(interval));
            continue;
        }

        for (IndependentCadenceState &state : row_independent_states)
            state = IndependentCadenceState{};

        std::vector<std::vector<double>> isolated_row_pass_heights(mixed_definitions.size());
        bool isolated_multi_row_mode = false;
        const bool shared_multi_row_fallback =
            LOCAL_Z_SHARED_FALLBACK_ENABLED &&
            interval.has_mixed_paint &&
            preferred_a <= EPSILON &&
            preferred_b <= EPSILON &&
            active_mixed_definitions > 1 &&
            (active_mixed_definitions > LOCAL_Z_MAX_ISOLATED_ACTIVE_ROWS ||
             active_row_mask_components > LOCAL_Z_MAX_ISOLATED_MASK_COMPONENTS ||
             active_row_mask_vertices > LOCAL_Z_MAX_ISOLATED_MASK_VERTICES);
        if (shared_multi_row_fallback)
            ++shared_multi_row_fallback_intervals;
        if (interval.has_mixed_paint &&
            subdivide_this_layer &&
            preferred_a <= EPSILON &&
            preferred_b <= EPSILON &&
            !shared_multi_row_fallback &&
            active_mixed_definitions > 1) {
            size_t isolated_rows_with_split = 0;
            for (size_t row_idx = 0; row_idx < row_active_this_layer.size(); ++row_idx) {
                if (row_active_this_layer[row_idx] == 0)
                    continue;

                std::vector<double> row_passes;
                bool                row_is_gradient = false;
                if (row_uses_direct_multicolor_solver[row_idx] != 0) {
                    row_passes = build_direct_pass_heights_for_definition(row_idx, layer_id, interval.base_height);
                } else {
                    double row_h_a = 0.0;
                    double row_h_b = 0.0;
                    const LocalZActivePair &active_pair = row_active_pairs[row_idx];
                    const int row_mix_b_percent =
                        active_pair.valid_pair(num_physical) ? active_pair.mix_b_percent :
                                                               mixed_definitions[row_idx]
                                                                   .recipe.blend.primary_pair_or().component_b_percent;
                    row_is_gradient = effective_gradient_heights_for_definition(row_idx, layer_id, interval.base_height, row_h_a, row_h_b);
                    if (!row_is_gradient)
                        compute_local_z_component_heights(row_mix_b_percent,
                                                          interval.base_height,
                                                          min_sublayer_height,
                                                          row_h_a,
                                                          row_h_b);
                    if (row_is_gradient)
                        std::swap(row_h_a, row_h_b);
                    row_passes = (row_is_gradient || active_pair.uses_layer_cycle_sequence)
                        ? build_local_z_two_pass_heights(interval.base_height, min_sublayer_height, row_h_a, row_h_b)
                        : build_local_z_alternating_pass_heights(interval.base_height,
                                                                 min_sublayer_height,
                                                                 row_h_a,
                                                                 row_h_b);
                }
                if (row_passes.empty())
                    row_passes.emplace_back(interval.base_height);
                if (!sanitize_local_z_pass_heights(row_passes, interval.base_height, min_sublayer_height))
                    row_passes = build_uniform_local_z_pass_heights(interval.base_height, min_sublayer_height);
                // Match the canonical two-pass order used by the single-row path. Otherwise entering or leaving
                // isolated multi-row mode reverses unequal A/B passes and duplicates a component at the boundary.
                if (!row_is_gradient && row_passes.size() == 2 && row_passes[0] > row_passes[1])
                    std::swap(row_passes[0], row_passes[1]);
                if (row_passes.size() > 1)
                    ++isolated_rows_with_split;
                isolated_row_pass_heights[row_idx] = std::move(row_passes);
            }
            if (isolated_rows_with_split > 0) {
                isolated_multi_row_mode = true;
                ++alternating_height_intervals;
            }
        }

        std::vector<double> pass_heights;
        if (interval.has_mixed_paint && subdivide_this_layer && !isolated_multi_row_mode) {
            // Local-Z mode emits an A/B pair for mixed regions and derives its
            // direct thickness ratio from the nominal layer height.
            if (preferred_a <= EPSILON && preferred_b <= EPSILON) {
                if (shared_multi_row_fallback) {
                    pass_heights = build_local_z_shared_pass_heights(interval.base_height, min_sublayer_height);
                    if (pass_heights.size() > 1)
                        ++alternating_height_intervals;
                } else if (dominant_mixed_idx < mixed_definitions.size() &&
                           row_uses_direct_multicolor_solver[dominant_mixed_idx] != 0) {
                    pass_heights = build_direct_pass_heights_for_definition(dominant_mixed_idx, layer_id, interval.base_height);
                    if (pass_heights.size() > 1)
                        ++alternating_height_intervals;
                } else if (dominant_gradient_valid) {
                    if (dominant_is_gradient) {
                        pass_heights = build_local_z_two_pass_heights(interval.base_height,
                                                                      min_sublayer_height,
                                                                      dominant_gradient_h_b,
                                                                      dominant_gradient_h_a);
                    } else {
                        const bool dominant_uses_pair_cycle =
                            dominant_mixed_idx < mixed_definitions.size() && row_active_pairs[dominant_mixed_idx].uses_layer_cycle_sequence;
                        pass_heights = dominant_uses_pair_cycle
                            ? build_local_z_two_pass_heights(interval.base_height, min_sublayer_height,
                                                             dominant_gradient_h_a, dominant_gradient_h_b)
                            : build_local_z_alternating_pass_heights(interval.base_height,
                                                                     min_sublayer_height,
                                                                     dominant_gradient_h_a,
                                                                     dominant_gradient_h_b);
                    }
                    if (pass_heights.size() > 1)
                        ++alternating_height_intervals;
                } else {
                    pass_heights = build_uniform_local_z_pass_heights(interval.base_height, min_sublayer_height);
                }
            } else {
                pass_heights = build_local_z_pass_heights(interval.base_height,
                                                          min_sublayer_height,
                                                          preferred_a,
                                                          preferred_b);
            }
        }
        else
            pass_heights.emplace_back(interval.base_height);

        if (interval.has_mixed_paint) {
            if (!sanitize_local_z_pass_heights(pass_heights, interval.base_height, min_sublayer_height))
                pass_heights = build_uniform_local_z_pass_heights(interval.base_height, min_sublayer_height);
        }

        // Keep auto local-Z 2-pass cadence order stable across layers even if the
        // dominant mixed row changes. Per-row phase assignment still controls
        // which filament gets pass-0 vs pass-1.
        if (!dominant_is_gradient &&
            interval.has_mixed_paint &&
            preferred_a <= EPSILON &&
            preferred_b <= EPSILON &&
            pass_heights.size() == 2 &&
            pass_heights[0] > pass_heights[1]) {
            std::swap(pass_heights[0], pass_heights[1]);
        }

        size_t pass_count_for_log = pass_heights.size();
        double pass_min_height_for_log = pass_heights.empty() ? 0.0 : *std::min_element(pass_heights.begin(), pass_heights.end());
        double pass_max_height_for_log = pass_heights.empty() ? 0.0 : *std::max_element(pass_heights.begin(), pass_heights.end());

        const bool split_interval = interval.has_mixed_paint && (isolated_multi_row_mode || pass_heights.size() > 1);
        const bool force_height_resolve = true;
        auto build_whole_object_fixed_plans = [&](size_t first_pass_index) {
            std::vector<SubLayerPlan> fixed_plans;
            if (!local_z_whole_objects || !subdivide_this_layer || fixed_state_masks_union.empty() || interval.base_height <= EPSILON)
                return fixed_plans;

            const std::vector<double> fixed_z_cuts {
                interval.z_lo,
                interval.z_lo + 0.5 * interval.base_height,
                interval.z_hi
            };
            const size_t fixed_pass_count      = fixed_z_cuts.size() - 1;
            const size_t fixed_dependency_group = mixed_definitions.size() + 1;
            for (size_t fixed_pass_idx = 0; fixed_pass_idx < fixed_pass_count; ++fixed_pass_idx) {
                const double z_lo = fixed_z_cuts[fixed_pass_idx];
                const double z_hi = fixed_z_cuts[fixed_pass_idx + 1];
                const double pass_height = z_hi - z_lo;
                if (pass_height <= EPSILON)
                    continue;

                const std::vector<ExPolygons> fixed_masks_for_pass =
                    build_local_z_transition_fixed_masks_for_pass(fixed_state_masks_by_extruder,
                                                                  prev_fixed_state_masks_by_extruder,
                                                                  next_fixed_state_masks_by_extruder,
                                                                  fixed_pass_idx,
                                                                  fixed_pass_count);

                SubLayerPlan fixed_plan;
                fixed_plan.layer_id         = layer_id;
                fixed_plan.pass_index       = first_pass_index + fixed_plans.size();
                fixed_plan.split_interval   = true;
                fixed_plan.z_lo             = z_lo;
                fixed_plan.z_hi             = z_hi;
                fixed_plan.print_z          = z_hi;
                fixed_plan.flow_height      = pass_height;
                fixed_plan.dependency_group = fixed_dependency_group;
                fixed_plan.dependency_order = fixed_pass_idx;
                fixed_plan.painted_masks_by_extruder.assign(num_physical, ExPolygons());
                fixed_plan.fixed_painted_masks_by_extruder.assign(num_physical, ExPolygons());

                bool plan_has_fixed_masks = false;
                for (size_t extruder_idx = 0; extruder_idx < fixed_masks_for_pass.size() &&
                                             extruder_idx < fixed_plan.fixed_painted_masks_by_extruder.size();
                     ++extruder_idx) {
                    if (fixed_masks_for_pass[extruder_idx].empty())
                        continue;
                    append(fixed_plan.fixed_painted_masks_by_extruder[extruder_idx], fixed_masks_for_pass[extruder_idx]);
                    plan_has_fixed_masks = true;
                }
                if (!plan_has_fixed_masks)
                    continue;

                for (ExPolygons &masks : fixed_plan.fixed_painted_masks_by_extruder)
                    if (masks.size() > 1)
                        masks = union_ex(masks);
                fixed_plans.emplace_back(std::move(fixed_plan));
            }
            return fixed_plans;
        };
        if (split_interval) {
            ++split_intervals;
            bool   interval_has_split_painted_masks = false;
            if (isolated_multi_row_mode) {
                std::vector<SubLayerPlan> isolated_plans;
                isolated_plans.reserve(std::max<size_t>(2, active_mixed_definitions * 2));

                for (size_t row_idx = 0; row_idx < row_active_this_layer.size(); ++row_idx) {
                    if (row_active_this_layer[row_idx] == 0)
                        continue;
                    const ExPolygons &state_masks = row_state_masks[row_idx];
                    if (state_masks.empty())
                        continue;

                    const std::vector<double> &row_passes_raw = isolated_row_pass_heights[row_idx];
                    const std::vector<double> row_passes = row_passes_raw.empty()
                        ? std::vector<double>{ interval.base_height }
                        : row_passes_raw;
                    const LocalZActivePair &active_pair = row_active_pairs[row_idx];
                    const bool uses_direct_multicolor = row_uses_direct_multicolor_solver[row_idx] != 0;
                    const bool valid_pair = active_pair.valid_pair(num_physical);
                    const int orientation_cadence_index = active_pair.uses_layer_cycle_sequence
                        ? row_layer_cycle_index[row_idx]
                        : row_cadence_index[row_idx];
                    const std::vector<unsigned int> direct_sequence = uses_direct_multicolor
                        ? build_direct_sequence_for_definition(row_idx, layer_id, row_passes)
                        : std::vector<unsigned int>();

                    bool start_with_a = true;
                    if (!uses_direct_multicolor && valid_pair && preferred_a <= EPSILON && preferred_b <= EPSILON) {
                        double row_h_a = 0.0;
                        double row_h_b = 0.0;
                        const bool row_is_gradient =
                            effective_gradient_heights_for_definition(row_idx, layer_id, interval.base_height, row_h_a, row_h_b);
                        if (!row_is_gradient)
                            compute_local_z_component_heights(active_pair.mix_b_percent,
                                                              interval.base_height,
                                                              min_sublayer_height,
                                                              row_h_a,
                                                              row_h_b);
                        start_with_a = row_is_gradient
                            ? false
                            : choose_local_z_start_with_component_a(row_passes, row_h_a, row_h_b, orientation_cadence_index);
                    }

                    double z_cursor = interval.z_lo;
                    bool   row_used = false;
                    size_t row_dependency_order = 0;
                    for (size_t pass_i = 0; pass_i < row_passes.size(); ++pass_i) {
                        if (z_cursor >= interval.z_hi - EPSILON)
                            break;

                        const double pass_height = std::min<double>(row_passes[pass_i], interval.z_hi - z_cursor);
                        if (pass_height <= EPSILON)
                            continue;
                        const double z_next = std::min<double>(interval.z_hi, z_cursor + pass_height);

                        SubLayerPlan plan;
                        plan.layer_id       = layer_id;
                        plan.pass_index     = isolated_plans.size();
                        plan.split_interval = true;
                        plan.z_lo           = z_cursor;
                        plan.z_hi           = z_next;
                        plan.print_z        = z_next;
                        plan.flow_height    = pass_height;
                        plan.dependency_group = row_idx + 1;
                        plan.dependency_order = row_dependency_order++;
                        plan.painted_masks_by_extruder.assign(num_physical, ExPolygons());
                        plan.fixed_painted_masks_by_extruder.assign(num_physical, ExPolygons());
                        ++split_passes_total;
                        ++forced_height_resolve_calls;

                        unsigned int target_extruder = 0;
                        if (uses_direct_multicolor) {
                            if (pass_i < direct_sequence.size())
                                target_extruder = direct_sequence[pass_i];
                        } else if (valid_pair) {
                            const bool even_pass = (pass_i % 2) == 0;
                            target_extruder = even_pass
                                ? (start_with_a ? active_pair.component_a : active_pair.component_b)
                                : (start_with_a ? active_pair.component_b : active_pair.component_a);
                            ++strict_ab_assignments;
                        }
                        if (target_extruder == 0) {
                            const unsigned int state_id = row_state_ids[row_idx];
                            if (state_id != 0) {
                                const int resolve_cadence_index = active_pair.uses_layer_cycle_sequence
                                    ? row_layer_cycle_index[row_idx]
                                    : row_cadence_index[row_idx];
                                target_extruder = mixed_mgr.resolve(state_id,
                                                                    num_physical,
                                                                    resolve_cadence_index,
                                                                    float(plan.print_z),
                                                                    float(plan.flow_height),
                                                                    force_height_resolve);
                            }
                        }
                        if (target_extruder == 0 || target_extruder > num_physical) {
                            ++forced_height_resolve_invalid_target;
                        } else {
                            append(plan.painted_masks_by_extruder[target_extruder - 1], state_masks);
                            if (!uses_direct_multicolor &&
                                row_idx < row_is_gradient_definition.size() &&
                                row_is_gradient_definition[row_idx] != 0 &&
                                row_idx < row_last_gradient_extruder.size())
                                row_last_gradient_extruder[row_idx] = target_extruder;
                            ++split_passes_with_painted_masks;
                            interval_has_split_painted_masks = true;
                        }
                        for (ExPolygons &masks : plan.painted_masks_by_extruder)
                            if (masks.size() > 1)
                                masks = union_ex(masks);

                        isolated_plans.emplace_back(std::move(plan));
                        row_used = true;
                        if (!uses_direct_multicolor && !active_pair.uses_layer_cycle_sequence)
                            ++row_cadence_index[row_idx];
                        z_cursor = z_next;
                    }
                    if (row_used && !uses_direct_multicolor && active_pair.uses_layer_cycle_sequence)
                        ++row_layer_cycle_index[row_idx];
                }

                if (!isolated_plans.empty()) {
                    auto sort_local_z_plans = [](std::vector<SubLayerPlan> &plans) {
                        std::sort(plans.begin(), plans.end(), [](const SubLayerPlan &lhs, const SubLayerPlan &rhs) {
                            if (std::abs(lhs.print_z - rhs.print_z) > EPSILON)
                                return lhs.print_z < rhs.print_z;
                            if (std::abs(lhs.z_lo - rhs.z_lo) > EPSILON)
                                return lhs.z_lo < rhs.z_lo;
                            return lhs.pass_index < rhs.pass_index;
                        });
                    };

                    sort_local_z_plans(isolated_plans);

                    std::vector<SubLayerPlan> fixed_plans = build_whole_object_fixed_plans(isolated_plans.size());
                    if (!fixed_plans.empty()) {
                        isolated_plans.insert(isolated_plans.end(),
                                              std::make_move_iterator(fixed_plans.begin()),
                                              std::make_move_iterator(fixed_plans.end()));
                        sort_local_z_plans(isolated_plans);
                    }

                    double min_flow_height = isolated_plans.front().flow_height;
                    double max_flow_height = isolated_plans.front().flow_height;
                    for (size_t idx = 0; idx < isolated_plans.size(); ++idx) {
                        isolated_plans[idx].pass_index = idx;
                        min_flow_height = std::min(min_flow_height, isolated_plans[idx].flow_height);
                        max_flow_height = std::max(max_flow_height, isolated_plans[idx].flow_height);
                        for (ExPolygons &masks : isolated_plans[idx].painted_masks_by_extruder)
                            if (masks.size() > 1)
                                masks = union_ex(masks);
                        for (ExPolygons &masks : isolated_plans[idx].fixed_painted_masks_by_extruder)
                            if (masks.size() > 1)
                                masks = union_ex(masks);
                        if (std::any_of(isolated_plans[idx].fixed_painted_masks_by_extruder.begin(),
                                        isolated_plans[idx].fixed_painted_masks_by_extruder.end(),
                                        [](const ExPolygons &masks) { return !masks.empty(); })) {
                            ++split_passes_with_painted_masks;
                            interval_has_split_painted_masks = true;
                        }
                    }
                    isolated_plans.back().base_masks = base_masks;
                    interval.sublayer_height = min_flow_height;
                    pass_count_for_log       = isolated_plans.size();
                    pass_min_height_for_log  = min_flow_height;
                    pass_max_height_for_log  = max_flow_height;
                    for (SubLayerPlan &plan : isolated_plans) {
                        plans.emplace_back(std::move(plan));
                        ++interval.sublayer_count;
                        ++total_generated_sublayer_cnt;
                    }
                }
            } else {
                // Derive per-row orientation against pass heights so each mixed row
                // maps thicker/thinner subpasses to the intended component.
                std::vector<uint8_t> start_with_component_a(mixed_definitions.size(), uint8_t(1));
                std::vector<std::vector<unsigned int>> row_direct_pass_sequences(mixed_definitions.size());
                size_t single_dependency_group = 0;
                size_t active_dependency_rows = 0;
                for (size_t row_idx = 0; row_idx < row_state_masks.size(); ++row_idx) {
                    if (row_state_masks[row_idx].empty() || row_state_ids[row_idx] == 0)
                        continue;
                    ++active_dependency_rows;
                    single_dependency_group = row_idx + 1;
                }
                if (active_dependency_rows != 1)
                    single_dependency_group = 0;
                if (preferred_a <= EPSILON && preferred_b <= EPSILON) {
                    for (size_t row_idx = 0; row_idx < row_active_this_layer.size(); ++row_idx) {
                        if (row_active_this_layer[row_idx] == 0 || !local_z_eligible_mixed_definition(mixed_definitions[row_idx]))
                            continue;
                        if (row_uses_direct_multicolor_solver[row_idx] != 0) {
                            row_direct_pass_sequences[row_idx] =
                                build_direct_sequence_for_definition(row_idx, layer_id, pass_heights);
                            continue;
                        }

                        const LocalZActivePair &active_pair = row_active_pairs[row_idx];
                        if (!active_pair.valid_pair(num_physical))
                            continue;

                        double row_h_a = 0.0;
                        double row_h_b = 0.0;
                        const int orientation_cadence_index = active_pair.uses_layer_cycle_sequence
                            ? row_layer_cycle_index[row_idx]
                            : row_cadence_index[row_idx];
                        const bool row_is_gradient =
                            effective_gradient_heights_for_definition(row_idx, layer_id, interval.base_height, row_h_a, row_h_b);
                        if (!row_is_gradient)
                            compute_local_z_component_heights(active_pair.mix_b_percent,
                                                              interval.base_height,
                                                              min_sublayer_height,
                                                              row_h_a,
                                                              row_h_b);
                        start_with_component_a[row_idx] = row_is_gradient
                            ? uint8_t(0)
                            : (choose_local_z_start_with_component_a(pass_heights,
                                                                      row_h_a,
                                                                      row_h_b,
                                                                      orientation_cadence_index) ? uint8_t(1) : uint8_t(0));
                    }
                }

                double z_cursor = interval.z_lo;
                size_t pass_idx = 0;
                interval.sublayer_height = *std::min_element(pass_heights.begin(), pass_heights.end());
                std::vector<uint8_t> row_seen_sequence_in_interval(mixed_definitions.size(), uint8_t(0));
                for (const double pass_height_nominal : pass_heights) {
                    if (z_cursor >= interval.z_hi - EPSILON)
                        break;
                    const double pass_height = std::min<double>(pass_height_nominal, interval.z_hi - z_cursor);
                    const double z_next      = std::min<double>(interval.z_hi, z_cursor + pass_height);

                    SubLayerPlan plan;
                    plan.layer_id       = layer_id;
                    plan.pass_index     = pass_idx;
                    plan.split_interval = true;
                    plan.z_lo           = z_cursor;
                    plan.z_hi           = z_next;
                    plan.print_z        = z_next;
                    plan.flow_height    = pass_height;
                    plan.dependency_group = single_dependency_group;
                    plan.dependency_order = pass_idx;
                    plan.painted_masks_by_extruder.assign(num_physical, ExPolygons());
                    plan.fixed_painted_masks_by_extruder.assign(num_physical, ExPolygons());
                    ++split_passes_total;
                    bool pass_has_painted_masks = false;
                    std::vector<uint8_t> row_seen_in_pass(mixed_definitions.size(), uint8_t(0));

                    for (size_t row_idx = 0; row_idx < row_state_masks.size(); ++row_idx) {
                        const ExPolygons &state_masks = row_state_masks[row_idx];
                        if (state_masks.empty())
                            continue;

                        const unsigned int state_id = row_state_ids[row_idx];
                        if (state_id == 0)
                            continue;
                        const MixedFilamentDefinition &definition = mixed_definitions[row_idx];
                        if (!local_z_eligible_mixed_definition(definition))
                            continue;
                        const LocalZActivePair &active_pair = row_active_pairs[row_idx];
                        const bool uses_direct_multicolor = row_uses_direct_multicolor_solver[row_idx] != 0;
                        row_seen_in_pass[row_idx] = uint8_t(1);
                        if (!uses_direct_multicolor && active_pair.uses_layer_cycle_sequence)
                            row_seen_sequence_in_interval[row_idx] = uint8_t(1);
                        ++forced_height_resolve_calls;
                        unsigned int target_extruder = 0;
                        if (uses_direct_multicolor) {
                            if (pass_idx < row_direct_pass_sequences[row_idx].size())
                                target_extruder = row_direct_pass_sequences[row_idx][pass_idx];
                        } else if (active_pair.valid_pair(num_physical)) {
                            const bool start_a = start_with_component_a[row_idx] != 0;
                            const bool even_pass = (pass_idx % 2) == 0;
                            // Local-Z mode alternates A/B on every subpass.
                            target_extruder = even_pass
                                ? (start_a ? active_pair.component_a : active_pair.component_b)
                                : (start_a ? active_pair.component_b : active_pair.component_a);
                            ++strict_ab_assignments;
                        }
                        if (target_extruder == 0) {
                            const int resolve_cadence_index = active_pair.uses_layer_cycle_sequence
                                ? row_layer_cycle_index[row_idx]
                                : row_cadence_index[row_idx];
                            target_extruder = mixed_mgr.resolve(state_id,
                                                                num_physical,
                                                                resolve_cadence_index,
                                                                float(plan.print_z),
                                                                float(plan.flow_height),
                                                                force_height_resolve);
                        }
                        if (target_extruder == 0 || target_extruder > num_physical) {
                            ++forced_height_resolve_invalid_target;
                            continue;
                        }
                        append(plan.painted_masks_by_extruder[target_extruder - 1], state_masks);
                        if (!uses_direct_multicolor &&
                            row_idx < row_is_gradient_definition.size() &&
                            row_is_gradient_definition[row_idx] != 0 &&
                            row_idx < row_last_gradient_extruder.size())
                            row_last_gradient_extruder[row_idx] = target_extruder;
                        pass_has_painted_masks = true;
                    }
                    for (ExPolygons &masks : plan.painted_masks_by_extruder)
                        if (masks.size() > 1)
                            masks = union_ex(masks);
                    for (ExPolygons &masks : plan.fixed_painted_masks_by_extruder)
                        if (masks.size() > 1)
                            masks = union_ex(masks);
                    if (pass_has_painted_masks) {
                        ++split_passes_with_painted_masks;
                        interval_has_split_painted_masks = true;
                    }

                    if (z_next >= interval.z_hi - EPSILON)
                        plan.base_masks = base_masks;

                    plans.emplace_back(std::move(plan));
                    ++interval.sublayer_count;
                    ++total_generated_sublayer_cnt;
                    ++pass_idx;
                    for (size_t mixed_idx = 0; mixed_idx < row_seen_in_pass.size(); ++mixed_idx)
                        if (row_seen_in_pass[mixed_idx] != 0 &&
                            row_uses_layer_cycle_pair[mixed_idx] == 0 &&
                            row_uses_direct_multicolor_solver[mixed_idx] == 0)
                            ++row_cadence_index[mixed_idx];
                    z_cursor = z_next;
                }
                std::vector<SubLayerPlan> fixed_plans = build_whole_object_fixed_plans(pass_idx);
                for (SubLayerPlan &fixed_plan : fixed_plans) {
                    interval.sublayer_height = std::min(interval.sublayer_height, fixed_plan.flow_height);
                    plans.emplace_back(std::move(fixed_plan));
                    ++interval.sublayer_count;
                    ++total_generated_sublayer_cnt;
                    ++split_passes_with_painted_masks;
                    interval_has_split_painted_masks = true;
                }
                for (size_t row_idx = 0; row_idx < row_seen_sequence_in_interval.size(); ++row_idx)
                    if (row_seen_sequence_in_interval[row_idx] != 0 &&
                        row_uses_direct_multicolor_solver[row_idx] == 0)
                        ++row_layer_cycle_index[row_idx];
            }
            if (!interval_has_split_painted_masks)
                ++split_intervals_without_painted_masks;
        } else {
            if (interval.has_mixed_paint)
                ++non_split_mixed_intervals;
            SubLayerPlan plan;
            plan.layer_id       = layer_id;
            plan.pass_index     = 0;
            plan.split_interval = false;
            plan.z_lo           = interval.z_lo;
            plan.z_hi           = interval.z_hi;
            plan.print_z        = interval.z_hi;
            plan.flow_height    = interval.base_height;
            plan.dependency_order = 0;
            plan.base_masks     = base_masks;
            plan.painted_masks_by_extruder.assign(num_physical, ExPolygons());
            plan.fixed_painted_masks_by_extruder.assign(num_physical, ExPolygons());
            std::vector<uint8_t> row_seen_in_interval(mixed_definitions.size(), uint8_t(0));

            for (size_t row_idx = 0; row_idx < row_state_masks.size(); ++row_idx) {
                const ExPolygons &state_masks = row_state_masks[row_idx];
                if (state_masks.empty())
                    continue;

                const unsigned int state_id = row_state_ids[row_idx];
                if (state_id == 0)
                    continue;
                const MixedFilamentDefinition &definition = mixed_definitions[row_idx];
                if (!local_z_eligible_mixed_definition(definition))
                    continue;
                row_seen_in_interval[row_idx] = uint8_t(1);
                ++forced_height_resolve_calls;
                unsigned int target_extruder = 0;
                if (row_uses_direct_multicolor_solver[row_idx] != 0) {
                    const std::vector<unsigned int> direct_sequence =
                        build_direct_sequence_for_definition(row_idx, layer_id, std::vector<double>{ interval.base_height });
                    if (!direct_sequence.empty())
                        target_extruder = direct_sequence.front();
                } else {
                    const int resolve_cadence_index = row_uses_layer_cycle_pair[row_idx] != 0
                        ? row_layer_cycle_index[row_idx]
                        : row_cadence_index[row_idx];
                    target_extruder =
                        mixed_mgr.resolve(state_id,
                                          num_physical,
                                          resolve_cadence_index,
                                          float(plan.print_z),
                                          float(plan.flow_height),
                                          force_height_resolve);
                }
                if (target_extruder == 0 || target_extruder > num_physical) {
                    ++forced_height_resolve_invalid_target;
                    continue;
                }
                append(plan.painted_masks_by_extruder[target_extruder - 1], state_masks);
                if (row_idx < row_is_gradient_definition.size() &&
                    row_is_gradient_definition[row_idx] != 0 &&
                    row_idx < row_last_gradient_extruder.size())
                    row_last_gradient_extruder[row_idx] = target_extruder;
            }
            for (size_t extruder_idx = 0; extruder_idx < fixed_state_masks_by_extruder.size(); ++extruder_idx)
                if (!fixed_state_masks_by_extruder[extruder_idx].empty())
                    append(plan.fixed_painted_masks_by_extruder[extruder_idx], fixed_state_masks_by_extruder[extruder_idx]);
            for (ExPolygons &masks : plan.painted_masks_by_extruder)
                if (masks.size() > 1)
                    masks = union_ex(masks);
            for (ExPolygons &masks : plan.fixed_painted_masks_by_extruder)
                if (masks.size() > 1)
                    masks = union_ex(masks);

            plans.emplace_back(std::move(plan));
            interval.sublayer_count = 1;
            ++total_generated_sublayer_cnt;
            for (size_t mixed_idx = 0; mixed_idx < row_seen_in_interval.size(); ++mixed_idx)
                if (row_seen_in_interval[mixed_idx] != 0)
                    if (row_uses_direct_multicolor_solver[mixed_idx] == 0)
                        (row_uses_layer_cycle_pair[mixed_idx] != 0 ? row_layer_cycle_index[mixed_idx] : row_cadence_index[mixed_idx])++;
        }

        if (interval.has_mixed_paint) {
            BOOST_LOG_TRIVIAL(debug) << "Local-Z interval"
                                     << " object=" << object_name
                                     << " layer_id=" << layer_id
                                     << " base_height=" << interval.base_height
                                     << " subdivision_allowed=" << (subdivide_this_layer ? 1 : 0)
                                     << " split=" << split_interval
                                     << " isolated_multi_row_mode=" << (isolated_multi_row_mode ? 1 : 0)
                                     << " shared_multi_row_fallback=" << (shared_multi_row_fallback ? 1 : 0)
                                     << " active_mixed_definitions=" << active_mixed_definitions
                                     << " active_row_mask_components=" << active_row_mask_components
                                     << " active_row_mask_vertices=" << active_row_mask_vertices
                                     << " mixed_states=" << mixed_state_count
                                     << " pass_count=" << pass_count_for_log
                                     << " pass_min_height=" << pass_min_height_for_log
                                     << " pass_max_height=" << pass_max_height_for_log
                                     << " mixed_mask_count=" << mixed_masks.size()
                                     << " base_mask_count=" << base_masks.size();
        }

        row_active_prev_layer = row_active_this_layer;
        intervals.emplace_back(std::move(interval));
    }

    if (!intervals.empty() && !plans.empty()) {
        print_object.set_local_z_plan(std::move(intervals), std::move(plans));
        // Prusa Colorin: Orca debug JSON dump (debug_out_path "out/") disabled - it crashed PrusaSlicer 2.9.6
        // right after the Local-Z plan was stored (diagnostic build #102). It only wrote a debug file.
        // export_local_z_plan_debug(print_object, min_sublayer_height);
        BOOST_LOG_TRIVIAL(warning) << "Local-Z plan built"
                                   << " object=" << object_name
                                   << " mixed_intervals=" << mixed_intervals
                                   << " split_intervals=" << split_intervals
                                   << " non_split_mixed_intervals=" << non_split_mixed_intervals
                                   << " split_intervals_without_painted_masks=" << split_intervals_without_painted_masks
                                   << " sublayer_passes=" << total_generated_sublayer_cnt
                                   << " split_passes_total=" << split_passes_total
                                   << " split_passes_with_painted_masks=" << split_passes_with_painted_masks
                                   << " alternating_height_intervals=" << alternating_height_intervals
                                   << " shared_multi_row_fallback_intervals=" << shared_multi_row_fallback_intervals
                                   << " max_isolated_active_rows=" << LOCAL_Z_MAX_ISOLATED_ACTIVE_ROWS
                                   << " max_isolated_mask_components=" << LOCAL_Z_MAX_ISOLATED_MASK_COMPONENTS
                                   << " max_isolated_mask_vertices=" << LOCAL_Z_MAX_ISOLATED_MASK_VERTICES
                                   << " strict_ab_assignments=" << strict_ab_assignments
                                   << " mixed_state_layers=" << total_mixed_state_layers
                                   << " forced_height_resolve_calls=" << forced_height_resolve_calls
                                   << " forced_height_resolve_invalid_target=" << forced_height_resolve_invalid_target
                                   << " min_sublayer_height=" << min_sublayer_height
                                   << " preferred_a=" << preferred_a
                                   << " preferred_b=" << preferred_b;
    } else {
        BOOST_LOG_TRIVIAL(warning) << "Local-Z plan empty after build"
                                   << " object=" << object_name
                                   << " intervals=" << intervals.size()
                                   << " plans=" << plans.size()
                                   << " mixed_intervals=" << mixed_intervals;
    }
}

// ---- end Prusa Colorin Local-Z parte 2 ----


static void colorin_resolve_mixed_channels(const PrintObject &print_object, std::vector<std::vector<ExPolygons>> &segmentation)
{
    const Print                &print        = *print_object.print();
    const PrintConfig          &print_cfg    = print.config();
    const MixedFilamentManager &mixed_mgr    = print.mixed_filament_manager();
    const size_t                num_physical = print_cfg.nozzle_diameter.size();
    if (segmentation.empty() || mixed_mgr.total_filaments(num_physical) <= num_physical)
        return;

    const float preferred_a      = float(print_cfg.mixed_color_layer_height_a.value);
    const float preferred_b      = float(print_cfg.mixed_color_layer_height_b.value);
    const float base_height      = std::max(0.01f, float(print_object.config().layer_height.value));
    const bool  collapse_enabled = print_cfg.mixed_filament_region_collapse.value;
    const bool  local_z_mode     = print_cfg.dithering_local_z_mode.value;

    std::atomic<size_t> pending_feature_channels{0};
    std::atomic<size_t> resolved_channels{0};
    tbb::parallel_for(tbb::blocked_range<size_t>(0, segmentation.size()),
        [&](const tbb::blocked_range<size_t> &range) {
            for (size_t layer_id = range.begin(); layer_id < range.end(); ++layer_id) {
                const Layer             &layer        = *print_object.get_layer(int(layer_id));
                std::vector<ExPolygons> &layer_slots  = segmentation[layer_id];
                for (size_t slot = num_physical; slot < layer_slots.size(); ++slot) {
                    if (layer_slots[slot].empty())
                        continue;
                    const unsigned int channel_id = unsigned(slot + 1);
                    if (!mixed_mgr.is_mixed(channel_id, num_physical))
                        continue;
                    const std::optional<MixedFilamentDefinition> definition =
                        mixed_mgr.mixed_filament_definition_from_id(channel_id, num_physical);
                    if (!collapse_enabled || (definition &&
                        (mixed_filament_definition_uses_local_z(*definition, local_z_mode) ||
                         definition->behavior.surface_bias.perimeter_modulation)))
                        {
                    // Prusa Colorin 4c-3b (Orca 0.9.18 PrintObjectSlice 4687-4705): keep the mixed channel as its
                    // own region; ToolOrdering / G-code resolve it to a physical tool per layer.
                    ++pending_feature_channels;
                    continue;
                }
                    const unsigned int physical_id = mixed_mgr.effective_painted_region_filament_id(
                        channel_id, num_physical, int(layer_id), float(layer.print_z), float(layer.height),
                        preferred_a, preferred_b, base_height);
                    if (physical_id < 1 || physical_id > num_physical)
                        continue;
                    append(layer_slots[physical_id - 1], std::move(layer_slots[slot]));
                    layer_slots[slot].clear();
                    ++resolved_channels;
                }
            }
        });

    BOOST_LOG_TRIVIAL(info) << "Prusa Colorin: mixed channels resolved per layer"
                            << " object=" << (print_object.model_object() ? print_object.model_object()->name : std::string("<unknown>"))
                            << " layers=" << segmentation.size()
                            << " resolved_layer_channels=" << resolved_channels.load()
                            << " physical_filaments=" << num_physical
                            << " total_filaments=" << mixed_mgr.total_filaments(num_physical);
    if (pending_feature_channels.load() > 0)
        BOOST_LOG_TRIVIAL(warning) << "Prusa Colorin: Local-Z / perimeter modulation / collapse-off channels kept as mixed regions"
                                   << " (resolved per layer in ToolOrdering)"
                                   << " layer_channels=" << pending_feature_channels.load();
}

template<typename ThrowOnCancel>
void apply_mm_segmentation(PrintObject &print_object, ThrowOnCancel throw_on_cancel)
{
    // Returns MM segmentation based on painting in MM segmentation gizmo
    std::vector<std::vector<ExPolygons>> segmentation = multi_material_segmentation_by_painting(print_object, throw_on_cancel);
    assert(segmentation.size() == print_object.layer_count());

    if (!print_object.print()->virtual_extruders().empty()) {
        std::vector<double> print_z_per_layer(segmentation.size());
        for (size_t layer_id = 0; layer_id < segmentation.size(); ++layer_id) {
            print_z_per_layer[layer_id] = print_object.get_layer(int(layer_id))->print_z;
        }

        FullSpectrum::remap_virtual_extruders_to_physical(
            segmentation,
            print_z_per_layer,
            print_object.print()->num_physical_extruders(),
            print_object.print()->virtual_extruders()
        );
    }

    // Prusa Colorin (FullSpectrum)
    // Prusa Colorin S3 (Orca 0.9.18): surface indentation and component bias offsets before mixed channels are resolved.
    apply_mixed_surface_indentation(print_object, segmentation);
    apply_mixed_component_surface_offsets(print_object, segmentation);
    // Prusa Colorin Local-Z parte 2 (Orca 0.9.18): construir el plan Local-Z con la segmentacion pintada.
    {
        std::vector<uint8_t> lz_adaptive_independent_rows;
        double               lz_adaptive_full_resolution_height_mm = 0.;
        const std::vector<uint8_t> lz_enabled_rows =
            local_z_enabled_mixed_rows(print_object, lz_adaptive_independent_rows, lz_adaptive_full_resolution_height_mm);
        bool has_whole_object_local_z_scope = false;
        std::vector<std::vector<ExPolygons>> local_z_segmentation =
            local_z_planner_segmentation_with_whole_object_mixed_wall(print_object, segmentation, lz_enabled_rows, has_whole_object_local_z_scope);
        build_local_z_plan(print_object, local_z_segmentation, has_whole_object_local_z_scope, lz_enabled_rows, lz_adaptive_independent_rows, throw_on_cancel);
    }
    colorin_resolve_mixed_channels(print_object, segmentation);

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, segmentation.size(), std::max(segmentation.size() / 128, size_t(1))),
        [&print_object, &segmentation, throw_on_cancel](const tbb::blocked_range<size_t> &range) {
            const auto  &layer_ranges   = print_object.shared_regions()->layer_ranges;
            double       z              = print_object.get_layer(int(range.begin()))->slice_z;
            auto         it_layer_range = layer_range_first(layer_ranges, z);
            const size_t num_extruders = print_object.print()->config().nozzle_diameter.size();

            struct ByExtruder {
                ExPolygons  expolygons;
                BoundingBox bbox;
            };

            struct ByRegion {
                ExPolygons expolygons;
                bool       needs_merge { false };
            };

            std::vector<ByExtruder> by_extruder;
            std::vector<ByRegion>   by_region;
            for (size_t layer_id = range.begin(); layer_id < range.end(); ++layer_id) {
                throw_on_cancel();
                Layer &layer = *print_object.get_layer(int(layer_id));
                it_layer_range = layer_range_next(layer_ranges, it_layer_range, layer.slice_z);
                const PrintObjectRegions::LayerRangeRegions &layer_range = *it_layer_range;
                // Gather per extruder expolygons.
                // Prusa Colorin 4c-3b: kept mixed channels live after the physical ones.
                const size_t layer_channels = std::max(num_extruders, segmentation[layer_id].size());
                by_extruder.assign(layer_channels, ByExtruder());
                by_region.assign(layer.region_count(), ByRegion());
                bool layer_split = false;
                for (size_t extruder_id = 0; extruder_id < layer_channels; ++ extruder_id) {
                    if (extruder_id >= segmentation[layer_id].size())
                        break;
                    ByExtruder &region = by_extruder[extruder_id];
                    append(region.expolygons, std::move(segmentation[layer_id][extruder_id]));
                    if (! region.expolygons.empty()) {
                        region.bbox = get_extents(region.expolygons);
                        layer_split = true;
                    }
                }

                if (!layer_split)
                    continue;

                // Split LayerRegions by by_extruder regions.
                // layer_range.painted_regions are sorted by extruder ID and parent PrintObject region ID.
                auto it_painted_region_begin = layer_range.painted_regions.cbegin();
                for (int parent_layer_region_idx = 0; parent_layer_region_idx < layer.region_count(); ++parent_layer_region_idx) {
                    if (it_painted_region_begin == layer_range.painted_regions.cend())
                        continue;

                    const LayerRegion &parent_layer_region = *layer.get_region(parent_layer_region_idx);
                    const PrintRegion &parent_print_region = parent_layer_region.region();
                    assert(parent_print_region.print_object_region_id() == parent_layer_region_idx);
                    if (parent_layer_region.slices().empty())
                        continue;

                    // Find the first PaintedRegion, which overrides the parent PrintRegion.
                    auto it_first_painted_region = std::find_if(it_painted_region_begin, layer_range.painted_regions.cend(), [&layer_range, &parent_print_region](const auto &painted_region) {
                        return layer_range.volume_regions[painted_region.parent].region->print_object_region_id() == parent_print_region.print_object_region_id();
                    });

                    if (it_first_painted_region == layer_range.painted_regions.cend())
                        continue; // This LayerRegion isn't overrides by any PaintedRegion.

                    assert(&parent_print_region == layer_range.volume_regions[it_first_painted_region->parent].region);

                    // Update the beginning PaintedRegion iterator for the next iteration.
                    it_painted_region_begin = it_first_painted_region;

                    const BoundingBox parent_layer_region_bbox = get_extents(parent_layer_region.slices().surfaces);
                    bool              self_trimmed             = false;
                    int               self_extruder_id         = -1; // 1-based extruder ID
                    for (int extruder_id = 1; extruder_id <= int(by_extruder.size()); ++extruder_id) {
                        const ByExtruder &segmented = by_extruder[extruder_id - 1];
                        if (!segmented.bbox.defined || !parent_layer_region_bbox.overlap(segmented.bbox))
                            continue;

                        // Find the first target region iterator.
                        auto it_target_region = std::find_if(it_painted_region_begin, layer_range.painted_regions.cend(), [extruder_id](const auto &painted_region) {
                            return int(painted_region.extruder_id) >= extruder_id;
                        });

                        assert(it_target_region != layer_range.painted_regions.end());
                        assert(layer_range.volume_regions[it_target_region->parent].region == &parent_print_region && int(it_target_region->extruder_id) == extruder_id);
                        // Prusa Colorin 4c-3b: never dereference a missing painted region (release builds skip the asserts).
                        if (it_target_region == layer_range.painted_regions.cend() || int(it_target_region->extruder_id) != extruder_id ||
                            layer_range.volume_regions[it_target_region->parent].region != &parent_print_region)
                            continue;

                        // Update the beginning PaintedRegion iterator for the next iteration.
                        it_painted_region_begin = it_target_region;

                        // FIXME: Don't trim by self, it is not reliable.
                        if (it_target_region->region == &parent_print_region) {
                            self_extruder_id = extruder_id;
                            continue;
                        }

                        // Steal from this region.
                        int        target_region_id = it_target_region->region->print_object_region_id();
                        ExPolygons stolen           = intersection_ex(parent_layer_region.slices().surfaces, segmented.expolygons);
                        if (!stolen.empty()) {
                            ByRegion &dst = by_region[target_region_id];
                            if (dst.expolygons.empty()) {
                                dst.expolygons = std::move(stolen);
                            } else {
                                append(dst.expolygons, std::move(stolen));
                                dst.needs_merge = true;
                            }
                        }
                    }

                    if (!self_trimmed) {
                        // Trim slices of this LayerRegion with all the MM regions.
                        Polygons mine = to_polygons(parent_layer_region.slices().surfaces);
                        for (auto &segmented : by_extruder) {
                            if (&segmented - by_extruder.data() + 1 != self_extruder_id && segmented.bbox.defined && parent_layer_region_bbox.overlap(segmented.bbox)) {
                                mine = diff(mine, segmented.expolygons);
                                if (mine.empty())
                                    break;
                            }
                        }

                        // Filter out unprintable polygons produced by subtraction multi-material painted regions from layerm.region().
                        // ExPolygon returned from multi-material segmentation does not precisely match ExPolygons in layerm.region()
                        // (because of preprocessing of the input regions in multi-material segmentation). Therefore, subtraction from
                        // layerm.region() could produce a huge number of small unprintable regions for the model's base extruder.
                        // This could, on some models, produce bulges with the model's base color (#7109).
                        if (!mine.empty()) {
                            mine = opening(union_ex(mine), scaled<float>(5. * EPSILON), scaled<float>(5. * EPSILON));
                        }

                        if (!mine.empty()) {
                            ByRegion &dst = by_region[parent_print_region.print_object_region_id()];
                            if (dst.expolygons.empty()) {
                                dst.expolygons = union_ex(mine);
                            } else {
                                append(dst.expolygons, union_ex(mine));
                                dst.needs_merge = true;
                            }
                        }
                    }
                }

                // Re-create Surfaces of LayerRegions.
                for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
                    ByRegion &src = by_region[region_id];
                    if (src.needs_merge) {
                        // Multiple regions were merged into one.
                        src.expolygons = closing_ex(src.expolygons, scaled<float>(10. * EPSILON));
                    }

                    layer.get_region(region_id)->m_slices.set(std::move(src.expolygons), stInternal);
                }

                // Prusa Colorin (Orca 0.9.18): diagnostico de emboss con mezcla tras la segmentacion.
                dump_surface_emboss_mixed_layer_state("post-mm-segmentation",
                                                      print_object,
                                                      layer_id,
                                                      layer,
                                                      layer_range,
                                                      &segmentation[layer_id]);
            }
        });
}

template<typename ThrowOnCancel>
void apply_fuzzy_skin_segmentation(PrintObject &print_object, ThrowOnCancel throw_on_cancel)
{
    // Returns fuzzy skin segmentation based on painting in the fuzzy skin painting gizmo.
    std::vector<std::vector<ExPolygons>> segmentation = fuzzy_skin_segmentation_by_painting(print_object, throw_on_cancel);
    assert(segmentation.size() == print_object.layer_count());

    struct ByRegion
    {
        ExPolygons expolygons;
        bool       needs_merge { false };
    };

    tbb::parallel_for(tbb::blocked_range<size_t>(0, segmentation.size(), std::max(segmentation.size() / 128, size_t(1))), [&print_object, &segmentation, throw_on_cancel](const tbb::blocked_range<size_t> &range) {
        const auto &layer_ranges   = print_object.shared_regions()->layer_ranges;
        auto        it_layer_range = layer_range_first(layer_ranges, print_object.get_layer(int(range.begin()))->slice_z);

        for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++layer_idx) {
            throw_on_cancel();

            Layer &layer = *print_object.get_layer(int(layer_idx));
            it_layer_range = layer_range_next(layer_ranges, it_layer_range, layer.slice_z);
            const PrintObjectRegions::LayerRangeRegions &layer_range = *it_layer_range;

            assert(segmentation[layer_idx].size() == 1);
            const ExPolygons &fuzzy_skin_segmentation      = segmentation[layer_idx][0];
            const BoundingBox fuzzy_skin_segmentation_bbox = get_extents(fuzzy_skin_segmentation);
            if (fuzzy_skin_segmentation.empty())
                continue;

            // Split LayerRegions by painted fuzzy skin regions.
            // layer_range.fuzzy_skin_painted_regions are sorted by parent PrintObject region ID.
            std::vector<ByRegion> by_region(layer.region_count());
            auto                  it_fuzzy_skin_region_begin = layer_range.fuzzy_skin_painted_regions.cbegin();
            for (int parent_layer_region_idx = 0; parent_layer_region_idx < layer.region_count(); ++parent_layer_region_idx) {
                if (it_fuzzy_skin_region_begin == layer_range.fuzzy_skin_painted_regions.cend())
                    continue;

                const LayerRegion &parent_layer_region = *layer.get_region(parent_layer_region_idx);
                const PrintRegion &parent_print_region = parent_layer_region.region();
                assert(parent_print_region.print_object_region_id() == parent_layer_region_idx);
                if (parent_layer_region.slices().empty())
                    continue;

                // Find the first FuzzySkinPaintedRegion, which overrides the parent PrintRegion.
                auto it_fuzzy_skin_region = std::find_if(it_fuzzy_skin_region_begin, layer_range.fuzzy_skin_painted_regions.cend(), [&layer_range, &parent_print_region](const auto &fuzzy_skin_region) {
                    return fuzzy_skin_region.parent_print_object_region_id(layer_range) == parent_print_region.print_object_region_id();
                });

                if (it_fuzzy_skin_region == layer_range.fuzzy_skin_painted_regions.cend())
                    continue; // This LayerRegion isn't overrides by any FuzzySkinPaintedRegion.

                assert(it_fuzzy_skin_region->parent_print_object_region(layer_range) == &parent_print_region);

                // Update the beginning FuzzySkinPaintedRegion iterator for the next iteration.
                it_fuzzy_skin_region_begin = std::next(it_fuzzy_skin_region);

                const BoundingBox parent_layer_region_bbox        = get_extents(parent_layer_region.slices().surfaces);
                Polygons          layer_region_remaining_polygons = to_polygons(parent_layer_region.slices().surfaces);
                // Don't trim by self, it is not reliable.
                if (parent_layer_region_bbox.overlap(fuzzy_skin_segmentation_bbox) && it_fuzzy_skin_region->region != &parent_print_region) {
                    // Steal from this region.
                    const int  target_region_id = it_fuzzy_skin_region->region->print_object_region_id();
                    ExPolygons stolen           = intersection_ex(parent_layer_region.slices().surfaces, fuzzy_skin_segmentation);
                    if (!stolen.empty()) {
                        ByRegion &dst = by_region[target_region_id];
                        if (dst.expolygons.empty()) {
                            dst.expolygons = std::move(stolen);
                        } else {
                            append(dst.expolygons, std::move(stolen));
                            dst.needs_merge = true;
                        }
                    }

                    // Trim slices of this LayerRegion by the fuzzy skin region.
                    layer_region_remaining_polygons = diff(layer_region_remaining_polygons, fuzzy_skin_segmentation);

                    // Filter out unprintable polygons. Detailed explanation is inside apply_mm_segmentation.
                    if (!layer_region_remaining_polygons.empty()) {
                        layer_region_remaining_polygons = opening(union_ex(layer_region_remaining_polygons), scaled<float>(5. * EPSILON), scaled<float>(5. * EPSILON));
                    }
                }

                if (!layer_region_remaining_polygons.empty()) {
                    ByRegion &dst = by_region[parent_print_region.print_object_region_id()];
                    if (dst.expolygons.empty()) {
                        dst.expolygons = union_ex(layer_region_remaining_polygons);
                    } else {
                        append(dst.expolygons, union_ex(layer_region_remaining_polygons));
                        dst.needs_merge = true;
                    }
                }
            }

            // Re-create Surfaces of LayerRegions.
            for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
                ByRegion &src = by_region[region_id];
                if (src.needs_merge) {
                    // Multiple regions were merged into one.
                    src.expolygons = closing_ex(src.expolygons, scaled<float>(10. * EPSILON));
                }

                layer.get_region(region_id)->m_slices.set(std::move(src.expolygons), stInternal);
            }
        }
    }); // end of parallel_for
}

// 1) Decides Z positions of the layers,
// 2) Initializes layers and their regions
// 3) Slices the object meshes
// 4) Slices the modifier meshes and reclassifies the slices of the object meshes by the slices of the modifier meshes
// 5) Applies size compensation (offsets the slices in XY plane)
// 6) Replaces bad slices by the slices reconstructed from the upper/lower layer
// Resulting expolygons of layer regions are marked as Internal.
//
// this should be idempotent
void PrintObject::slice_volumes()
{
    BOOST_LOG_TRIVIAL(info) << "Slicing volumes..." << log_memory_info();
    const Print *print                      = this->print();
    const auto   throw_on_cancel_callback   = std::function<void()>([print](){ print->throw_if_canceled(); });

    // Clear old LayerRegions, allocate for new PrintRegions.
    for (Layer* layer : m_layers) {
        layer->m_regions.clear();
        layer->m_regions.reserve(m_shared_regions->all_regions.size());
        for (const std::unique_ptr<PrintRegion> &pr : m_shared_regions->all_regions)
            layer->m_regions.emplace_back(new LayerRegion(layer, pr.get()));
    }

    std::vector<float>                   slice_zs      = zs_from_layers(m_layers);
    std::vector<std::vector<ExPolygons>> region_slices = slices_to_regions(this->model_object()->volumes, *m_shared_regions, slice_zs,
        slice_volumes_inner(
            print->config(), this->config(), this->trafo_centered(),
            this->model_object()->volumes, m_shared_regions->layer_ranges, slice_zs, throw_on_cancel_callback),
        throw_on_cancel_callback);

    for (size_t region_id = 0; region_id < region_slices.size(); ++ region_id) {
        std::vector<ExPolygons> &by_layer = region_slices[region_id];
        for (size_t layer_id = 0; layer_id < by_layer.size(); ++ layer_id)
            m_layers[layer_id]->regions()[region_id]->m_slices.append(std::move(by_layer[layer_id]), stInternal);
    }
    region_slices.clear();
    
    BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - removing top empty layers";
    while (! m_layers.empty()) {
        const Layer *layer = m_layers.back();
        if (! layer->empty())
            break;
        delete layer;
        m_layers.pop_back();
    }
    if (! m_layers.empty())
        m_layers.back()->upper_layer = nullptr;
    m_print->throw_if_canceled();

    // Prusa Colorin Local-Z parte 2 (Orca 0.9.18): filas de mezcla con Local-Z y aviso de altura adaptativa.
    std::vector<uint8_t>       colorin_adaptive_independent_local_z_rows;
    double                     colorin_adaptive_full_resolution_height_mm = 0.;
    const std::vector<uint8_t> colorin_local_z_enabled_rows =
        local_z_enabled_mixed_rows(*this, colorin_adaptive_independent_local_z_rows, colorin_adaptive_full_resolution_height_mm);
    if (colorin_adaptive_full_resolution_height_mm > EPSILON) {
        const double minimum_sublayer_height = std::max(0.01, double(m_print->config().mixed_filament_height_lower_bound.value));
        std::ostringstream warning;
        warning << L("Adaptive Local-Z height modulation was clipped by the configured minimum sublayer height of ")
                << std::fixed << std::setprecision(2) << minimum_sublayer_height
                << L(" mm while preserving the strict cadence-cycle top. To use the full adaptive ratio resolution, "
                     "decrease the Local-Z minimum "
                     "sublayer height to ")
                << std::fixed << std::setprecision(2) << colorin_adaptive_full_resolution_height_mm << L(" mm or lower.");
        this->active_step_add_warning(PrintStateBase::WarningLevel::NON_CRITICAL, warning.str());
    }

    // Is any ModelVolume multi-material painted?
    if (m_print->config().nozzle_diameter.size() > 1 && this->model_object()->is_mm_painted()) {
        // If XY Size compensation is also enabled, notify the user that XY Size compensation
        // would not be used because the object is multi-material painted.
        if (m_config.xy_size_compensation.value != 0.f) {
            this->active_step_add_warning(
                PrintStateBase::WarningLevel::CRITICAL,
                _u8L("An object has enabled XY Size compensation which will not be used because it is also multi-material painted.\nXY Size "
                  "compensation cannot be combined with multi-material painting.") +
                    "\n" + (_u8L("Object name")) + ": " + this->model_object()->name);
        }

        BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - MMU segmentation";
        apply_mm_segmentation(*this, [print]() { print->throw_if_canceled(); });
    }

    // Prusa Colorin S3 (Orca 0.9.18): mixed region surface offsets after segmentation is applied.
    apply_mixed_region_surface_offsets(*this);

    // Prusa Colorin Local-Z parte 2 (Orca 0.9.18): plan Local-Z de objeto completo si la segmentacion no genero intervalos.
    if (this->local_z_intervals().empty()) {
        std::vector<std::vector<ExPolygons>> whole_object_local_z_segmentation =
            whole_object_local_z_segmentation_by_mixed_wall(*this, colorin_local_z_enabled_rows);
        if (!whole_object_local_z_segmentation.empty())
            build_local_z_plan(*this,
                               whole_object_local_z_segmentation,
                               true,
                               colorin_local_z_enabled_rows,
                               colorin_adaptive_independent_local_z_rows,
                               [print]() { print->throw_if_canceled(); });
    }

    // Is any ModelVolume fuzzy skin painted?
    if (this->model_object()->is_fuzzy_skin_painted()) {
        // If XY Size compensation is also enabled, notify the user that XY Size compensation
        // would not be used because the object has custom fuzzy skin painted.
        if (m_config.xy_size_compensation.value != 0.f) {
            this->active_step_add_warning(
                PrintStateBase::WarningLevel::CRITICAL,
                _u8L("An object has enabled XY Size compensation which will not be used because it is also fuzzy skin painted.\nXY Size "
                     "compensation cannot be combined with fuzzy skin painting.") +
                    "\n" + (_u8L("Object name")) + ": " + this->model_object()->name);
        }

        BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - Fuzzy skin segmentation";
        apply_fuzzy_skin_segmentation(*this, [print]() { print->throw_if_canceled(); });
    }

    // Prusa Colorin S4 (Orca 0.9.18): surface emboss with mixed filament region override.
    apply_surface_emboss_mixed_region_override(*this, [print]() { print->throw_if_canceled(); });

    if (!m_print->virtual_extruders().empty()) {
        FullSpectrum::remap_virtual_region_slices_to_physical(
            *this,
            m_print->num_physical_extruders(),
            m_print->virtual_extruders()
        );
    }

    if (m_config.interlocking_beam) {
        BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - Applying multi-material interlocking";
        InterlockingGenerator::generate_interlocking_structure(*this);
        m_print->throw_if_canceled();
    }

    BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - make_slices in parallel - begin";
    {
        // Compensation value, scaled. Only applying the negative scaling here, as the positive scaling has already been applied during slicing.
        const size_t num_extruders = print->config().nozzle_diameter.size();
        const auto   xy_compensation_scaled            = (num_extruders > 1 && this->is_mm_painted()) ? scaled<float>(0.f) : scaled<float>(std::min(m_config.xy_size_compensation.value, 0.));
        const float  elephant_foot_compensation_scaled = (m_config.raft_layers == 0) ?
        	// Only enable Elephant foot compensation if printing directly on the print bed.
            float(scale_(m_config.elefant_foot_compensation.value)) :
        	0.f;
        // Uncompensated slices for the first layer in case the Elephant foot compensation is applied.
	    ExPolygons  lslices_1st_layer;
	    tbb::parallel_for(
	        tbb::blocked_range<size_t>(0, m_layers.size()),
			[this, xy_compensation_scaled, elephant_foot_compensation_scaled, &lslices_1st_layer](const tbb::blocked_range<size_t>& range) {
	            for (size_t layer_id = range.begin(); layer_id < range.end(); ++ layer_id) {
	                m_print->throw_if_canceled();
	                Layer *layer = m_layers[layer_id];
	                // Apply size compensation and perform clipping of multi-part objects.
	                float elfoot = (layer_id == 0) ? elephant_foot_compensation_scaled : 0.f;
	                if (layer->m_regions.size() == 1) {
	                    // Optimized version for a single region layer.
	                    // Single region, growing or shrinking.
	                    LayerRegion *layerm = layer->m_regions.front();
	                    if (elfoot > 0) {
		                    // Apply the elephant foot compensation and store the 1st layer slices without the Elephant foot compensation applied.
		                    lslices_1st_layer = to_expolygons(std::move(layerm->m_slices.surfaces));
		                    float delta = xy_compensation_scaled;
	                        if (delta > elfoot) {
	                            delta -= elfoot;
	                            elfoot = 0.f;
	                        } else if (delta > 0)
	                            elfoot -= delta;
							layerm->m_slices.set(
								union_ex(
									Slic3r::elephant_foot_compensation(
										(delta == 0.f) ? lslices_1st_layer : offset_ex(lslices_1st_layer, delta), 
	                            		layerm->flow(frExternalPerimeter), unscale<double>(elfoot))),
								stInternal);
							if (xy_compensation_scaled < 0.f)
								lslices_1st_layer = offset_ex(std::move(lslices_1st_layer), xy_compensation_scaled);
	                    } else if (xy_compensation_scaled < 0.f) {
	                        // Apply the XY compensation.
	                        layerm->m_slices.set(
                                offset_ex(to_expolygons(std::move(layerm->m_slices.surfaces)), xy_compensation_scaled),
	                            stInternal);
	                    }
	                } else {
	                    if (xy_compensation_scaled < 0.f || elfoot > 0.f) {
	                        // Apply the negative XY compensation.
	                        Polygons trimming;
	                        static const float eps = float(scale_(m_config.slice_closing_radius.value) * 1.5);
	                        if (elfoot > 0.f) {
	                        	lslices_1st_layer = offset_ex(layer->merged(eps), std::min(xy_compensation_scaled, 0.f) - eps);
								trimming = to_polygons(Slic3r::elephant_foot_compensation(lslices_1st_layer,
									layer->m_regions.front()->flow(frExternalPerimeter), unscale<double>(elfoot)));
	                        } else
		                        trimming = offset(layer->merged(float(SCALED_EPSILON)), xy_compensation_scaled - float(SCALED_EPSILON));
	                        for (size_t region_id = 0; region_id < layer->m_regions.size(); ++ region_id)
	                            layer->m_regions[region_id]->trim_surfaces(trimming);
	                    }
	                }
	                // Merge all regions' slices to get islands sorted topologically, chain them by a shortest path in separate index list
	                layer->make_slices();
	            }
	        });
	    if (elephant_foot_compensation_scaled > 0.f && ! m_layers.empty()) {
	    	// The Elephant foot has been compensated, therefore the 1st layer's lslices are shrank with the Elephant foot compensation value.
	    	// Store the uncompensated value there.
            //FIXME is this operation needed? MMU painting and brim now have to do work arounds to work with compensated layer, not with the uncompensated layer.
            // There may be subtle issues removing this block such as support raft sticking too well with the first object layer.
            Layer &layer = *m_layers.front();
	    	assert(layer.id() == 0);
			layer.lslices = std::move(lslices_1st_layer);
            layer.lslice_indices_sorted_by_print_order = chain_expolygons(layer.lslices);
		}
	}

    m_print->throw_if_canceled();
    BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - make_slices in parallel - end";
}

std::vector<Polygons> PrintObject::slice_support_volumes(const ModelVolumeType model_volume_type) const
{
    auto it_volume     = this->model_object()->volumes.begin();
    auto it_volume_end = this->model_object()->volumes.end();
    for (; it_volume != it_volume_end && (*it_volume)->type() != model_volume_type; ++ it_volume) ;
    std::vector<Polygons> slices;
    if (it_volume != it_volume_end) {
        // Found at least a single support volume of model_volume_type.
        std::vector<float> zs = zs_from_layers(this->layers());
        std::vector<char>  merge_layers;
        bool               merge = false;
        const Print       *print = this->print();
        auto               throw_on_cancel_callback = std::function<void()>([print](){ print->throw_if_canceled(); });
        MeshSlicingParamsEx params;
        params.trafo = this->trafo_centered();
        for (; it_volume != it_volume_end; ++ it_volume)
            if ((*it_volume)->type() == model_volume_type) {
                std::vector<ExPolygons> slices2 = slice_volume(*(*it_volume), zs, params, throw_on_cancel_callback);
                if (slices.empty()) {
                    slices.reserve(slices2.size());
                    for (ExPolygons &src : slices2)
                        slices.emplace_back(to_polygons(std::move(src)));
                } else if (!slices2.empty()) {
                    if (merge_layers.empty())
                        merge_layers.assign(zs.size(), false);
                    for (size_t i = 0; i < zs.size(); ++ i) {
                        if (slices[i].empty())
                            slices[i] = to_polygons(std::move(slices2[i]));
                        else if (! slices2[i].empty()) {
                            append(slices[i], to_polygons(std::move(slices2[i])));
                            merge_layers[i] = true;
                            merge = true;
                        }
                    }
                }
            }
        if (merge) {
            std::vector<Polygons*> to_merge;
            to_merge.reserve(zs.size());
            for (size_t i = 0; i < zs.size(); ++ i)
                if (merge_layers[i])
                    to_merge.emplace_back(&slices[i]);
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, to_merge.size()),
                [&to_merge](const tbb::blocked_range<size_t> &range) {
                    for (size_t i = range.begin(); i < range.end(); ++ i)
                        *to_merge[i] = union_(*to_merge[i]);
            });
        }
    }
    return slices;
}

} // namespace Slic3r
