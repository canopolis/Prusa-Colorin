// Prusa Colorin (FullSpectrum): ver ColorinObjImport.hpp.
#include "ColorinObjImport.hpp"

#include "Model.hpp"
#include "TriangleMesh.hpp"
#include "Format/OBJImageMap.hpp"
#include "ImageMap/VolumeData.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include <boost/log/trivial.hpp>

namespace Slic3r {

static const std::vector<std::string> CONST_FILAMENTS = {
    "", "4", "8", "0C", "1C", "2C", "3C", "4C", "5C", "6C", "7C", "8C", "9C", "AC", "BC", "CC", "DC",
}; // 5                           10                            15    16

static void get_real_filament_id(const unsigned char &id, std::string &result) {
    if (id < CONST_FILAMENTS.size()) {
        result = CONST_FILAMENTS[id];
    } else {
        result = "";//error
    }
};

bool obj_import_vertex_color_deal(const std::vector<unsigned char> &vertex_filament_ids, const unsigned char &first_extruder_id, Model *model)
{
    if (vertex_filament_ids.size() == 0) {
        return false;
    }
    // 2.generate mm_segmentation_facets
    if (model->objects.size() == 1 ) {
        auto obj = model->objects[0];
        obj->config.set("extruder", int(first_extruder_id));
        if (obj->volumes.size() == 1) {
            enum VertexColorCase {
                _3_SAME_COLOR,
                _3_DIFF_COLOR,
                _2_SAME_1_DIFF_COLOR,
            };
            auto calc_vertex_color_case = [](const unsigned char &c0, const unsigned char &c1, const unsigned char &c2, VertexColorCase &vertex_color_case,
                                             unsigned char &iso_index) {
                if (c0 == c1 && c1 == c2) {
                    vertex_color_case = VertexColorCase::_3_SAME_COLOR;
                } else if (c0 != c1 && c1 != c2 && c0 != c2) {
                    vertex_color_case = VertexColorCase::_3_DIFF_COLOR;
                } else if (c0 == c1) {
                    vertex_color_case = _2_SAME_1_DIFF_COLOR;
                    iso_index         = 2;
                } else if (c1 == c2) {
                    vertex_color_case = _2_SAME_1_DIFF_COLOR;
                    iso_index         = 0;
                } else if (c0 == c2) {
                    vertex_color_case = _2_SAME_1_DIFF_COLOR;
                    iso_index         = 1;
                } else {
                    std::cout << "error";
                }
            };
            auto calc_tri_area = [](const Vec3f &v0, const Vec3f &v1, const Vec3f &v2) {
                return std::abs((v0 - v1).cross(v0 - v2).norm()) / 2;
            };
            auto volume = obj->volumes[0];
            volume->config.set("extruder", int(first_extruder_id));
            auto face_count = volume->mesh().its.indices.size();
            volume->mm_segmentation_facets.reserve(face_count);
            if (volume->mesh().its.vertices.size() != vertex_filament_ids.size()) {
                return false;
            }
            for (size_t i = 0; i < volume->mesh().its.indices.size(); i++) {
                auto face   = volume->mesh().its.indices[i];
                auto filament_id0 = vertex_filament_ids[face[0]];
                auto filament_id1 = vertex_filament_ids[face[1]];
                auto filament_id2 = vertex_filament_ids[face[2]];
                if (filament_id0 <= 1 && filament_id1 <= 1 && filament_id2 <= 2) {
                    continue;
                }
                if (i == 0) {
                    std::cout << "";
                }
                VertexColorCase vertex_color_case;
                unsigned char iso_index;
                calc_vertex_color_case(filament_id0, filament_id1, filament_id2, vertex_color_case, iso_index);
                switch (vertex_color_case) {
                case _3_SAME_COLOR: {
                    std::string result;
                    get_real_filament_id(filament_id0, result);
                    volume->mm_segmentation_facets.set_triangle_from_string(i, result);
                    break;
                }
                case _3_DIFF_COLOR: {
                    std::string result0, result1, result2;
                    get_real_filament_id(filament_id0, result0);
                    get_real_filament_id(filament_id1, result1);
                    get_real_filament_id(filament_id2, result2);

                    auto v0 = volume->mesh().its.vertices[face[0]];
                    auto v1 = volume->mesh().its.vertices[face[1]];
                    auto v2 = volume->mesh().its.vertices[face[2]];
                    auto                 dir_0_1  = (v1 - v0).normalized().eval();
                    auto                 dir_0_2  = (v2 - v0).normalized().eval();
                    float                sita0    = acos(dir_0_1.dot(dir_0_2));
                    auto                 dir_1_0  = (-dir_0_1).eval();
                    auto                 dir_1_2  = (v2 - v1).normalized().eval();
                    float                sita1    = acos(dir_1_0.dot(dir_1_2));
                    float                sita2    = PI - sita0 - sita1;
                    std::array<float, 3> sitas    = {sita0, sita1, sita2};
                    float                max_sita = sitas[0];
                    int                  max_sita_vertex_index = 0;
                    for (size_t j = 1; j < sitas.size(); j++) {
                        if (sitas[j] > max_sita) {
                            max_sita_vertex_index = j;
                            max_sita = sitas[j];
                        }
                    }
                    if (max_sita_vertex_index == 0) {
                        volume->mm_segmentation_facets.set_triangle_from_string(i, result0 + result1 + result2 + (result1 + result2 + "5" )+ "3"); //"1C0C2C0C1C13"
                    } else if (max_sita_vertex_index == 1) {
                        volume->mm_segmentation_facets.set_triangle_from_string(i, result0 + result1 + result2 + (result0 + result2 + "9") + "3");
                    } else{// if (max_sita_vertex_index == 2)
                        volume->mm_segmentation_facets.set_triangle_from_string(i, result0 + result1 + result2 + (result1 + result0 + "1") + "3");
                    }
                    break;
                }
                case _2_SAME_1_DIFF_COLOR: {
                    std::string result0, result1, result2;
                    get_real_filament_id(filament_id0, result0);
                    get_real_filament_id(filament_id1, result1);
                    get_real_filament_id(filament_id2, result2);
                    if (iso_index == 0) {
                        volume->mm_segmentation_facets.set_triangle_from_string(i, result0 + result1 + result1 + "2");
                    } else if (iso_index == 1) {
                        volume->mm_segmentation_facets.set_triangle_from_string(i, result1 + result0 + result0 + "6");
                    } else if (iso_index == 2) {
                        volume->mm_segmentation_facets.set_triangle_from_string(i, result2 + result0 + result0 + "A");
                    }
                    break;
                }
                default: break;
                }
            }
            return true;
        }
    }
    return false;
}

bool obj_import_face_color_deal(const std::vector<unsigned char> &face_filament_ids, const unsigned char &first_extruder_id, Model *model)
{
    if (face_filament_ids.size() == 0) { return false; }
    // 2.generate mm_segmentation_facets
    if (model->objects.size() == 1) {
        auto obj = model->objects[0];
        obj->config.set("extruder", int(first_extruder_id));
        if (obj->volumes.size() == 1) {
            auto volume        = obj->volumes[0];
            volume->config.set("extruder", int(first_extruder_id));
            auto face_count    = volume->mesh().its.indices.size();
            volume->mm_segmentation_facets.reserve(face_count);
            if (volume->mesh().its.indices.size() != face_filament_ids.size()) { return false; }
            for (size_t i = 0; i < volume->mesh().its.indices.size(); i++) {
                auto face         = volume->mesh().its.indices[i];
                auto filament_id = face_filament_ids[i];
                if (filament_id <= 1) { continue; }
                std::string result;
                get_real_filament_id(filament_id, result);
                volume->mm_segmentation_facets.set_triangle_from_string(i, result);
            }
            return true;
        }
    }
    return false;
}

bool obj_import_image_map_deal(const std::vector<unsigned char> &filament_ids,
                                      const ObjImageMapSamplePlan &sample_plan,
                                      const unsigned char &first_extruder_id,
                                      Model *model)
{
    if (model == nullptr || model->objects.size() != 1 || model->objects.front()->volumes.size() != 1 || filament_ids.empty())
        return false;

    ModelObject *object = model->objects.front();
    ModelVolume *volume = object->volumes.front();
    const size_t triangle_count = volume->mesh().its.indices.size();
    if (sample_plan.triangle_subdivision_depths.size() != triangle_count)
        return false;

    size_t expected_filament_ids = 0;
    for (const int8_t depth : sample_plan.triangle_subdivision_depths) {
        if (depth < 0)
            continue;
        const size_t leaf_count = obj_image_map_leaf_count(unsigned(depth));
        if (leaf_count == 0 || expected_filament_ids > std::numeric_limits<size_t>::max() - leaf_count)
            return false;
        expected_filament_ids += leaf_count;
    }
    if (expected_filament_ids != filament_ids.size())
        return false;

    object->config.set("extruder", int(first_extruder_id));
    volume->config.set("extruder", int(first_extruder_id));
    volume->mm_segmentation_facets.reserve(triangle_count);

    size_t offset = 0;
    for (size_t triangle_idx = 0; triangle_idx < triangle_count; ++triangle_idx) {
        const int8_t depth = sample_plan.triangle_subdivision_depths[triangle_idx];
        if (depth < 0)
            continue;
        const std::string encoded = encode_obj_image_map_triangle_filaments(
            filament_ids, offset, unsigned(depth), first_extruder_id);
        if (!encoded.empty())
            volume->mm_segmentation_facets.set_triangle_from_string(int(triangle_idx), encoded);
        offset += obj_image_map_leaf_count(unsigned(depth));
    }
    return offset == filament_ids.size();
}

bool obj_import_persistent_image_map_deal(ImageMap::VolumeData data,
                                                 const unsigned char &first_extruder_id,
                                                 Model *model)
{
    if (model == nullptr || model->objects.size() != 1 || model->objects.front()->volumes.size() != 1)
        return false;
    ModelObject *object = model->objects.front();
    ModelVolume *volume = object->volumes.front();
    object->config.set("extruder", int(first_extruder_id));
    volume->config.set("extruder", int(first_extruder_id));
    return volume->set_image_map_data(std::move(data));
}

bool colorin_apply_obj_color_import(Model&                       model,
                                    ObjInfo&                     obj_info,
                                    const ObjImportColorFn&      objFn,
                                    const ObjImageMapProgressFn& image_map_progress_fn,
                                    const bool&                  obj_image_map_cancelled)
{
    bool result = true;
        if (result && objFn && !model.objects.empty() && !model.objects.front()->volumes.empty()) {
            unsigned char       first_extruder_id             = 1;
            const TriangleMesh& mesh                          = model.objects.front()->volumes.front()->mesh();
            const bool          vertex_colors_available       = !obj_info.vertex_colors.empty();
            const bool          face_colors_available         = !obj_info.face_colors.empty();
            const bool          texture_coordinates_available = obj_info.triangle_uvs.size() == mesh.its.indices.size() &&
                                                       obj_info.triangle_uvs_valid.size() == mesh.its.indices.size() &&
                                                       std::any_of(obj_info.triangle_uvs_valid.begin(), obj_info.triangle_uvs_valid.end(),
                                                                   [](uint8_t valid) { return valid != 0; });

            ObjImageMapSamplePlan detected_image_map_plan;
            std::string           detected_image_map_warning;
            const bool            detected_texture_available = obj_info.has_uv_png && texture_coordinates_available &&
                                                    build_obj_image_map_sample_plan(mesh, obj_info, 0.4f, 200000, detected_image_map_plan,
                                                                                    &detected_image_map_warning, image_map_progress_fn);
            if (obj_image_map_cancelled)
                return false;
            if (!detected_image_map_warning.empty())
                BOOST_LOG_TRIVIAL(warning) << obj_info.source_format << " image-map import: " << detected_image_map_warning;

            ObjColorImportSource  current_source = ObjColorImportSource::VertexColors;
            ObjColorImportMode    current_mode   = ObjColorImportMode::Colors;
            ObjImageMapSamplePlan current_image_map_plan;
            std::string           current_texture_file;
            bool                  source_available = false;
            if (detected_texture_available) {
                current_source         = ObjColorImportSource::ImageTexture;
                current_mode           = ObjColorImportMode::ImageMap;
                current_image_map_plan = detected_image_map_plan;
                source_available       = true;
            } else if (vertex_colors_available) {
                current_source   = ObjColorImportSource::VertexColors;
                source_available = true;
            } else if (face_colors_available) {
                current_source   = ObjColorImportSource::FaceColors;
                source_available = true;
            }

            std::string pending_warning;
            while (source_available) {
                std::vector<RGBA>* input_colors    = nullptr;
                bool               is_single_color = false;
                switch (current_source) {
                case ObjColorImportSource::ImageTexture: input_colors = &current_image_map_plan.colors; break;
                case ObjColorImportSource::VertexColors: input_colors = &obj_info.vertex_colors; break;
                case ObjColorImportSource::FaceColors:
                    input_colors    = &obj_info.face_colors;
                    is_single_color = obj_info.is_single_mtl;
                    break;
                }
                if (input_colors == nullptr || input_colors->empty())
                    break;

                ObjColorImportContext import_context;
                import_context.source                        = current_source;
                import_context.mode                          = current_mode;
                import_context.vertex_colors_available       = vertex_colors_available;
                import_context.face_colors_available         = face_colors_available;
                import_context.texture_coordinates_available = texture_coordinates_available;
                import_context.detected_texture_available    = detected_texture_available;
                import_context.requested_source              = current_source;
                import_context.requested_mode                = current_mode;
                import_context.warning_message               = std::move(pending_warning);
                import_context.image_map_progress_fn          = image_map_progress_fn;

                std::vector<unsigned char> filament_ids;
                objFn(*input_colors, is_single_color, filament_ids, first_extruder_id, import_context);
                if (obj_image_map_cancelled)
                    return false;

                if (import_context.source_change_requested) {
                    ObjColorImportSource next_source = import_context.requested_source;
                    ObjColorImportMode   next_mode   = import_context.requested_mode;
                    bool                 switched    = false;
                    if (next_source == ObjColorImportSource::ImageTexture) {
                        ObjImageMapSamplePlan next_plan;
                        if (!import_context.requested_texture_file.empty()) {
                            switched = build_obj_image_map_sample_plan_with_texture(mesh, obj_info, import_context.requested_texture_file,
                                                                                    0.4f, 200000, next_plan, &pending_warning,
                                                                                    image_map_progress_fn);
                            if (obj_image_map_cancelled)
                                return false;
                            if (switched)
                                current_texture_file = import_context.requested_texture_file;
                        } else if (detected_texture_available) {
                            next_plan = detected_image_map_plan;
                            switched  = true;
                            current_texture_file.clear();
                        } else {
                            pending_warning = "No usable texture was detected. Select a PNG or JPEG image for the mesh UV coordinates.";
                        }
                        if (switched)
                            current_image_map_plan = std::move(next_plan);
                    } else if (next_source == ObjColorImportSource::VertexColors) {
                        switched = vertex_colors_available;
                        if (switched)
                            current_texture_file.clear();
                        if (!switched)
                            pending_warning = "This mesh does not contain vertex colors.";
                    } else if (next_source == ObjColorImportSource::FaceColors) {
                        switched = face_colors_available;
                        if (switched)
                            current_texture_file.clear();
                        if (!switched)
                            pending_warning = "This mesh does not contain material colors.";
                    }

                    if (switched) {
                        current_source = next_source;
                        current_mode   = next_mode;
                        pending_warning.clear();
                    }
                    continue;
                }

                if (filament_ids.empty())
                    break;
                if (current_mode == ObjColorImportMode::ImageMap) {
                    ImageMap::Zone zone;
                    zone.stable_id = obj_info.source_format == "OBJ" ? "obj-image-map-zone" : "mesh-image-map-zone";
                    zone.display_name                = obj_info.source_format + " image map";
                    switch (import_context.image_map_render_mode) {
                    case ObjImageMapRenderMode::PerimeterModulationV2:
                        zone.render_mode = ImageMap::RenderMode::PerimeterModulationV2;
                        break;
                    case ObjImageMapRenderMode::AdaptiveLocalizedCycles:
                        zone.render_mode = ImageMap::RenderMode::AdaptiveLocalizedCycles;
                        break;
                    case ObjImageMapRenderMode::NormalMix:
                    default:
                        zone.render_mode = ImageMap::RenderMode::NormalMix;
                        break;
                    }
                    zone.adaptive_modulation_mode =
                        import_context.image_map_adaptive_modulation_mode == ObjAdaptiveModulationMode::LocalZHeight ?
                            ImageMap::AdaptiveModulationMode::LocalZHeight :
                            ImageMap::AdaptiveModulationMode::Perimeter;
                    switch (import_context.image_map_color_mix_model) {
                    case ObjImageMapColorMixModel::FullSpectrumKmKs:
                        zone.color_mix_model = ImageMap::ColorMixModel::FullSpectrumKmKs;
                        break;
                    case ObjImageMapColorMixModel::FilamentMixer:
                        zone.color_mix_model = ImageMap::ColorMixModel::FilamentMixer;
                        break;
                    default:
                        zone.color_mix_model = ImageMap::ColorMixModel::FullSpectrumKmKs;
                        break;
                    }
                    zone.synchronize_whole_object_cadence =
                        zone.render_mode == ImageMap::RenderMode::PerimeterModulationV2 &&
                        import_context.image_map_synchronize_whole_object_cadence;
                    zone.minimum_component_percent  = import_context.image_map_minimum_component_percent;
                    zone.modulation_sample_spacing_mm = import_context.image_map_modulation_sample_spacing_mm;
                    zone.disable_broad_path_smoothing = import_context.image_map_disable_broad_path_smoothing;
                    zone.gaussian_smoothing_strength  = import_context.image_map_gaussian_smoothing_strength;
                    zone.first_path_smoothing_strength = import_context.image_map_first_path_smoothing_strength;
                    zone.second_path_smoothing_strength = import_context.image_map_second_path_smoothing_strength;
                    zone.tone_gamma                   = import_context.image_map_tone_gamma;
                    zone.overhang_contrast_percent    = import_context.image_map_overhang_contrast_percent;
                    zone.image_exposure_ev            = import_context.image_map_exposure_ev;
                    zone.image_contrast_percent       = import_context.image_map_contrast_percent;
                    zone.image_saturation_percent     = import_context.image_map_saturation_percent;
                    zone.image_edge_boost_percent     = import_context.image_map_edge_boost_percent;
                    const size_t palette_size = std::min({import_context.image_map_palette_colors.size(),
                                                          import_context.image_map_palette_filament_ids.size(),
                                                          import_context.image_map_palette_mixed_stable_ids.size()});
                    for (size_t palette_idx = 0; palette_idx < palette_size; ++palette_idx) {
                        zone.palette.push_back({import_context.image_map_palette_colors[palette_idx],
                                                import_context.image_map_palette_mixed_stable_ids[palette_idx],
                                                import_context.image_map_palette_filament_ids[palette_idx]});
                    }

                    ImageMap::VolumeData image_map_data;
                    std::string image_map_warning;
                    result = !zone.palette.empty() &&
                             build_obj_image_map_volume_data(mesh,
                                                             obj_info,
                                                             current_source,
                                                             current_texture_file,
                                                             std::move(zone),
                                                             image_map_data,
                                                             &image_map_warning,
                                                             image_map_progress_fn) &&
                             obj_import_persistent_image_map_deal(std::move(image_map_data), first_extruder_id, &model);
                    if (obj_image_map_cancelled)
                        return false;
                    if (!image_map_warning.empty())
                        BOOST_LOG_TRIVIAL(warning) << obj_info.source_format << " image-map source: " << image_map_warning;
                } else if (current_source == ObjColorImportSource::ImageTexture)
                    result = obj_import_image_map_deal(filament_ids, current_image_map_plan, first_extruder_id, &model);
                else if (current_source == ObjColorImportSource::VertexColors)
                    result = obj_import_vertex_color_deal(filament_ids, first_extruder_id, &model);
                else
                    result = obj_import_face_color_deal(filament_ids, first_extruder_id, &model);
                break;
            }
        }
    return result;
}

} // namespace Slic3r
