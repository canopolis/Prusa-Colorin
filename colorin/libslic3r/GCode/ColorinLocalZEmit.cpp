// Prusa Colorin - Local-Z G-code emission support (LZ-3c). See ColorinLocalZEmit.hpp.
#include "ColorinLocalZEmit.hpp"
#include "ColorinLocalZClip.hpp"

#include <cmath>
#include <boost/log/trivial.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"

namespace Slic3r {
namespace ColorinLocalZ {

void LocalZLayerSwap::replace(ExtrusionEntityCollection &parent, size_t idx, std::unique_ptr<ExtrusionEntityCollection> replacement)
{
    if (idx >= parent.entities.size())
        return;
    if (!replacement)
        replacement = std::make_unique<ExtrusionEntityCollection>();
    m_slots.push_back(Slot{ &parent, idx, parent.entities[idx] });
    parent.entities[idx] = replacement.get();
    m_owned.emplace_back(std::move(replacement));
}

void LocalZLayerSwap::restore()
{
    for (auto it = m_slots.rbegin(); it != m_slots.rend(); ++it)
        it->parent->entities[it->idx] = it->original;
    m_slots.clear();
}

bool local_z_infill_enabled(const Print &print)
{
    const ConfigOptionBool *opt = print.full_print_config().option<ConfigOptionBool>("dithering_local_z_infill");
    return opt != nullptr && opt->value;
}

// LayerRegion only exposes const accessors; the swap is undone before anything else reads the layer.
static ExtrusionEntityCollection &mutable_collection(const ExtrusionEntityCollection &c)
{
    return const_cast<ExtrusionEntityCollection&>(c);
}

void swap_in_base(LocalZLayerSwap &swap, const Layer &layer, LocalZLayerContext &ctx, bool infill_enabled, double print_z)
{
    if (!ctx.enabled || ctx.mixed_masks_union.empty())
        return;
    const ExPolygons *base_exclude_masks =
        ctx.mixed_masks_union_for_base_exclude.empty() ? &ctx.mixed_masks_union : &ctx.mixed_masks_union_for_base_exclude;
    const ExPolygons *base_external_exclude_masks = ctx.has_adaptive_perimeter_pass ? &ctx.mixed_masks_union : base_exclude_masks;
    const ExPolygons &mixed_leak_ref = !ctx.raw_mixed_masks_union.empty() ? ctx.raw_mixed_masks_union : ctx.mixed_masks_union;

    for (const LayerRegion *layerm : layer.regions()) {
        if (layerm == nullptr)
            continue;
        for (int pass = 0; pass < 2; ++pass) {
            const bool is_perimeters = pass == 1;
            if (!is_perimeters && !infill_enabled)
                continue;
            ExtrusionEntityCollection &parent = mutable_collection(is_perimeters ? layerm->perimeters() : layerm->fills());
            for (size_t idx = 0; idx < parent.entities.size(); ++idx) {
                const auto *extrusions = dynamic_cast<const ExtrusionEntityCollection*>(parent.entities[idx]);
                if (extrusions == nullptr || extrusions->entities.empty())
                    continue;
                auto clipped_base = clip_extrusion_collection_for_local_z(
                    *extrusions, nullptr, base_external_exclude_masks, 0., LocalZClipRoleScope::All,
                    is_perimeters && ctx.has_adaptive_perimeter_pass ? &ctx.non_adaptive_masks_for_base_exclude : nullptr);
                if (clipped_base) {
                    const LocalZPathHeightStats base_height_stats = collect_local_z_path_height_stats(*clipped_base);
                    ++ctx.base_clipped_collections;
                    if (ctx.base_clip_leak_warnings < 3 && !mixed_leak_ref.empty()) {
                        Polylines clipped_base_lines = collect_local_z_external_perimeter_polylines(*clipped_base);
                        if (!clipped_base_lines.empty()) {
                            Polylines leaked_segments = intersection_pl(std::move(clipped_base_lines), mixed_leak_ref);
                            if (!leaked_segments.empty()) {
                                ++ctx.base_clip_leak_warnings;
                                BOOST_LOG_TRIVIAL(warning) << "Local-Z base clip leak"
                                                           << " print_z=" << print_z
                                                           << " leaked_segment_count=" << leaked_segments.size()
                                                           << " warn_index=" << ctx.base_clip_leak_warnings;
                            }
                        }
                    }
                    if (base_height_stats.count > 0 && ctx.base_clipped_collections <= 5)
                        BOOST_LOG_TRIVIAL(debug) << "Local-Z base clip heights"
                                                 << " print_z=" << print_z
                                                 << " observed_min=" << base_height_stats.min
                                                 << " observed_max=" << base_height_stats.max
                                                 << " path_count=" << base_height_stats.count;
                }
                // Orca keeps the original when clipping returns nothing; in Orca that path is
                // "continue" (the island is dropped from the base), so an empty slot is used here.
                swap.replace(parent, idx, std::move(clipped_base));
            }
        }
    }
}

bool swap_in_pass(LocalZLayerSwap &swap, const Layer &layer, LocalZLayerContext &ctx, const LocalZPassBucket &bucket,
                  size_t extruder_id, bool infill_enabled, double print_z)
{
    if (bucket.plan == nullptr)
        return false;
    const SubLayerPlan &plan = *bucket.plan;
    if (extruder_id >= plan.painted_masks_by_extruder.size())
        return false;
    const ExPolygons &pass_masks = bucket.compensated_masks_by_extruder.empty() || extruder_id >= bucket.compensated_masks_by_extruder.size()
        ? plan.painted_masks_by_extruder[extruder_id]
        : bucket.compensated_masks_by_extruder[extruder_id];
    if (pass_masks.empty())
        return false;

    bool any = false;
    for (const LayerRegion *layerm : layer.regions()) {
        if (layerm == nullptr)
            continue;
        for (int pass = 0; pass < 2; ++pass) {
            const bool is_perimeters = pass == 1;
            ExtrusionEntityCollection &parent = mutable_collection(is_perimeters ? layerm->perimeters() : layerm->fills());
            for (size_t idx = 0; idx < parent.entities.size(); ++idx) {
                const auto *extrusions = dynamic_cast<const ExtrusionEntityCollection*>(parent.entities[idx]);
                std::unique_ptr<ExtrusionEntityCollection> clipped_local;
                if (extrusions != nullptr && !extrusions->entities.empty() && (is_perimeters || infill_enabled)) {
                    clipped_local = clip_extrusion_collection_for_local_z(
                        *extrusions, &pass_masks, nullptr, plan.flow_height,
                        is_perimeters && plan.external_perimeters_only ? LocalZClipRoleScope::ExternalPerimeters : LocalZClipRoleScope::All);
                    if (clipped_local) {
                        any = true;
                        ++ctx.local_clipped_collections;
                        const LocalZPathHeightStats local_height_stats = collect_local_z_path_height_stats(*clipped_local);
                        if (local_height_stats.count > 0 &&
                            (std::abs(local_height_stats.min - plan.flow_height) > 1e-3 ||
                             std::abs(local_height_stats.max - plan.flow_height) > 1e-3))
                            BOOST_LOG_TRIVIAL(warning) << "Local-Z local pass height mismatch"
                                                       << " print_z=" << print_z
                                                       << " plan_layer_id=" << plan.layer_id
                                                       << " pass_index=" << plan.pass_index
                                                       << " expected_height=" << plan.flow_height
                                                       << " observed_min=" << local_height_stats.min
                                                       << " observed_max=" << local_height_stats.max
                                                       << " path_count=" << local_height_stats.count;
                    }
                }
                // Everything outside the pass masks is hidden while this pass is collected.
                swap.replace(parent, idx, std::move(clipped_local));
            }
        }
    }
    return any;
}

} // namespace ColorinLocalZ
} // namespace Slic3r
