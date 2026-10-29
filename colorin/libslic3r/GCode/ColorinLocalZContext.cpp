// Prusa Colorin - Local-Z per-layer G-code context (LZ-3b). See ColorinLocalZContext.hpp.
#include "ColorinLocalZContext.hpp"
#include "ColorinLocalZClip.hpp"

#include <algorithm>
#include <boost/log/trivial.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Print.hpp"

namespace Slic3r {
namespace ColorinLocalZ {

static const LocalZInterval *find_interval(const PrintObject &print_object, size_t layer_id)
{
    const auto &intervals = print_object.local_z_intervals();
    auto it = std::find_if(intervals.begin(), intervals.end(),
                           [layer_id](const LocalZInterval &interval) { return interval.layer_id == layer_id; });
    return it == intervals.end() ? nullptr : &(*it);
}

bool local_z_requested_for_layer(const PrintObject &print_object, size_t layer_id)
{
    const LocalZInterval *it_interval = find_interval(print_object, layer_id);
    return it_interval != nullptr && it_interval->has_mixed_paint &&
           (it_interval->sublayer_count > 1 ||
            (it_interval->independent_layer_height && !it_interval->managed_masks.empty()));
}

LocalZLayerContext build_local_z_layer_context(const PrintObject &print_object, size_t layer_id, double print_z,
                                               bool &requested_out, size_t &rejected_logs)
{
    LocalZLayerContext ctx;
    const auto &intervals = print_object.local_z_intervals();
    const auto &plans     = print_object.local_z_sublayer_plan();
    if (intervals.empty() || plans.empty())
        return ctx;

    const LocalZInterval *it_interval = find_interval(print_object, layer_id);
    if (it_interval == nullptr)
        return ctx;
    const bool independent_layer_height = it_interval->independent_layer_height && !it_interval->managed_masks.empty();
    if (!it_interval->has_mixed_paint ||
        (!independent_layer_height &&
         (it_interval->sublayer_count <= 1 || it_interval->first_sublayer_idx >= plans.size())))
        return ctx;

    const float local_z_perimeter_mask_expand    = float(scale_(LOCAL_Z_PERIMETER_MASK_EXPAND_MM));
    const float local_z_base_mask_expand         = float(scale_(LOCAL_Z_BASE_MASK_EXPAND_MM));
    const float local_z_adaptive_inner_ownership = float(scale_(LOCAL_Z_ADAPTIVE_INNER_OWNERSHIP_MM));

    ctx.enabled                  = true;
    ctx.independent_layer_height = independent_layer_height;
    const size_t first_idx = it_interval->first_sublayer_idx;
    const size_t end_idx   = std::min(plans.size(), first_idx + it_interval->sublayer_count);
    size_t split_plan_count                  = 0;
    size_t split_plan_with_raw_masks         = 0;
    size_t split_plan_with_compensated_masks = 0;
    size_t raw_mask_polygon_count            = 0;
    size_t compensated_mask_polygon_count    = 0;
    ExPolygons raw_mixed_masks_union = independent_layer_height ? it_interval->managed_masks : ExPolygons();
    for (size_t plan_idx = first_idx; plan_idx < end_idx; ++plan_idx) {
        const SubLayerPlan &plan = plans[plan_idx];
        if (!plan.split_interval)
            continue;
        ++split_plan_count;

        LocalZPassBucket bucket;
        bucket.plan = &plan;
        const size_t plan_mask_slots = std::max(plan.painted_masks_by_extruder.size(), plan.fixed_painted_masks_by_extruder.size());
        bucket.compensated_masks_by_extruder.assign(plan_mask_slots, ExPolygons());
        bool pass_has_raw_masks         = false;
        bool pass_has_compensated_masks = false;
        ExPolygons fixed_raw_masks_union;
        for (const ExPolygons &fixed_masks : plan.fixed_painted_masks_by_extruder)
            if (!fixed_masks.empty())
                append(fixed_raw_masks_union, fixed_masks);
        if (!fixed_raw_masks_union.empty() && fixed_raw_masks_union.size() > 1)
            fixed_raw_masks_union = union_ex(fixed_raw_masks_union);
        const ExPolygons fixed_compensated_guard =
            fixed_raw_masks_union.empty() ? ExPolygons() : local_z_compensate_masks(fixed_raw_masks_union, local_z_perimeter_mask_expand, true);

        for (size_t extruder_id = 0; extruder_id < plan_mask_slots; ++extruder_id) {
            const ExPolygons mixed_raw_masks =
                extruder_id < plan.painted_masks_by_extruder.size() ? plan.painted_masks_by_extruder[extruder_id] : ExPolygons();
            const ExPolygons fixed_raw_masks =
                extruder_id < plan.fixed_painted_masks_by_extruder.size() ? plan.fixed_painted_masks_by_extruder[extruder_id] : ExPolygons();
            if (mixed_raw_masks.empty() && fixed_raw_masks.empty())
                continue;
            pass_has_raw_masks = true;
            raw_mask_polygon_count += mixed_raw_masks.size() + fixed_raw_masks.size();
            if (!mixed_raw_masks.empty())
                append(raw_mixed_masks_union, mixed_raw_masks);
            if (!fixed_raw_masks.empty())
                append(raw_mixed_masks_union, fixed_raw_masks);

            ExPolygons compensated;
            if (!mixed_raw_masks.empty()) {
                ExPolygons compensated_mixed = local_z_compensate_masks(mixed_raw_masks, local_z_perimeter_mask_expand, true);
                if (!fixed_compensated_guard.empty())
                    compensated_mixed = diff_ex(compensated_mixed, fixed_compensated_guard);
                if (!compensated_mixed.empty())
                    append(compensated, compensated_mixed);
            }
            if (!fixed_raw_masks.empty())
                append(compensated, fixed_raw_masks);
            if (!compensated.empty() && compensated.size() > 1)
                compensated = union_ex(compensated);
            if (!compensated.empty()) {
                pass_has_compensated_masks = true;
                compensated_mask_polygon_count += compensated.size();
            }
            bucket.compensated_masks_by_extruder[extruder_id] = std::move(compensated);
        }
        if (pass_has_raw_masks)
            ++split_plan_with_raw_masks;
        if (pass_has_compensated_masks) {
            ++split_plan_with_compensated_masks;
            ctx.pass_buckets.emplace_back(std::move(bucket));
            const LocalZPassBucket &appended_bucket = ctx.pass_buckets.back();
            for (const ExPolygons &masks : appended_bucket.compensated_masks_by_extruder) {
                append(ctx.mixed_masks_union, masks);
                if (appended_bucket.plan != nullptr && appended_bucket.plan->external_perimeters_only)
                    ctx.has_adaptive_perimeter_pass = true;
                else
                    append(ctx.non_adaptive_mixed_masks_union, masks);
            }
        }
    }
    if (!raw_mixed_masks_union.empty() && raw_mixed_masks_union.size() > 1)
        raw_mixed_masks_union = union_ex(raw_mixed_masks_union);
    ctx.raw_mixed_masks_union = raw_mixed_masks_union;
    if (!ctx.mixed_masks_union.empty() && ctx.mixed_masks_union.size() > 1) {
        ExPolygons merged_masks = union_ex(ctx.mixed_masks_union);
        if (!merged_masks.empty())
            ctx.mixed_masks_union = std::move(merged_masks);
        else if (!raw_mixed_masks_union.empty())
            ctx.mixed_masks_union = raw_mixed_masks_union;
    }
    if (ctx.mixed_masks_union.empty() && !raw_mixed_masks_union.empty())
        ctx.mixed_masks_union = raw_mixed_masks_union;
    if (!ctx.non_adaptive_mixed_masks_union.empty() && ctx.non_adaptive_mixed_masks_union.size() > 1)
        ctx.non_adaptive_mixed_masks_union = union_ex(ctx.non_adaptive_mixed_masks_union);
    const ExPolygons &base_exclude_source = !ctx.raw_mixed_masks_union.empty() ? ctx.raw_mixed_masks_union : ctx.mixed_masks_union;
    if (!base_exclude_source.empty())
        ctx.mixed_masks_union_for_base_exclude = local_z_compensate_masks(base_exclude_source, local_z_base_mask_expand, true);
    if (!ctx.non_adaptive_mixed_masks_union.empty())
        ctx.non_adaptive_masks_for_base_exclude = local_z_compensate_masks(ctx.non_adaptive_mixed_masks_union, local_z_base_mask_expand, true);
    if (ctx.has_adaptive_perimeter_pass)
        ctx.adaptive_inner_ownership_masks = local_z_compensate_masks(ctx.mixed_masks_union, local_z_adaptive_inner_ownership, true);
    if ((!independent_layer_height && ctx.pass_buckets.empty()) || ctx.mixed_masks_union.empty()) {
        ctx.enabled = false;
        if (rejected_logs < 50) {
            ++rejected_logs;
            BOOST_LOG_TRIVIAL(warning) << "Local-Z context rejected"
                                       << " print_z=" << print_z
                                       << " layer_id=" << layer_id
                                       << " first_idx=" << first_idx
                                       << " end_idx=" << end_idx
                                       << " interval_sublayer_count=" << it_interval->sublayer_count
                                       << " split_plan_count=" << split_plan_count
                                       << " split_plan_with_raw_masks=" << split_plan_with_raw_masks
                                       << " split_plan_with_compensated_masks=" << split_plan_with_compensated_masks
                                       << " raw_mask_polygon_count=" << raw_mask_polygon_count
                                       << " compensated_mask_polygon_count=" << compensated_mask_polygon_count
                                       << " pass_buckets=" << ctx.pass_buckets.size()
                                       << " mixed_mask_count=" << ctx.mixed_masks_union.size();
        }
    }
    if (ctx.enabled && !ctx.pass_buckets.empty())
        requested_out = true;
    BOOST_LOG_TRIVIAL(debug) << "Local-Z context"
                             << " print_z=" << print_z
                             << " layer_id=" << layer_id
                             << " enabled=" << ctx.enabled
                             << " split_pass_count=" << ctx.pass_buckets.size()
                             << " adaptive_external_only=" << ctx.has_adaptive_perimeter_pass
                             << " mixed_mask_count=" << ctx.mixed_masks_union.size()
                             << " base_exclude_mask_count=" << ctx.mixed_masks_union_for_base_exclude.size()
                             << " perimeter_mask_expand_mm=" << LOCAL_Z_PERIMETER_MASK_EXPAND_MM
                             << " base_mask_expand_mm=" << LOCAL_Z_BASE_MASK_EXPAND_MM;
    return ctx;
}

} // namespace ColorinLocalZ
} // namespace Slic3r
