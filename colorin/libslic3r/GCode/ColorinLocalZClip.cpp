// Prusa Colorin - Local-Z G-code helpers (LZ-3a). See ColorinLocalZClip.hpp.
#include "ColorinLocalZClip.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include "libslic3r/ClipperUtils.hpp"

namespace Slic3r {
namespace ColorinLocalZ {

static inline bool is_ext(const ExtrusionRole &role) { return role.is_external_perimeter(); }

void apply_local_z_flow_height_override(ExtrusionPath &path, const double flow_height_override)
{
    if (flow_height_override <= EPSILON)
        return;
    ExtrusionAttributes attr = path.attributes();
    if (attr.height > EPSILON) {
        const double ratio = flow_height_override / attr.height;
        attr.mm3_per_mm *= ratio;
    }
    attr.height = float(flow_height_override);
    path = ExtrusionPath(std::move(path.polyline), attr);
}

void append_clipped_path(const ExtrusionPath &src_path, const ExPolygons *include_masks, const ExPolygons *exclude_masks,
                         const double flow_height_override, ExtrusionEntityCollection &dst)
{
    Polylines segments{src_path.polyline};
    if (include_masks != nullptr && !include_masks->empty())
        segments = intersection_pl(segments, *include_masks);
    if (exclude_masks != nullptr && !exclude_masks->empty())
        segments = diff_pl(segments, *exclude_masks);

    for (Polyline &segment : segments) {
        if (!segment.is_valid())
            continue;
        ExtrusionPath clipped(std::move(segment), src_path.attributes());
        apply_local_z_flow_height_override(clipped, flow_height_override);
        dst.append(std::move(clipped));
    }
}

bool local_z_clipping_keeps_path_complete(const ExtrusionPath &src_path, const ExPolygons *include_masks, const ExPolygons *exclude_masks)
{
    if (include_masks != nullptr && !include_masks->empty()) {
        const Polylines outside = diff_pl(Polylines{src_path.polyline}, *include_masks);
        if (std::any_of(outside.begin(), outside.end(), [](const Polyline &segment) { return segment.is_valid(); }))
            return false;
    }
    if (exclude_masks != nullptr && !exclude_masks->empty()) {
        const Polylines excluded = intersection_pl(Polylines{src_path.polyline}, *exclude_masks);
        if (std::any_of(excluded.begin(), excluded.end(), [](const Polyline &segment) { return segment.is_valid(); }))
            return false;
    }
    return true;
}

void append_clipped_loop(const ExtrusionLoop &src_loop, const ExPolygons *include_masks, const ExPolygons *exclude_masks,
                         const double flow_height_override, ExtrusionEntityCollection &dst)
{
    if (src_loop.paths.empty())
        return;
    const bool loop_is_complete = std::all_of(src_loop.paths.begin(), src_loop.paths.end(),
        [include_masks, exclude_masks](const ExtrusionPath &path) {
            return local_z_clipping_keeps_path_complete(path, include_masks, exclude_masks);
        });
    if (loop_is_complete) {
        ExtrusionLoop preserved_loop(src_loop);
        for (ExtrusionPath &path : preserved_loop.paths)
            apply_local_z_flow_height_override(path, flow_height_override);
        dst.append(std::move(preserved_loop));
        return;
    }
    // Once a mask cuts a loop, the resulting open path endpoints are physical
    // material boundaries and cannot be moved by the regular seam placer.
    for (const ExtrusionPath &path : src_loop.paths)
        append_clipped_path(path, include_masks, exclude_masks, flow_height_override, dst);
}

ExPolygons local_z_compensate_masks(const ExPolygons &src_masks, const float delta_scaled, const bool fallback_to_source)
{
    if (src_masks.empty() || std::abs(delta_scaled) <= EPSILON)
        return src_masks;
    ExPolygons compensated = offset_ex(src_masks, delta_scaled);
    if (!compensated.empty() && compensated.size() > 1)
        compensated = union_ex(compensated);
    if (compensated.empty() && fallback_to_source)
        return src_masks;
    return compensated;
}

static void add_height(LocalZPathHeightStats &stats, double h, bool &first)
{
    ++stats.count;
    if (first) { stats.min = h; stats.max = h; first = false; }
    else { stats.min = std::min(stats.min, h); stats.max = std::max(stats.max, h); }
}

static void collect_stats(const ExtrusionEntity &entity, LocalZPathHeightStats &stats, bool &first)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity)) {
        add_height(stats, path->height(), first);
    } else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath*>(&entity)) {
        for (const ExtrusionPath &p : multipath->paths) add_height(stats, p.height(), first);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity)) {
        for (const ExtrusionPath &p : loop->paths) add_height(stats, p.height(), first);
    } else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            if (child != nullptr)
                collect_stats(*child, stats, first);
    }
}

LocalZPathHeightStats collect_local_z_path_height_stats(const ExtrusionEntityCollection &source)
{
    LocalZPathHeightStats stats;
    bool first = true;
    ExtrusionEntityCollection flattened = source.flatten(false);
    for (const ExtrusionEntity *entity : flattened.entities)
        if (entity != nullptr)
            collect_stats(*entity, stats, first);
    return stats;
}

LocalZPathHeightStats collect_local_z_path_height_stats(const ExtrusionEntitiesPtr &source)
{
    LocalZPathHeightStats stats;
    bool first = true;
    for (const ExtrusionEntity *entity : source)
        if (entity != nullptr)
            collect_stats(*entity, stats, first);
    return stats;
}

Polylines collect_local_z_role_polylines(const ExtrusionEntityCollection &source, ExtrusionRole role)
{
    Polylines lines;
    ExtrusionEntityCollection flattened = source.flatten(false);
    for (const ExtrusionEntity *entity : flattened.entities) {
        if (const auto *path = dynamic_cast<const ExtrusionPath*>(entity)) {
            if (path->role() == role) lines.emplace_back(path->polyline);
        } else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath*>(entity)) {
            for (const ExtrusionPath &p : multipath->paths) if (p.role() == role) lines.emplace_back(p.polyline);
        } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(entity)) {
            for (const ExtrusionPath &p : loop->paths) if (p.role() == role) lines.emplace_back(p.polyline);
        }
    }
    return lines;
}

Polylines collect_local_z_external_perimeter_polylines(const ExtrusionEntityCollection &source)
{
    return collect_local_z_role_polylines(source, ExtrusionRole::ExternalPerimeter);
}

double local_z_polyline_length(const Polylines &polylines)
{
    return std::accumulate(polylines.begin(), polylines.end(), 0., [](double length, const Polyline &polyline) {
        return length + polyline.length();
    });
}

bool local_z_role_matches(ExtrusionRole role, LocalZClipRoleScope scope)
{
    return scope == LocalZClipRoleScope::All ||
           (scope == LocalZClipRoleScope::ExternalPerimeters && is_ext(role)) ||
           (scope == LocalZClipRoleScope::AllExceptInnerPerimeters && !(role == ExtrusionRole::Perimeter));
}

template<typename Paths>
static void clip_paths_container(const Paths &paths, bool is_loop, const ExtrusionLoop *loop,
                                 const ExPolygons *include_masks, const ExPolygons *exclude_masks,
                                 double flow_height_override, LocalZClipRoleScope role_scope,
                                 const ExPolygons *non_external_exclude_masks, ExtrusionEntityCollection &out)
{
    const bool all_matching  = std::all_of(paths.begin(), paths.end(), [role_scope](const ExtrusionPath &p) { return local_z_role_matches(p.role(), role_scope); });
    const bool none_matching = std::none_of(paths.begin(), paths.end(), [role_scope](const ExtrusionPath &p) { return local_z_role_matches(p.role(), role_scope); });
    if (none_matching)
        return;
    const bool all_external     = std::all_of(paths.begin(), paths.end(), [](const ExtrusionPath &p) { return is_ext(p.role()); });
    const bool all_non_external = std::none_of(paths.begin(), paths.end(), [](const ExtrusionPath &p) { return is_ext(p.role()); });
    if (all_matching && (all_external || all_non_external)) {
        const ExPolygons *path_exclude_masks = non_external_exclude_masks != nullptr && all_non_external ? non_external_exclude_masks : exclude_masks;
        if (is_loop)
            append_clipped_loop(*loop, include_masks, path_exclude_masks, flow_height_override, out);
        else
            for (const ExtrusionPath &path : paths)
                append_clipped_path(path, include_masks, path_exclude_masks, flow_height_override, out);
        return;
    }
    for (const ExtrusionPath &path : paths) {
        if (!local_z_role_matches(path.role(), role_scope))
            continue;
        const ExPolygons *path_exclude_masks = non_external_exclude_masks != nullptr && !is_ext(path.role()) ? non_external_exclude_masks : exclude_masks;
        append_clipped_path(path, include_masks, path_exclude_masks, flow_height_override, out);
    }
}

std::unique_ptr<ExtrusionEntityCollection> clip_extrusion_collection_for_local_z(
    const ExtrusionEntityCollection &source, const ExPolygons *include_masks, const ExPolygons *exclude_masks,
    const double flow_height_override, const LocalZClipRoleScope role_scope, const ExPolygons *non_external_exclude_masks)
{
    if (source.entities.empty())
        return nullptr;
    if ((include_masks == nullptr || include_masks->empty()) &&
        (exclude_masks == nullptr || exclude_masks->empty()) &&
        (non_external_exclude_masks == nullptr || non_external_exclude_masks->empty()) &&
        flow_height_override <= EPSILON && role_scope == LocalZClipRoleScope::All)
        return std::make_unique<ExtrusionEntityCollection>(source);

    auto out = std::make_unique<ExtrusionEntityCollection>();
    out->no_sort = source.no_sort;

    ExtrusionEntityCollection flattened = source.flatten(false);
    for (const ExtrusionEntity *entity : flattened.entities) {
        if (const auto *path = dynamic_cast<const ExtrusionPath*>(entity)) {
            if (local_z_role_matches(path->role(), role_scope)) {
                const ExPolygons *path_exclude_masks = non_external_exclude_masks != nullptr && !is_ext(path->role()) ? non_external_exclude_masks : exclude_masks;
                append_clipped_path(*path, include_masks, path_exclude_masks, flow_height_override, *out);
            }
        } else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath*>(entity)) {
            clip_paths_container(multipath->paths, false, nullptr, include_masks, exclude_masks, flow_height_override, role_scope, non_external_exclude_masks, *out);
        } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(entity)) {
            clip_paths_container(loop->paths, true, loop, include_masks, exclude_masks, flow_height_override, role_scope, non_external_exclude_masks, *out);
        } else {
            // Fallback for unknown entity subclasses: keep behavior unchanged for now.
            if (include_masks == nullptr && exclude_masks == nullptr && flow_height_override <= EPSILON)
                out->append(*entity);
        }
    }
    if (out->entities.empty())
        return nullptr;
    return out;
}

} // namespace ColorinLocalZ
} // namespace Slic3r
