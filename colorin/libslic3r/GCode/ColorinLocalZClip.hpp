// Prusa Colorin - Local-Z G-code helpers (LZ-3a).
// Port of Snapmaker Orca FullSpectrum v0.9.18 src/libslic3r/GCode.cpp lines 3757-4060
// (clip_extrusion_collection_for_local_z and helpers), adapted to the PrusaSlicer 2.9.6 API:
// ExtrusionPath attributes are immutable -> rebuilt through ExtrusionAttributes copies,
// erExternalPerimeter -> ExtrusionRole::is_external_perimeter(), erPerimeter -> ExtrusionRole::Perimeter.
#pragma once

#include <memory>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionRole.hpp"

namespace Slic3r {
namespace ColorinLocalZ {

enum class LocalZClipRoleScope
{
    All,
    ExternalPerimeters,
    AllExceptInnerPerimeters
};

struct LocalZPathHeightStats
{
    size_t count { 0 };
    double min   { 0. };
    double max   { 0. };
};

void apply_local_z_flow_height_override(ExtrusionPath &path, double flow_height_override);
void append_clipped_path(const ExtrusionPath &src_path, const ExPolygons *include_masks, const ExPolygons *exclude_masks,
                         double flow_height_override, ExtrusionEntityCollection &dst);
bool local_z_clipping_keeps_path_complete(const ExtrusionPath &src_path, const ExPolygons *include_masks, const ExPolygons *exclude_masks);
void append_clipped_loop(const ExtrusionLoop &src_loop, const ExPolygons *include_masks, const ExPolygons *exclude_masks,
                         double flow_height_override, ExtrusionEntityCollection &dst);
ExPolygons local_z_compensate_masks(const ExPolygons &src_masks, float delta_scaled, bool fallback_to_source);
LocalZPathHeightStats collect_local_z_path_height_stats(const ExtrusionEntityCollection &source);
LocalZPathHeightStats collect_local_z_path_height_stats(const ExtrusionEntitiesPtr &source);
Polylines collect_local_z_role_polylines(const ExtrusionEntityCollection &source, ExtrusionRole role);
Polylines collect_local_z_external_perimeter_polylines(const ExtrusionEntityCollection &source);
double local_z_polyline_length(const Polylines &polylines);
bool local_z_role_matches(ExtrusionRole role, LocalZClipRoleScope scope);
std::unique_ptr<ExtrusionEntityCollection> clip_extrusion_collection_for_local_z(
    const ExtrusionEntityCollection &source, const ExPolygons *include_masks, const ExPolygons *exclude_masks,
    double flow_height_override, LocalZClipRoleScope role_scope, const ExPolygons *non_external_exclude_masks = nullptr);

} // namespace ColorinLocalZ
} // namespace Slic3r
