// Prusa Colorin - Local-Z per-layer G-code context (LZ-3b).
// Port of Snapmaker Orca FullSpectrum v0.9.18 src/libslic3r/GCode.cpp lines 4875-5079
// (LocalZPassBucket / LocalZLayerContext construction inside GCode::process_layer).
// Orca kept per-bucket ObjectByExtruder maps; PrusaSlicer 2.9.6 builds its order in
// GCode/ExtrusionOrder.cpp, so the extrusion lists are attached in LZ-3c instead.
#pragma once

#include <cstddef>
#include <vector>
#include "libslic3r/ExPolygon.hpp"

namespace Slic3r {

class PrintObject;
struct SubLayerPlan;

namespace ColorinLocalZ {

// Same constants as Orca 0.9.18.
constexpr double LOCAL_Z_PERIMETER_MASK_EXPAND_MM      = 0.10;
constexpr double LOCAL_Z_BASE_MASK_EXPAND_MM           = 0.04;
constexpr double LOCAL_Z_ADAPTIVE_INNER_OWNERSHIP_MM   = 0.75;

struct LocalZPassBucket
{
    const SubLayerPlan     *plan { nullptr };
    std::vector<ExPolygons> compensated_masks_by_extruder;
};

struct LocalZLayerContext
{
    bool                          enabled { false };
    bool                          independent_layer_height { false };
    ExPolygons                    raw_mixed_masks_union;
    ExPolygons                    mixed_masks_union;
    ExPolygons                    non_adaptive_mixed_masks_union;
    ExPolygons                    mixed_masks_union_for_base_exclude;
    ExPolygons                    non_adaptive_masks_for_base_exclude;
    ExPolygons                    adaptive_inner_ownership_masks;
    bool                          has_adaptive_perimeter_pass { false };
    size_t                        local_clipped_collections { 0 };
    size_t                        base_clipped_collections { 0 };
    size_t                        base_clip_leak_warnings { 0 };
    std::vector<LocalZPassBucket> pass_buckets;
};

// Builds the Local-Z context of one object layer. Returns a disabled context when the
// layer has no Local-Z interval. requested_out is set when the layer asked for Local-Z
// sub-passes (Orca local_z_phase_b_requested_for_layer). rejected_logs limits warnings
// to 50 per G-code export, as in Orca.
LocalZLayerContext build_local_z_layer_context(const PrintObject &print_object, size_t layer_id, double print_z,
                                               bool &requested_out, size_t &rejected_logs);

// Orca else-branch (wiping overrides active): only reports whether Local-Z was requested.
bool local_z_requested_for_layer(const PrintObject &print_object, size_t layer_id);

} // namespace ColorinLocalZ
} // namespace Slic3r
