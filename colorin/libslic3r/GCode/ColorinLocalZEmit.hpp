// Prusa Colorin - Local-Z G-code emission support (LZ-3c).
// Port of Snapmaker Orca FullSpectrum v0.9.18 src/libslic3r/GCode.cpp lines 5234-5460
// (base / sub-pass clipping of perimeters and infill per island).
// Orca rebuilt its ObjectByExtruder lists with clipped collections. PrusaSlicer 2.9.6 reads the
// extrusions straight from LayerRegion in GCode/ExtrusionOrder.cpp, so the clipped collections are
// swapped in place of the originals only while GCode::ExtrusionOrder::get_extrusions() runs and the
// original pointers are always restored afterwards. The clipped collections stay alive until the
// layer G-code is finished, because the sorted extrusions keep pointers to them.
#pragma once

#include <memory>
#include <vector>
#include "ColorinLocalZContext.hpp"

namespace Slic3r {

class Layer;
class Print;
class ExtrusionEntity;
class ExtrusionEntityCollection;

namespace ColorinLocalZ {

class LocalZLayerSwap
{
public:
    LocalZLayerSwap() = default;
    LocalZLayerSwap(const LocalZLayerSwap &) = delete;
    LocalZLayerSwap &operator=(const LocalZLayerSwap &) = delete;
    ~LocalZLayerSwap() { this->restore(); }

    // Puts back the original extrusion pointers. The clipped collections are kept alive.
    void restore();
    // Replaces slot idx of parent with replacement (owned by this object). nullptr -> empty collection.
    void replace(ExtrusionEntityCollection &parent, size_t idx, std::unique_ptr<ExtrusionEntityCollection> replacement);

private:
    struct Slot { ExtrusionEntityCollection *parent; size_t idx; ExtrusionEntity *original; };
    std::vector<Slot>                                       m_slots;
    std::vector<std::unique_ptr<ExtrusionEntityCollection>> m_owned;
};

// Orca: print.full_print_config().opt_bool("dithering_local_z_infill").
bool local_z_infill_enabled(const Print &print);

// Base pass (Orca 5370-5421): every island loses the parts that Local-Z sub-passes print.
void swap_in_base(LocalZLayerSwap &swap, const Layer &layer, LocalZLayerContext &ctx, bool infill_enabled, double print_z);

// One sub-pass for one extruder (Orca 5316-5368): only the parts inside that extruder's masks,
// with the pass flow height. Returns false when nothing of this layer falls inside the masks.
bool swap_in_pass(LocalZLayerSwap &swap, const Layer &layer, LocalZLayerContext &ctx, const LocalZPassBucket &bucket,
                  size_t extruder_id, bool infill_enabled, double print_z);

} // namespace ColorinLocalZ
} // namespace Slic3r
