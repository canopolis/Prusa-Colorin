// Prusa Colorin LZ-4b: Local-Z toolchanges for the wipe tower (Orca FullSpectrum 0.9.18).
#ifndef slic3r_ColorinLocalZWipeTower_hpp_
#define slic3r_ColorinLocalZWipeTower_hpp_

#include <vector>
#include "libslic3r/GCode/ExtrusionOrder.hpp"

namespace Slic3r {
class Print;
struct SubLayerPlan;
namespace ColorinLocalZ {

struct LocalZWipeTowerToolchange
{
    unsigned int old_tool { 0 };
    unsigned int new_tool { 0 };
};

// One Local-Z sub-pass printed with one extruder, in the order Orca 0.9.18 prints them (LZ-4c2a).
struct LocalZPassStep
{
    size_t              layer_to_print_idx { 0 };
    const SubLayerPlan *plan { nullptr };
    unsigned int        extruder_id { 0 };
};

// Extra toolchanges needed by the Local-Z sub-passes of the layers printed at one print_z,
// starting from start_extruder (0-based). Same scheduling as Orca 0.9.18.
std::vector<LocalZWipeTowerToolchange> collect_local_z_wipe_tower_toolchanges(
    const Print                                   &print,
    const std::vector<GCode::ObjectLayerToPrint>  &layers,
    int                                            start_extruder,
    std::vector<LocalZPassStep>                   *steps_out = nullptr);

} // namespace ColorinLocalZ
} // namespace Slic3r

#endif
