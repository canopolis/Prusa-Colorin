// Prusa Colorin - compatibility layer between the FullSpectrum engine (OrcaSlicer) and PrusaSlicer.
#ifndef slic3r_ColorinCompat_hpp_
#define slic3r_ColorinCompat_hpp_

#include "Color.hpp"

#include <array>
#include <string>

namespace Slic3r {

// OrcaSlicer defines RGBA in Color.hpp; PrusaSlicer does not. Re-declaring the same alias is legal C++.
using RGBA = std::array<float, 4>;
// OrcaSlicer Color.hpp: const RGBA UNDEFINE_COLOR = {0,0,0,0};
inline const RGBA UNDEFINE_COLOR = {0, 0, 0, 0};

namespace Colorin {

// Highest painted filament state supported by the FullSpectrum facet encoding (OrcaSlicer: ExtruderMax).
constexpr unsigned int PaintStateMax = 255;

// Same encoding as OrcaSlicer FullSpectrum encode_enforcer_blocker_type_as_facet_string().
std::string encode_paint_state_as_facet_string(unsigned int state);

} // namespace Colorin
} // namespace Slic3r

#endif
