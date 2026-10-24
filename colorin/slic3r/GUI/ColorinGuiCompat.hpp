// Prusa Colorin: equivalencias de interfaz entre Orca FullSpectrum 0.9.18 y PrusaSlicer 2.9.6.
#ifndef slic3r_ColorinGuiCompat_hpp_
#define slic3r_ColorinGuiCompat_hpp_

#include "libslic3r/Color.hpp"

#include <cstddef>
#include <wx/colour.h>

namespace Slic3r {
// OrcaSlicer libslic3r.h: limite de filamentos (255). PrusaSlicer no lo define.
inline constexpr size_t MAXIMUM_FILAMENT_NUMBER = 255;
// OrcaSlicer ColorRGB::ORCA(): color de acento de Orca.
inline ColorRGB colorin_orca_rgb() { return ColorRGB(0.0f, 150.f / 255.0f, 136.0f / 255); }
} // namespace Slic3r

// OrcaSlicer StateColor::darkModeColorFor(): en modo claro devuelve el mismo color.
// Prusa Colorin usa modo claro.
inline wxColour colorin_dark_mode_color_for(const wxColour &color) { return color; }

namespace Slic3r { namespace GUI {
// OrcaSlicer Plater::on_filaments_change(): guarda los filamentos mezclados en el proyecto
// y avisa a PrusaSlicer del cambio de configuracion.
void colorin_on_filaments_change(size_t num_filaments);
} } // namespace Slic3r::GUI

#endif // slic3r_ColorinGuiCompat_hpp_
