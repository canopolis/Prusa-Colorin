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

#include <array>
#include <cmath>
#include <string>

class wxBitmap;

namespace Slic3r {
// OrcaSlicer Color.cpp: color_is_equal (misma tolerancia 0.01).
inline bool color_is_equal(const std::array<float, 4> a, const std::array<float, 4> &b)
{
    for (size_t i = 0; i < 4; i++) {
        if (std::abs(a[i] - b[i]) > 0.01)
            return false;
    }
    return true;
}
} // namespace Slic3r

// OrcaSlicer wxExtensions.cpp: icono de color con numero de filamento.
wxBitmap *get_extruder_color_icon(std::string color, std::string label, int icon_width, int icon_height);

// OrcaSlicer ComboBox::SetItemTooltip: el ComboBox de PrusaSlicer no tiene tooltip por elemento.
template<class C, class S> inline void colorin_set_item_tooltip(C *, int, const S &) {}

#endif // slic3r_ColorinGuiCompat_hpp_
