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

#include "libslic3r/TriangleSelector.hpp"

namespace Slic3r {
class ModelVolume;
// OrcaSlicer TriangleSelector.hpp: EnforcerBlockerType::ExtruderMax = 255 y EnforcerBlockerStateMap.
// En PrusaSlicer el tipo se llama TriangleStateType.
inline constexpr TriangleStateType COLORIN_EXTRUDER_MAX_TYPE = TriangleStateType(255);
using EnforcerBlockerStateMap = std::array<TriangleStateType, 256>;
// OrcaSlicer ModelVolume::remap_extruder_ids (renumera los colores pintados).
void colorin_remap_extruder_ids(ModelVolume &mv, size_t extruder_count, const EnforcerBlockerStateMap &state_map);
} // namespace Slic3r

#include <vector>

namespace Slic3r {
class PresetBundle;
class Preset;
// OrcaSlicer libslic3r.h: MAXIMUM_EXTRUDER_NUMBER = 64.
inline constexpr size_t MAXIMUM_EXTRUDER_NUMBER = 64;
// OrcaSlicer PresetBundle::filament_presets: nombre del filamento elegido en cada posicion.
std::vector<std::string> colorin_filament_presets(const PresetBundle &preset_bundle);
// OrcaSlicer Preset::get_filament_type (PrusaSlicer no tiene filament_is_support: devuelve filament_type).
std::string colorin_filament_type(const Preset &preset);
// OrcaSlicer ModelVolume::get_extruders.
std::vector<int> colorin_volume_extruders(const ModelVolume &mv);
} // namespace Slic3r

#endif // slic3r_ColorinGuiCompat_hpp_
