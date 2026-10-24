// Prusa Colorin: ver ColorinGuiCompat.hpp.
#include "ColorinGuiCompat.hpp"

#include "GUI_App.hpp"
#include "Plater.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/ImageMap/VolumeData.hpp"

#include <algorithm>
#include "Widgets/Label.hpp"

#include <map>
#include <memory>
#include <string>
#include <wx/bitmap.h>
#include <wx/dcmemory.h>

namespace Slic3r { namespace GUI {

void colorin_on_filaments_change(size_t /*num_filaments*/)
{
    PresetBundle *preset_bundle = wxGetApp().preset_bundle;
    if (preset_bundle == nullptr)
        return;
    preset_bundle->sync_mixed_filament_definitions_to_project_config();
    if (Plater *plater = wxGetApp().plater())
        plater->on_config_change(preset_bundle->full_config());
}

} } // namespace Slic3r::GUI

// Copia de OrcaSlicer wxExtensions.cpp get_extruder_color_icon.
// Diferencias: cache propio (no BitmapCache de Orca) y, si el color es transparente,
// fondo blanco con borde gris en vez del patron "transparent".svg de Orca.
wxBitmap *get_extruder_color_icon(std::string color, std::string label, int icon_width, int icon_height)
{
    static std::map<std::string, std::unique_ptr<wxBitmap>> bmp_cache;

    std::string bitmap_key = color + "-h" + std::to_string(icon_height) + "-w" + std::to_string(icon_width) + "-i" + label;

    auto it = bmp_cache.find(bitmap_key);
    if (it != bmp_cache.end())
        return it->second.get();

    wxColor clr(color);
    auto    bitmap = std::make_unique<wxBitmap>(icon_width, icon_height);
    {
        wxMemoryDC dc;
        dc.SetFont(::Label::Body_12);
        dc.SelectObject(*bitmap);
        if (clr.Alpha() == 0) {
            dc.SetBackground(*wxWHITE_BRUSH);
            dc.Clear();
            dc.SetPen(*wxGREY_PEN);
            dc.SetBrush(*wxTRANSPARENT_BRUSH);
            dc.DrawRectangle(0, 0, icon_width, icon_height);
            clr.SetRGB(0xffffff); // for text color
        } else {
            dc.SetBackground(wxBrush(clr));
            dc.Clear();
            dc.SetBrush(wxBrush(clr));
        }
        if (clr.Red() > 224 && clr.Blue() > 224 && clr.Green() > 224) {
            dc.SetPen(*wxGREY_PEN);
            dc.DrawRectangle(0, 0, icon_width, icon_height);
        }
        auto size = dc.GetTextExtent(wxString(label));
        dc.SetTextForeground(clr.GetLuminance() < 0.51 ? *wxWHITE : *wxBLACK);
        dc.DrawText(label, (icon_width - size.x) / 2, (icon_height - size.y) / 2);
        dc.SelectObject(wxNullBitmap);
    }
    wxBitmap *out = bitmap.get();
    bmp_cache.emplace(bitmap_key, std::move(bitmap));
    return out;
}

namespace Slic3r {
// Equivale a OrcaSlicer ModelVolume::remap_extruder_ids + FacetsAnnotation::remap_enforcer_block_types:
// se cargan los triangulos pintados, se cambia cada color segun state_map y se guardan.
void colorin_remap_extruder_ids(ModelVolume &mv, size_t /*extruder_count*/, const EnforcerBlockerStateMap &state_map)
{
    if (mv.mm_segmentation_facets.empty())
        return;
    TriangleSelector selector(mv.mesh());
    selector.deserialize(mv.mm_segmentation_facets.get_data(), false);
    selector.remap_triangle_state(state_map);
    mv.mm_segmentation_facets.set(selector);
}
} // namespace Slic3r

namespace Slic3r {
std::vector<std::string> colorin_filament_presets(const PresetBundle &preset_bundle)
{
    std::vector<std::string> out;
    out.reserve(preset_bundle.extruders_filaments.size());
    for (const ExtruderFilaments &extr_filaments : preset_bundle.extruders_filaments)
        out.emplace_back(extr_filaments.get_selected_preset_name());
    return out;
}

std::string colorin_filament_type(const Preset &preset)
{
    const auto *filament_type = dynamic_cast<const ConfigOptionStrings*>(preset.config.option("filament_type"));
    if (!filament_type)
        return "";
    return filament_type->get_at(0);
}

// Copia de OrcaSlicer Model.cpp image_map_fallback_filament_ids.
static std::vector<unsigned int> colorin_image_map_fallback_filament_ids(const ModelVolume &volume)
{
    const std::shared_ptr<const ImageMap::VolumeData> data = volume.image_map_data();
    if (!data)
        return {};

    std::vector<bool> referenced_zones(data->zones.size(), false);
    for (const ImageMap::TriangleBinding &binding : data->triangle_bindings) {
        if (binding.zone_index < referenced_zones.size())
            referenced_zones[binding.zone_index] = true;
    }

    std::vector<unsigned int> filament_ids;
    for (size_t zone_idx = 0; zone_idx < data->zones.size(); ++zone_idx) {
        const ImageMap::Zone &zone = data->zones[zone_idx];
        if (!referenced_zones[zone_idx] || !zone.enabled)
            continue;
        for (const ImageMap::PaletteEntry &entry : zone.palette) {
            if (entry.fallback_filament_id > 0 &&
                std::find(filament_ids.begin(), filament_ids.end(), entry.fallback_filament_id) == filament_ids.end())
                filament_ids.push_back(entry.fallback_filament_id);
        }
    }
    return filament_ids;
}

// Equivale a OrcaSlicer ModelVolume::get_extruders: colores pintados usados + filamento del volumen
// + filamentos de respaldo del mapa de imagen.
std::vector<int> colorin_volume_extruders(const ModelVolume &mv)
{
    if (mv.type() == ModelVolumeType::INVALID || mv.type() == ModelVolumeType::NEGATIVE_VOLUME ||
        mv.type() == ModelVolumeType::SUPPORT_BLOCKER || mv.type() == ModelVolumeType::SUPPORT_ENFORCER)
        return std::vector<int>();

    std::vector<int> volume_extruders;
    const std::vector<bool> &used_states = mv.mm_segmentation_facets.get_data().used_states;
    for (size_t idx = 1; idx < used_states.size(); ++idx)
        if (used_states[idx])
            volume_extruders.push_back(int(idx));

    const int volume_extruder_id = mv.extruder_id();
    if (volume_extruder_id > 0)
        volume_extruders.push_back(volume_extruder_id);
    else if (volume_extruder_id == 0)
        volume_extruders.push_back(volume_extruder_id + 1);

    for (unsigned int filament_id : colorin_image_map_fallback_filament_ids(mv)) {
        if (std::find(volume_extruders.begin(), volume_extruders.end(), int(filament_id)) == volume_extruders.end())
            volume_extruders.push_back(int(filament_id));
    }
    return volume_extruders;
}
} // namespace Slic3r
