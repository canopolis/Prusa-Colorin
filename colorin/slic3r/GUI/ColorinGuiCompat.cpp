// Prusa Colorin: ver ColorinGuiCompat.hpp.
#include "ColorinGuiCompat.hpp"

#include "GUI_App.hpp"
#include "Plater.hpp"
#include "libslic3r/PresetBundle.hpp"
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
