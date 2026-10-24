// Prusa Colorin: ver ColorinGuiCompat.hpp.
#include "ColorinGuiCompat.hpp"

#include "GUI_App.hpp"
#include "Plater.hpp"
#include "libslic3r/PresetBundle.hpp"

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
