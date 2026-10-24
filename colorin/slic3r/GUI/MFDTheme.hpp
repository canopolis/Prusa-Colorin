#ifndef slic3r_GUI_MFDTheme_hpp_
#define slic3r_GUI_MFDTheme_hpp_

#include "slic3r/GUI/ColorinGuiCompat.hpp" // Prusa Colorin
#include <wx/colour.h>
#include <wx/window.h>

#include "Widgets/StateColor.hpp"

namespace Slic3r::GUI::MFDTheme {

inline wxColour dialog_background() { return colorin_dark_mode_color_for(*wxWHITE); }
inline wxColour content_background() { return colorin_dark_mode_color_for(wxColour("#F5F5F5")); }
inline wxColour card_background() { return colorin_dark_mode_color_for(*wxWHITE); }
inline wxColour card_border() { return colorin_dark_mode_color_for(wxColour("#EEEEEE")); }
inline wxColour divider() { return colorin_dark_mode_color_for(wxColour("#EBEBEB")); }
inline wxColour input_background() { return colorin_dark_mode_color_for(*wxWHITE); }
inline wxColour input_border() { return colorin_dark_mode_color_for(wxColour("#D1D5DC")); }
inline wxColour primary_text() { return colorin_dark_mode_color_for(wxColour("#242424")); }
inline wxColour secondary_text() { return colorin_dark_mode_color_for(wxColour("#4A4A4A")); }
inline wxColour muted_text() { return colorin_dark_mode_color_for(wxColour("#8F8F8F")); }
inline wxColour error_text() { return colorin_dark_mode_color_for(wxColour("#D32F2F")); }

inline void apply_window(wxWindow* window, const wxColour& bg = card_background())
{
    if (window)
        window->SetBackgroundColour(bg);
}

inline void apply_text(wxWindow* window, const wxColour& fg = primary_text(), const wxColour& bg = card_background())
{
    if (!window)
        return;
    window->SetForegroundColour(fg);
    window->SetBackgroundColour(bg);
}

inline void apply_input(wxWindow* window)
{
    if (!window)
        return;
    window->SetForegroundColour(primary_text());
    window->SetBackgroundColour(input_background());
}

} // namespace Slic3r::GUI::MFDTheme

#endif // slic3r_GUI_MFDTheme_hpp_
