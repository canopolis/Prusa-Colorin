///|/ Copyright (c) Prusa Research 2019 - 2023 Oleksandra Iushchenko @YuSanka, Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966, Filip Sykala @Jony01, Lukáš Hejl @hejllukas, David Kocík @kocikdav, Vojtěch Bubník @bubnikv
///|/ Copyright (c) 2021 Justin Schuh @jschuh
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "GLGizmoMmuSegmentation.hpp"

#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/BitmapCache.hpp"
#include "slic3r/GUI/format.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"
#include "slic3r/GUI/wxExtensions.hpp"
#include "slic3r/GUI/NotificationManager.hpp"
#include "slic3r/GUI/OpenGLManager.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/MixedFilament.hpp" // Prusa Colorin (FullSpectrum)
#include "slic3r/GUI/MixedColorMatchHelpers.hpp" // Prusa Colorin (FullSpectrum)
#include <optional>
#include <cmath>
#include "libslic3r/Model.hpp"
#include "slic3r/Utils/UndoRedo.hpp"


#include <GL/glew.h>

namespace Slic3r::GUI {

// Prusa Colorin (FullSpectrum, Orca 0.9.18): when the project has mixed filaments, the paint
// brush offers the physical filaments followed by the mixed filaments A, B, C... and paints
// state = physical count + mixed index (same numbering as Orca). Without mixed filaments the
// native PrusaSlicer behaviour (physical + virtual extruders) is kept unchanged.
size_t colorin_mixed_visible_cnt()
{
    if (wxGetApp().preset_bundle == nullptr)
        return 0;
    const size_t num_physical = size_t(std::max(wxGetApp().extruders_edited_cnt(), 0));
    const size_t total        = wxGetApp().preset_bundle->mixed_filaments.total_filaments(num_physical);
    return total > num_physical ? total - num_physical : 0;
}

std::vector<ColorRGBA> colorin_paint_colors()
{
    std::vector<ColorRGBA> colors = wxGetApp().plater()->get_extruder_colors_from_plater_config();
    if (colorin_mixed_visible_cnt() == 0)
        return colors;
    const size_t num_physical = size_t(std::max(wxGetApp().extruders_edited_cnt(), 0));
    if (colors.size() > num_physical)
        colors.resize(num_physical); // drop PrusaSlicer virtual extruders, replaced by mixed filaments
    for (const std::string &hex : wxGetApp().preset_bundle->mixed_filaments.display_colors()) {
        ColorRGBA c(0.5f, 0.5f, 0.5f, 1.f);
        decode_color(hex, c);
        colors.emplace_back(c);
    }
    return colors;
}

// Prusa Colorin (Orca 0.9.18 3DScene preview_gradient_colors): sampled gradient colours of every
// mixed filament, indexed like colorin_paint_colors() (index = filament id - 1). Physical
// filaments get an empty vector. Gradients with fewer than 2 samples are drawn as solid colour.
std::vector<std::vector<ColorRGBA>> colorin_paint_gradients(size_t n_colors)
{
    std::vector<std::vector<ColorRGBA>> gradients(n_colors);
    if (colorin_mixed_visible_cnt() == 0 || wxGetApp().preset_bundle == nullptr)
        return gradients;
    const size_t num_physical = size_t(std::max(wxGetApp().extruders_edited_cnt(), 0));
    std::vector<std::string> physical_hex = wxGetApp().plater()->get_extruder_color_strings_from_plater_config();
    if (physical_hex.size() > num_physical)
        physical_hex.resize(num_physical);
    const MixedFilamentDisplayContext ctx = build_mixed_filament_display_context(physical_hex);
    for (size_t idx = num_physical; idx < n_colors; ++idx) {
        const std::optional<MixedFilamentDefinition> definition =
            wxGetApp().preset_bundle->mixed_filaments.mixed_filament_definition_from_id(unsigned(idx + 1), ctx.num_physical);
        if (!definition)
            continue;
        for (const wxColour &color : build_mixed_filament_gradient_preview(*definition, ctx).sampled_colors)
            gradients[idx].emplace_back(float(color.Red()) / 255.f, float(color.Green()) / 255.f, float(color.Blue()) / 255.f, 1.f);
    }
    return gradients;
}


static inline void show_notification_extruders_limit_exceeded()
{
    wxGetApp()
        .plater()
        ->get_notification_manager()
        ->push_notification(NotificationType::MmSegmentationExceededExtrudersLimit, NotificationManager::NotificationLevel::PrintInfoNotificationLevel,
                            GUI::format(_L("Your printer has more extruders than the multi-material painting gizmo supports. For this reason, only the "
                                           "first %1% extruders will be able to be used for painting."), GLGizmoMmuSegmentation::EXTRUDERS_LIMIT));
}

void GLGizmoMmuSegmentation::on_opening()
{
    const int total_extruders = int(colorin_paint_colors().size());
    if (total_extruders > int(GLGizmoMmuSegmentation::EXTRUDERS_LIMIT)) {
        show_notification_extruders_limit_exceeded();
    }
}

void GLGizmoMmuSegmentation::on_shutdown()
{
    m_parent.use_slope(false);
    m_parent.toggle_model_objects_visibility(true);
}

std::string GLGizmoMmuSegmentation::on_get_name() const
{
    return _u8L("Multimaterial painting");
}

bool GLGizmoMmuSegmentation::on_is_selectable() const
{
    return (wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology() == ptFFF
            && wxGetApp().get_mode() != comSimple && wxGetApp().extruders_edited_cnt() > 1);
}

bool GLGizmoMmuSegmentation::on_is_activable() const
{
    return GLGizmoPainterBase::on_is_activable() && wxGetApp().extruders_edited_cnt() > 1;
}


// Prusa Colorin: helpers copied from Orca FullSpectrum 0.9.18 GLGizmoMmuSegmentation.cpp
// --- Gradient rendering helpers (ported from MixedFilamentBadge) ---

static ImU32 interpolate_ImU32(ImU32 c0, ImU32 c1, double t)
{
    t = std::max(0.0, std::min(1.0, t));
    int r = int(((c0 >> IM_COL32_R_SHIFT) & 0xFF) * (1.0 - t) + ((c1 >> IM_COL32_R_SHIFT) & 0xFF) * t);
    int g = int(((c0 >> IM_COL32_G_SHIFT) & 0xFF) * (1.0 - t) + ((c1 >> IM_COL32_G_SHIFT) & 0xFF) * t);
    int b = int(((c0 >> IM_COL32_B_SHIFT) & 0xFF) * (1.0 - t) + ((c1 >> IM_COL32_B_SHIFT) & 0xFF) * t);
    return IM_COL32(r, g, b, 255);
}

static ImU32 interpolate_ImU32(const std::vector<ImU32>& colors, double pos)
{
    if (colors.empty())
        return IM_COL32(0x26, 0xA6, 0x9A, 255);
    if (colors.size() == 1)
        return colors.front();

    pos = std::clamp(pos, 0.0, 1.0);
    const double scaled = pos * double(colors.size() - 1);
    const size_t segment = std::min<size_t>(colors.size() - 2, size_t(std::floor(scaled)));
    const double local = scaled - double(segment);
    return interpolate_ImU32(colors[segment], colors[segment + 1], local);
}

static float bT601_luminance(ImU32 c)
{
    float r = ((c >> IM_COL32_R_SHIFT) & 0xFF) / 255.0f;
    float g = ((c >> IM_COL32_G_SHIFT) & 0xFF) / 255.0f;
    float b = ((c >> IM_COL32_B_SHIFT) & 0xFF) / 255.0f;
    return 0.299f * r + 0.587f * g + 0.114f * b;
}

static bool very_light(ImU32 c)
{
    return ((c >> IM_COL32_R_SHIFT) & 0xFF) > 224
        && ((c >> IM_COL32_G_SHIFT) & 0xFF) > 224
        && ((c >> IM_COL32_B_SHIFT) & 0xFF) > 224;
}

static bool is_gradient_definition(const MixedFilamentDefinition& definition)
{
    return definition.recipe.kind == MixedFilamentRecipeKind::WeightedBlend &&
           definition.behavior.gradient.enabled &&
           definition.recipe.blend.components.size() >= 2;
}

static std::vector<ImU32> mixed_filament_gradient_stops_ImU32(const MixedFilamentDefinition& definition,
                                                              const MixedFilamentDisplayContext& ctx)
{
    const std::vector<wxColour> preview_colors =
        build_mixed_filament_gradient_preview(definition, ctx).sampled_colors;

    std::vector<ImU32> colors;
    colors.reserve(preview_colors.size());
    for (const wxColour& color : preview_colors)
        colors.emplace_back(IM_COL32(color.Red(), color.Green(), color.Blue(), 255));
    return colors;
}

static std::vector<std::string> get_extruders_names()
{
    const size_t physical_extruders_cnt = wxGetApp().extruders_edited_cnt();
    const size_t virtual_extruders_cnt  = wxGetApp().virtual_extruders_cnt();
    const size_t total_extruders_cnt    = physical_extruders_cnt + virtual_extruders_cnt;

    std::vector<std::string> extruders_out;
    extruders_out.reserve(physical_extruders_cnt + virtual_extruders_cnt);
    for (size_t extruder_idx = 1; extruder_idx <= physical_extruders_cnt; ++extruder_idx) {
        extruders_out.emplace_back(_u8L("Extruder") + " " + std::to_string(extruder_idx));
    }

    if (const size_t mixed_cnt = colorin_mixed_visible_cnt(); mixed_cnt > 0) {
        for (size_t mixed_idx = 0; mixed_idx < mixed_cnt; ++mixed_idx)
            extruders_out.emplace_back(GUI::format(_L("Mixed Filament %s"), Slic3r::mixed_filament_index_to_letter(mixed_idx)));
        return extruders_out;
    }

    const FullSpectrum::VirtualExtruders& virtual_extruders =
        wxGetApp().plater()->model().virtual_extruders;
    for (const FullSpectrum::VirtualExtruder& ve : virtual_extruders) {
        extruders_out.emplace_back("[V] " + _u8L("Extruder") + " " + std::to_string(ve.id));
    }

    return extruders_out;
}

static std::vector<int> get_extruder_id_for_volumes(const ModelObject &model_object)
{
    std::vector<int> extruders_idx;
    extruders_idx.reserve(model_object.volumes.size());
    for (const ModelVolume *model_volume : model_object.volumes) {
        if (!model_volume->is_model_part())
            continue;

        extruders_idx.emplace_back(model_volume->extruder_id());
    }

    return extruders_idx;
}

void GLGizmoMmuSegmentation::init_extruders_data()
{
    m_original_extruders_names     = get_extruders_names();
    m_original_extruders_colors    = colorin_paint_colors();
    m_modified_extruders_colors    = m_original_extruders_colors;
    m_first_selected_extruder_idx  = 0;
    m_second_selected_extruder_idx = 1;
}

bool GLGizmoMmuSegmentation::on_init()
{
    m_shortcut_key = WXK_CONTROL_N;

    m_desc["reset_direction"]      = _u8L("Reset direction");
    m_desc["clipping_of_view"]     = _u8L("Clipping of view") + ": ";
    m_desc["cursor_size"]          = _u8L("Brush size") + ": ";
    m_desc["cursor_type"]          = _u8L("Brush shape");
    m_desc["first_color_caption"]  = _u8L("Left mouse button") + ": ";
    m_desc["first_color"]          = _u8L("First color");
    m_desc["second_color_caption"] = _u8L("Right mouse button") + ": ";
    m_desc["second_color"]         = _u8L("Erase"); // Prusa Colorin: right mouse button erases (Orca 0.9.18)
    m_desc["remove_caption"]       = _u8L("Shift + Left mouse button") + ": ";
    m_desc["remove"]               = _u8L("Remove painted color");

    m_desc["alt_caption"]          = _u8L("Alt + Mouse wheel") + ": ";
    m_desc["alt_brush"]            = _u8L("Change brush size");
    m_desc["alt_fill"]             = _u8L("Change angle");
    m_desc["alt_height_range"]     = _u8L("Change height range");

    m_desc["remove_all"]           = _u8L("Clear all");
    m_desc["circle"]               = _u8L("Circle");
    m_desc["sphere"]               = _u8L("Sphere");
    m_desc["pointer"]              = _u8L("Triangles");

    m_desc["tool_type"]            = _u8L("Tool type");
    m_desc["tool_brush"]           = _u8L("Brush");
    m_desc["tool_smart_fill"]      = _u8L("Smart fill");
    m_desc["tool_bucket_fill"]     = _u8L("Bucket fill");
    m_desc["tool_height_range"]    = _u8L("Height range");

    m_desc["smart_fill_angle"]     = _u8L("Smart fill angle");
    m_desc["bucket_fill_angle"]    = _u8L("Bucket fill angle");

    m_desc["split_triangles"]      = _u8L("Split triangles");

    m_desc["height_range_z_range"] = _u8L("Height range");

    init_extruders_data();

    return true;
}

void GLGizmoMmuSegmentation::render_painter_gizmo()
{
    const Selection& selection = m_parent.get_selection();

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glEnable(GL_DEPTH_TEST));

    render_triangles(selection);

    m_c->object_clipper()->render_cut();
    m_c->instances_hider()->render_cut();
    render_cursor();

    glsafe(::glDisable(GL_BLEND));
}

void GLGizmoMmuSegmentation::data_changed(bool is_serializing)
{
    GLGizmoPainterBase::data_changed(is_serializing);
    if (m_state != On || wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology() != ptFFF || wxGetApp().extruders_edited_cnt() <= 1)
        return;

    ModelObject* model_object = m_c->selection_info()->model_object();
    const std::vector<ColorRGBA> current_colors =
        colorin_paint_colors();
    if (int prev_extruders_count = int(m_original_extruders_colors.size());
        prev_extruders_count != int(current_colors.size()) || current_colors != m_original_extruders_colors) {
        if (int(current_colors.size()) > int(GLGizmoMmuSegmentation::EXTRUDERS_LIMIT))
            show_notification_extruders_limit_exceeded();

        this->init_extruders_data();
        // Reinitialize triangle selectors because of change of extruder count need also change the size of GLIndexedVertexArray
        if (prev_extruders_count != int(current_colors.size()))
            this->init_model_triangle_selectors();
    } else if (model_object != nullptr && get_extruder_id_for_volumes(*model_object) != m_original_volumes_extruder_idxs) {
        this->init_model_triangle_selectors();
    }
}

void GLGizmoMmuSegmentation::render_triangles(const Selection &selection) const
{
    ClippingPlaneDataWrapper clp_data = this->get_clipping_plane_data();
    auto                    *shader   = wxGetApp().get_shader("mm_gouraud");
    if (!shader)
        return;
    shader->start_using();
    shader->set_uniform("clipping_plane", clp_data.clp_dataf);
    shader->set_uniform("z_range", clp_data.z_range);
    ScopeGuard guard([shader]() { if (shader) shader->stop_using(); });

    const ModelObject *mo      = m_c->selection_info()->model_object();
    int                mesh_id = -1;
    for (const ModelVolume *mv : mo->volumes) {
        if (!mv->is_model_part())
            continue;

        ++mesh_id;

        const Transform3d trafo_matrix = mo->instances[selection.get_instance_idx()]->get_transformation().get_matrix() * mv->get_matrix();

        const bool is_left_handed = trafo_matrix.matrix().determinant() < 0.0;
        if (is_left_handed)
            glsafe(::glFrontFace(GL_CW));

        const Camera& camera = wxGetApp().plater()->get_camera();
        const Transform3d& view_matrix = camera.get_view_matrix();
        shader->set_uniform("view_model_matrix", view_matrix * trafo_matrix);
        shader->set_uniform("projection_matrix", camera.get_projection_matrix());
        const Matrix3d view_normal_matrix = view_matrix.matrix().block(0, 0, 3, 3) * trafo_matrix.matrix().block(0, 0, 3, 3).inverse().transpose();
        shader->set_uniform("view_normal_matrix", view_normal_matrix);

        shader->set_uniform("volume_world_matrix", trafo_matrix);
        shader->set_uniform("volume_mirrored", is_left_handed);
        static_cast<TriangleSelectorMmGui*>(m_triangle_selectors[mesh_id].get())->colorin_z_range = clp_data.z_range; // Prusa Colorin
        m_triangle_selectors[mesh_id]->render(m_imgui, trafo_matrix);

        if (is_left_handed)
            glsafe(::glFrontFace(GL_CCW));
    }
}

static void render_extruders_combo(const std::string& label,
                                   const std::vector<std::string>& extruders,
                                   const std::vector<ColorRGBA>& extruders_colors,
                                   size_t& selection_idx)
{
    assert(!extruders_colors.empty());
    assert(extruders_colors.size() == extruders_colors.size());

    size_t selection_out = selection_idx;
    // It is necessary to use BeginGroup(). Otherwise, when using SameLine() is called, then other items will be drawn inside the combobox.
    ImGui::BeginGroup();
    ImVec2 combo_pos = ImGui::GetCursorScreenPos();
    if (ImGui::BeginCombo(label.c_str(), "")) {
        for (size_t extruder_idx = 0; extruder_idx < std::min(extruders.size(), GLGizmoMmuSegmentation::EXTRUDERS_LIMIT); ++extruder_idx) {
            ImGui::PushID(int(extruder_idx));
            ImVec2 start_position = ImGui::GetCursorScreenPos();

            if (ImGui::Selectable("", extruder_idx == selection_idx))
                selection_out = extruder_idx;

            ImGui::SameLine();
            ImGuiStyle &style  = ImGui::GetStyle();
            float       height = ImGui::GetTextLineHeight();
            ImGui::GetWindowDrawList()->AddRectFilled(start_position, ImVec2(start_position.x + height + height / 2, start_position.y + height), ImGuiPSWrap::to_ImU32(extruders_colors[extruder_idx]));
            ImGui::GetWindowDrawList()->AddRect(start_position, ImVec2(start_position.x + height + height / 2, start_position.y + height), IM_COL32_BLACK);

            ImGui::SetCursorScreenPos(ImVec2(start_position.x + height + height / 2 + style.FramePadding.x, start_position.y));
            ImGui::Text("%s", extruders[extruder_idx].c_str());
            ImGui::PopID();
        }

        ImGui::EndCombo();
    }

    ImVec2      backup_pos = ImGui::GetCursorScreenPos();
    ImGuiStyle &style      = ImGui::GetStyle();

    ImGui::SetCursorScreenPos(ImVec2(combo_pos.x + style.FramePadding.x, combo_pos.y + style.FramePadding.y));
    ImVec2 p      = ImGui::GetCursorScreenPos();
    float  height = ImGui::GetTextLineHeight();

    ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + height + height / 2, p.y + height), ImGuiPSWrap::to_ImU32(extruders_colors[selection_idx]));
    ImGui::GetWindowDrawList()->AddRect(p, ImVec2(p.x + height + height / 2, p.y + height), IM_COL32_BLACK);

    ImGui::SetCursorScreenPos(ImVec2(p.x + height + height / 2 + style.FramePadding.x, p.y));
    ImGui::Text("%s", extruders[selection_out].c_str());
    ImGui::SetCursorScreenPos(backup_pos);
    ImGui::EndGroup();

    selection_idx = selection_out;
}

void GLGizmoMmuSegmentation::on_render_input_window(float x, float y, float bottom_limit)
{
    if (!m_c->selection_info()->model_object())
        return;

    const float approx_height = m_imgui->scaled(25.35f);
                            y = std::min(y, bottom_limit - approx_height);
    ImGuiPureWrap::set_next_window_pos(x, y, ImGuiCond_Always);

    ImGuiPureWrap::begin(get_name(), ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);

    // First calculate width of all the texts that are could possibly be shown. We will decide set the dialog width based on that:
    const float clipping_slider_left     = std::max(ImGuiPureWrap::calc_text_size(m_desc.at("clipping_of_view")).x,
                                                    ImGuiPureWrap::calc_text_size(m_desc.at("reset_direction")).x) + m_imgui->scaled(1.5f);
    const float cursor_slider_left       = ImGuiPureWrap::calc_text_size(m_desc.at("cursor_size")).x + m_imgui->scaled(1.f);
    const float smart_fill_slider_left   = ImGuiPureWrap::calc_text_size(m_desc.at("smart_fill_angle")).x + m_imgui->scaled(1.f);
    const float bucket_fill_slider_left  = ImGuiPureWrap::calc_text_size(m_desc.at("bucket_fill_angle")).x + m_imgui->scaled(1.f);
    const float height_range_slider_left = ImGuiPureWrap::calc_text_size(m_desc.at("height_range_z_range")).x + m_imgui->scaled(1.f);

    const float cursor_type_radio_circle  = ImGuiPureWrap::calc_text_size(m_desc["circle"]).x + m_imgui->scaled(2.5f);
    const float cursor_type_radio_sphere  = ImGuiPureWrap::calc_text_size(m_desc["sphere"]).x + m_imgui->scaled(2.5f);
    const float cursor_type_radio_pointer = ImGuiPureWrap::calc_text_size(m_desc["pointer"]).x + m_imgui->scaled(2.5f);

    const float button_width             = ImGuiPureWrap::calc_text_size(m_desc.at("remove_all")).x + m_imgui->scaled(1.f);
    const float buttons_width            = m_imgui->scaled(0.5f);
    const float minimal_slider_width     = m_imgui->scaled(4.f);
    const float color_button_width       = m_imgui->scaled(1.75f);
    const float combo_label_width        = std::max(ImGuiPureWrap::calc_text_size(m_desc.at("first_color")).x,
                                                    ImGuiPureWrap::calc_text_size(m_desc.at("second_color")).x) + m_imgui->scaled(1.f);

    const float tool_type_radio_brush        = ImGuiPureWrap::calc_text_size(m_desc["tool_brush"]).x + m_imgui->scaled(2.5f);
    const float tool_type_radio_bucket_fill  = ImGuiPureWrap::calc_text_size(m_desc["tool_bucket_fill"]).x + m_imgui->scaled(2.5f);
    const float tool_type_radio_smart_fill   = ImGuiPureWrap::calc_text_size(m_desc["tool_smart_fill"]).x + m_imgui->scaled(2.5f);
    const float tool_type_radio_height_range = ImGuiPureWrap::calc_text_size(m_desc["tool_height_range"]).x + m_imgui->scaled(2.5f);

    const float tool_type_radio_first_line  = tool_type_radio_brush + tool_type_radio_bucket_fill + tool_type_radio_smart_fill;
    const float tool_type_radio_second_line = tool_type_radio_height_range;
    const float tool_type_radio_max_width   = std::max(tool_type_radio_first_line, tool_type_radio_second_line);

    const float split_triangles_checkbox_width = ImGuiPureWrap::calc_text_size(m_desc["split_triangles"]).x + m_imgui->scaled(2.5f);

    float caption_max = 0.f;
    for (const std::string t : {"first_color", "second_color", "remove", "alt"}) {
        caption_max = std::max(caption_max, ImGuiPureWrap::calc_text_size(m_desc[t + "_caption"]).x);
    }

    float total_text_max = 0.f;
    for (const std::string t : {"first_color", "second_color", "remove", "alt_brush", "alt_fill", "alt_height_range"}) {
        total_text_max = std::max(total_text_max, ImGuiPureWrap::calc_text_size(m_desc[t]).x);
    }

    total_text_max += caption_max + m_imgui->scaled(1.f);
    caption_max    += m_imgui->scaled(1.f);

    const float sliders_left_width = std::max({smart_fill_slider_left, bucket_fill_slider_left, cursor_slider_left, clipping_slider_left, height_range_slider_left});
    const float slider_icon_width  = ImGuiPureWrap::get_slider_icon_size().x;
    float       window_width       = minimal_slider_width + sliders_left_width + slider_icon_width;
    window_width = std::max(window_width, total_text_max);
    window_width = std::max(window_width, button_width);
    window_width = std::max(window_width, split_triangles_checkbox_width);
    window_width = std::max(window_width, cursor_type_radio_circle + cursor_type_radio_sphere + cursor_type_radio_pointer);
    window_width = std::max(window_width, tool_type_radio_max_width);
    window_width = std::max(window_width, 2.f * buttons_width + m_imgui->scaled(1.f));

    auto draw_text_with_caption = [&caption_max](const std::string &caption, const std::string& text) {
        ImGuiPureWrap::text_colored(ImGuiPureWrap::COL_ORANGE_LIGHT, caption);
        ImGui::SameLine(caption_max);
        ImGuiPureWrap::text(text);
    };

    for (const std::string t : {"first_color", "second_color", "remove"}) {
        draw_text_with_caption(m_desc.at(t + "_caption"), m_desc.at(t));
    }

    std::string alt_hint_text = (m_tool_type == ToolType::BRUSH)        ? "alt_brush" :
                                (m_tool_type == ToolType::HEIGHT_RANGE) ? "alt_height_range"
                                                                        : "alt_fill";
    draw_text_with_caption(m_desc.at("alt_caption"), m_desc.at(alt_hint_text));

    ImGui::Separator();

    // Prusa Colorin: filament grid of Orca FullSpectrum 0.9.18 (numbered physical filaments,
    // lettered mixed filaments, gradient swatches, mixed filaments on their own row when > 10).
    // Left mouse button paints with the selected filament, right mouse button erases.
    {
        const float max_grid_tooltip_width = ImGui::GetFontSize() * 20.0f;
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        std::vector<std::string> physical_hex = wxGetApp().plater()->get_extruder_color_strings_from_plater_config();
        const size_t num_physical = size_t(std::max(wxGetApp().extruders_edited_cnt(), 0));
        if (physical_hex.size() > num_physical)
            physical_hex.resize(num_physical);
        const MixedFilamentDisplayContext mixed_ctx = build_mixed_filament_display_context(physical_hex);

        const int   max_filament_items_per_line = 8;
        const float font_size             = ImGui::GetFontSize() * 0.85f;
        const float filament_button_h     = std::round(ImGui::GetFrameHeight() + m_imgui->scaled(0.2f));
        const float filament_button_w     = filament_button_h;
        const float filament_item_spacing = m_imgui->scaled(0.75f);
        const float filament_item_width   = filament_button_w + filament_item_spacing;

        ImGuiPureWrap::text(_u8L("Filaments"));

        const float  start_pos_x       = ImGui::GetCursorPos().x;
        const size_t n_extruder_colors = std::min(GLGizmoMmuSegmentation::EXTRUDERS_LIMIT, m_original_extruders_colors.size());
        const bool   separate_mixed_row = (n_extruder_colors > 10);
        int  current_col       = 0;
        bool prev_was_physical = false;

        for (size_t extruder_idx = 0; extruder_idx < n_extruder_colors; ++extruder_idx) {
            const unsigned int actual_filament_id = unsigned(extruder_idx + 1);
            const bool is_mixed = colorin_mixed_visible_cnt() > 0 && actual_filament_id > num_physical;

            if (separate_mixed_row && is_mixed && prev_was_physical && current_col > 0)
                current_col = 0;
            prev_was_physical = !is_mixed;

            const ColorRGBA  &extruder_color = m_modified_extruders_colors[extruder_idx];
            const ImVec4      color_vec      = ImGuiPSWrap::to_ImVec4(extruder_color);
            const std::string color_label    = std::string("##extruder color ") + std::to_string(extruder_idx);
            const std::string item_text      = !is_mixed ? std::to_string(actual_filament_id) :
                                                           Slic3r::mixed_filament_index_to_letter(actual_filament_id - num_physical - 1);
            const ImVec2 button_size(filament_button_w, filament_button_h);

            if (current_col > 0)
                ImGui::SameLine(start_pos_x + filament_item_width * current_col);

            const std::optional<MixedFilamentDefinition> mixed_definition =
                (is_mixed && wxGetApp().preset_bundle != nullptr) ?
                    wxGetApp().preset_bundle->mixed_filaments.mixed_filament_definition_from_id(actual_filament_id, mixed_ctx.num_physical) :
                    std::nullopt;
            const bool is_gradient = mixed_definition && is_gradient_definition(*mixed_definition);

            std::vector<ImU32> gradient_stops;
            ImVec4 button_color = color_vec;
            if (is_gradient) {
                gradient_stops = mixed_filament_gradient_stops_ImU32(*mixed_definition, mixed_ctx);
                int avg_r = 0, avg_g = 0, avg_b = 0;
                for (const ImU32 stop : gradient_stops) {
                    avg_r += int((stop >> IM_COL32_R_SHIFT) & 0xFF);
                    avg_g += int((stop >> IM_COL32_G_SHIFT) & 0xFF);
                    avg_b += int((stop >> IM_COL32_B_SHIFT) & 0xFF);
                }
                const int stop_count = std::max(1, int(gradient_stops.size()));
                button_color = ImVec4(float(avg_r / stop_count) / 255.f, float(avg_g / stop_count) / 255.f, float(avg_b / stop_count) / 255.f, 1.f);
            }

            ImGuiColorEditFlags flags = ImGuiColorEditFlags_NoAlpha | ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel |
                                        ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoTooltip;
            if (m_first_selected_extruder_idx != extruder_idx)
                flags |= ImGuiColorEditFlags_NoBorder;
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGuiPureWrap::COL_ORANGE_LIGHT);
            ImGui::PushStyleColor(ImGuiCol_Border, ImGuiPureWrap::COL_ORANGE_LIGHT);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 2.0f);
            const bool color_picked = ImGui::ColorButton(color_label.c_str(), button_color, flags, button_size);
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(2);
            if (color_picked)
                m_first_selected_extruder_idx = extruder_idx;

            const ImVec2 btn_min = ImGui::GetItemRectMin();
            const ImVec2 btn_max = ImGui::GetItemRectMax();
            ImU32 text_col = IM_COL32(0, 0, 0, 255);

            if (is_gradient) {
                const float  inset = 2.5f;
                const ImVec2 fill_min(btn_min.x + inset, btn_min.y + inset);
                const ImVec2 fill_max(btn_max.x - inset, btn_max.y - inset);
                const float  fill_h = fill_max.y - fill_min.y;
                if (fill_h > 0.0f) {
                    for (int y_i = 0; y_i < int(std::round(fill_h)); ++y_i) {
                        const double pos = 1.0 - double(y_i) / double(fill_h);
                        draw_list->AddRectFilled(ImVec2(fill_min.x, fill_min.y + float(y_i)),
                                                 ImVec2(fill_max.x, fill_min.y + float(y_i + 1)),
                                                 interpolate_ImU32(gradient_stops, pos));
                    }
                }
                const bool all_stops_light = !gradient_stops.empty() &&
                    std::all_of(gradient_stops.begin(), gradient_stops.end(), [](ImU32 color) { return very_light(color); });
                if (all_stops_light)
                    draw_list->AddRect(fill_min, fill_max, IM_COL32(128, 128, 128, 255), ImGui::GetStyle().FrameRounding, 0, 1.0f);
                float lum_avg = 0.0f;
                for (const ImU32 stop : gradient_stops)
                    lum_avg += bT601_luminance(stop);
                lum_avg /= float(std::max<size_t>(1, gradient_stops.size()));
                text_col = (lum_avg < 0.51f) ? IM_COL32(255, 255, 255, 255) : IM_COL32(0, 0, 0, 255);
            } else {
                const float gray = 0.299f * extruder_color.r() + 0.587f * extruder_color.g() + 0.114f * extruder_color.b();
                text_col = (gray * 255.f < 130.f) ? IM_COL32(255, 255, 255, 255) : IM_COL32(0, 0, 0, 255);
            }

            if (ImGui::IsItemHovered()) {
                if (extruder_idx < 9)
                    ImGuiPureWrap::tooltip(_u8L("Shortcut Key ") + std::to_string(extruder_idx + 1), max_grid_tooltip_width);
                else if (!is_mixed)
                    ImGuiPureWrap::tooltip(GUI::format(_L("Filament %d"), int(actual_filament_id)), max_grid_tooltip_width);
                else
                    ImGuiPureWrap::tooltip(GUI::format(_L("Mixed Filament %s"), item_text), max_grid_tooltip_width);
            }

            const float text_w = ImGui::GetFont()->CalcTextSizeA(font_size, FLT_MAX, 0.0f, item_text.c_str()).x;
            draw_list->AddText(ImGui::GetFont(), font_size,
                               ImVec2(btn_min.x + (btn_max.x - btn_min.x - text_w) / 2.f, btn_min.y + (btn_max.y - btn_min.y - font_size) / 2.f),
                               text_col, item_text.c_str());

            if (++current_col >= max_filament_items_per_line)
                current_col = 0;
        }
        ImGui::Dummy(ImVec2(0.0f, ImGui::GetFontSize() * 0.1f));

        if (n_extruder_colors > 0) {
            int selected_filament = int(m_first_selected_extruder_idx) + 1;
            ImGui::AlignTextToFramePadding();
            ImGuiPureWrap::text(_u8L("Selected filament"));
            ImGui::SameLine();
            ImGui::PushItemWidth(m_imgui->scaled(4.5f));
            if (ImGui::InputInt("##selected_filament", &selected_filament, 1, 10, ImGuiInputTextFlags_CharsDecimal)) {
                selected_filament = std::clamp(selected_filament, 1, int(n_extruder_colors));
                m_first_selected_extruder_idx = size_t(selected_filament - 1);
            }
            ImGui::PopItemWidth();
        }
    }

    const float max_tooltip_width = ImGui::GetFontSize() * 20.0f;

    ImGui::Separator();

    ImGuiPureWrap::text(m_desc.at("tool_type"));
    ImGui::NewLine();

    const float tool_type_first_line_offset = (window_width - tool_type_radio_first_line + m_imgui->scaled(1.5f)) / 2.f;
    ImGui::SameLine(tool_type_first_line_offset);
    ImGui::PushItemWidth(tool_type_radio_brush);
    if (ImGuiPureWrap::radio_button(m_desc["tool_brush"], m_tool_type == ToolType::BRUSH)) {
        m_tool_type = ToolType::BRUSH;
        for (auto &triangle_selector : m_triangle_selectors) {
            triangle_selector->seed_fill_unselect_all_triangles();
            triangle_selector->request_update_render_data();
        }
    }

    if (ImGui::IsItemHovered())
        ImGuiPureWrap::tooltip(_u8L("Paints facets according to the chosen painting brush."), max_tooltip_width);

    ImGui::SameLine(tool_type_first_line_offset + tool_type_radio_brush);
    ImGui::PushItemWidth(tool_type_radio_smart_fill);
    if (ImGuiPureWrap::radio_button(m_desc["tool_smart_fill"], m_tool_type == ToolType::SMART_FILL)) {
        m_tool_type = ToolType::SMART_FILL;
        for (auto &triangle_selector : m_triangle_selectors) {
            triangle_selector->seed_fill_unselect_all_triangles();
            triangle_selector->request_update_render_data();
        }
    }

    if (ImGui::IsItemHovered())
        ImGuiPureWrap::tooltip(_u8L("Paints neighboring facets whose relative angle is less or equal to set angle."), max_tooltip_width);

    ImGui::SameLine(tool_type_first_line_offset + tool_type_radio_brush + tool_type_radio_smart_fill);
    ImGui::PushItemWidth(tool_type_radio_bucket_fill);
    if (ImGuiPureWrap::radio_button(m_desc["tool_bucket_fill"], m_tool_type == ToolType::BUCKET_FILL)) {
        m_tool_type = ToolType::BUCKET_FILL;
        for (auto &triangle_selector : m_triangle_selectors) {
            triangle_selector->seed_fill_unselect_all_triangles();
            triangle_selector->request_update_render_data();
        }
    }

    if (ImGui::IsItemHovered())
        ImGuiPureWrap::tooltip(_u8L("Paints neighboring facets that have the same color."), max_tooltip_width);

    ImGui::NewLine();

    const float tool_type_second_line_offset = (window_width - tool_type_radio_second_line + m_imgui->scaled(1.5f)) / 2.f;
    ImGui::SameLine(tool_type_second_line_offset);
    ImGui::PushItemWidth(tool_type_radio_height_range);
    if (ImGuiPureWrap::radio_button(m_desc["tool_height_range"], m_tool_type == ToolType::HEIGHT_RANGE)) {
        m_tool_type = ToolType::HEIGHT_RANGE;
        for (auto &triangle_selector : m_triangle_selectors) {
            triangle_selector->seed_fill_unselect_all_triangles();
            triangle_selector->request_update_render_data();
        }
    }

    if (ImGui::IsItemHovered())
        ImGuiPureWrap::tooltip(_u8L("Paints facets within the chosen height range."), max_tooltip_width);

    ImGui::Separator();

    if (m_tool_type == ToolType::BRUSH) {
        ImGuiPureWrap::text(m_desc.at("cursor_type"));
        ImGui::NewLine();

        float cursor_type_offset = (window_width - cursor_type_radio_sphere - cursor_type_radio_circle - cursor_type_radio_pointer + m_imgui->scaled(1.5f)) / 2.f;
        ImGui::SameLine(cursor_type_offset);
        ImGui::PushItemWidth(cursor_type_radio_sphere);
        if (ImGuiPureWrap::radio_button(m_desc["sphere"], m_cursor_type == TriangleSelector::CursorType::SPHERE))
            m_cursor_type = TriangleSelector::CursorType::SPHERE;

        if (ImGui::IsItemHovered())
            ImGuiPureWrap::tooltip(_u8L("Paints all facets inside, regardless of their orientation."), max_tooltip_width);

        ImGui::SameLine(cursor_type_offset + cursor_type_radio_sphere);
        ImGui::PushItemWidth(cursor_type_radio_circle);

        if (ImGuiPureWrap::radio_button(m_desc["circle"], m_cursor_type == TriangleSelector::CursorType::CIRCLE))
            m_cursor_type = TriangleSelector::CursorType::CIRCLE;

        if (ImGui::IsItemHovered())
            ImGuiPureWrap::tooltip(_u8L("Ignores facets facing away from the camera."), max_tooltip_width);

        ImGui::SameLine(cursor_type_offset + cursor_type_radio_sphere + cursor_type_radio_circle);
        ImGui::PushItemWidth(cursor_type_radio_pointer);

        if (ImGuiPureWrap::radio_button(m_desc["pointer"], m_cursor_type == TriangleSelector::CursorType::POINTER))
            m_cursor_type = TriangleSelector::CursorType::POINTER;

        if (ImGui::IsItemHovered())
            ImGuiPureWrap::tooltip(_u8L("Paints only one facet."), max_tooltip_width);

        m_imgui->disabled_begin(m_cursor_type != TriangleSelector::CursorType::SPHERE && m_cursor_type != TriangleSelector::CursorType::CIRCLE);

        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(m_desc.at("cursor_size"));
        ImGui::SameLine(sliders_left_width);
        ImGui::PushItemWidth(window_width - sliders_left_width - slider_icon_width);
        m_imgui->slider_float("##cursor_radius", &m_cursor_radius, CursorRadiusMin, CursorRadiusMax, "%.2f", 1.0f, true, _L("Alt + Mouse wheel"));

        ImGuiPureWrap::checkbox(m_desc["split_triangles"], m_triangle_splitting_enabled);

        if (ImGui::IsItemHovered())
            ImGuiPureWrap::tooltip(_u8L("Split bigger facets into smaller ones while the object is painted."), max_tooltip_width);

        m_imgui->disabled_end();

        ImGui::Separator();
    } else if (m_tool_type == ToolType::SMART_FILL || m_tool_type == ToolType::BUCKET_FILL) {
        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text((m_tool_type == ToolType::SMART_FILL ? m_desc["smart_fill_angle"] : m_desc["bucket_fill_angle"])  + ":");
        std::string format_str_angle = std::string("%.f") + I18N::translate_utf8("°", "Degree sign to use in the respective slider in MMU gizmo,"
                                                                                      "placed after the number with no whitespace in between.");
        ImGui::SameLine(sliders_left_width);
        ImGui::PushItemWidth(window_width - sliders_left_width - slider_icon_width);
        float &fill_angle = (m_tool_type == ToolType::SMART_FILL) ? m_smart_fill_angle : m_bucket_fill_angle;
        if (m_imgui->slider_float("##fill_angle", &fill_angle, SmartFillAngleMin, SmartFillAngleMax, format_str_angle.data(), 1.0f, true, _L("Alt + Mouse wheel"))) {
            for (auto &triangle_selector: m_triangle_selectors) {
                triangle_selector->seed_fill_unselect_all_triangles();
                triangle_selector->request_update_render_data();
            }
        }

        ImGui::Separator();
    } else if (m_tool_type == ToolType::HEIGHT_RANGE) {
        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(m_desc["height_range_z_range"] + ":");
        std::string format_str_angle = std::string("%.2f ") + I18N::translate_utf8("mm", "Millimeter sign to use in the respective slider in multi-material painting gizmo,"
                                                                                         "placed after the number with space in between.");
        ImGui::SameLine(sliders_left_width);
        ImGui::PushItemWidth(window_width - sliders_left_width - slider_icon_width);
        if (m_imgui->slider_float("##height_range_z_range", &m_height_range_z_range, HeightRangeZRangeMin, HeightRangeZRangeMax, format_str_angle.data(), 1.0f, true, _L("Alt + Mouse wheel"))) {
            for (auto &triangle_selector: m_triangle_selectors) {
                triangle_selector->seed_fill_unselect_all_triangles();
                triangle_selector->request_update_render_data();
            }
        }

        ImGui::Separator();
    }

    if (m_c->object_clipper()->get_position() == 0.f) {
        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(m_desc.at("clipping_of_view"));
    } else {
        if (ImGuiPureWrap::button(m_desc.at("reset_direction"))) {
            wxGetApp().CallAfter([this]() { m_c->object_clipper()->set_position_by_ratio(-1., false); });
        }
    }

    auto clp_dist = float(m_c->object_clipper()->get_position());
    ImGui::SameLine(sliders_left_width);
    ImGui::PushItemWidth(window_width - sliders_left_width - slider_icon_width);
    if (m_imgui->slider_float("##clp_dist", &clp_dist, 0.f, 1.f, "%.2f", 1.0f, true, from_u8(GUI::shortkey_ctrl_prefix()) + _L("Mouse wheel")))
        m_c->object_clipper()->set_position_by_ratio(clp_dist, true);

    ImGui::Separator();
    if (ImGuiPureWrap::button(m_desc.at("remove_all"))) {
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), _L("Reset selection"),
                                      UndoRedo::SnapshotType::GizmoAction);
        ModelObject *        mo  = m_c->selection_info()->model_object();
        int                  idx = -1;
        for (ModelVolume *mv : mo->volumes)
            if (mv->is_model_part()) {
                ++idx;
                m_triangle_selectors[idx]->reset();
                m_triangle_selectors[idx]->request_update_render_data();
            }

        update_model_object();
        m_parent.set_as_dirty();
    }

    ImGuiPureWrap::end();
}

void GLGizmoMmuSegmentation::update_model_object() const
{
    bool updated = false;
    ModelObject* mo = m_c->selection_info()->model_object();
    int idx = -1;
    for (ModelVolume* mv : mo->volumes) {
        if (! mv->is_model_part())
            continue;
        ++idx;
        updated |= mv->mm_segmentation_facets.set(*m_triangle_selectors[idx]);
    }

    if (updated) {
        const ModelObjectPtrs &mos = wxGetApp().model().objects;
        wxGetApp().obj_list()->update_info_items(std::find(mos.begin(), mos.end(), mo) - mos.begin());
        m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
    }
}

void GLGizmoMmuSegmentation::init_model_triangle_selectors()
{
    const int          extruders_count = int(m_original_extruders_colors.size());
    const ModelObject *mo              = m_c->selection_info()->model_object();
    m_triangle_selectors.clear();

    // Don't continue when extruders colors are not initialized
    if(m_original_extruders_colors.empty())
        return;

    for (const ModelVolume *mv : mo->volumes) {
        if (!mv->is_model_part())
            continue;

        // This mesh does not account for the possible Z up SLA offset.
        const TriangleMesh *mesh = &mv->mesh();

        const int physical_cnt = wxGetApp().extruders_edited_cnt();
        const FullSpectrum::VirtualExtruders& virtual_extruders =
            mo->get_model()->virtual_extruders;
        size_t extruder_idx = get_extruder_color_idx(*mv, physical_cnt, virtual_extruders);
        if (colorin_mixed_visible_cnt() > 0) // Prusa Colorin: base colour = volume extruder (physical or mixed)
            extruder_idx = size_t(std::max(mv->extruder_id(), 1) - 1);
        extruder_idx = std::min(extruder_idx, m_original_extruders_colors.size() - 1);
        m_triangle_selectors.emplace_back(std::make_unique<TriangleSelectorMmGui>(*mesh, m_modified_extruders_colors, m_original_extruders_colors[extruder_idx]));
        static_cast<TriangleSelectorMmGui*>(m_triangle_selectors.back().get())->set_colorin_gradients(colorin_paint_gradients(m_original_extruders_colors.size())); // Prusa Colorin
        // Reset of TriangleSelector is done inside TriangleSelectorMmGUI's constructor, so we don't need it to perform it again in deserialize().
        m_triangle_selectors.back()->deserialize(mv->mm_segmentation_facets.get_data(), false);
        m_triangle_selectors.back()->request_update_render_data();
    }
    m_original_volumes_extruder_idxs = get_extruder_id_for_volumes(*mo);
}

void GLGizmoMmuSegmentation::update_from_model_object()
{
    wxBusyCursor wait;

    // Extruder colors need to be reloaded before calling init_model_triangle_selectors to render painted triangles
    // using colors from loaded 3MF and not from printer profile in Slicer.
    if (const std::vector<ColorRGBA> current_colors = colorin_paint_colors();
        int(m_original_extruders_colors.size()) != int(current_colors.size()) || current_colors != m_original_extruders_colors)
        this->init_extruders_data();

    this->init_model_triangle_selectors();
}

PainterGizmoType GLGizmoMmuSegmentation::get_painter_type() const
{
    return PainterGizmoType::MM_SEGMENTATION;
}

ColorRGBA GLGizmoMmuSegmentation::get_cursor_sphere_left_button_color() const
{
    ColorRGBA color = m_modified_extruders_colors[m_first_selected_extruder_idx];
    color.a(0.25f);
    return color;
}

ColorRGBA GLGizmoMmuSegmentation::get_cursor_sphere_right_button_color() const
{
    // Prusa Colorin: right mouse button erases (Orca 0.9.18), show the erase cursor colour.
    return ColorRGBA(0.5f, 0.5f, 0.5f, 0.25f);
}

void TriangleSelectorMmGui::render(ImGuiWrapper* imgui, const Transform3d& matrix)
{
    if (m_update_render_data)
        update_render_data();

    auto *shader = wxGetApp().get_current_shader();
    if (!shader)
        return;

    assert(shader->get_name() == "mm_gouraud");

    for (size_t color_idx = 0; color_idx < m_gizmo_scene.triangle_indices.size(); ++color_idx) {
        if (m_gizmo_scene.has_VBOs(color_idx)) {
            if (color_idx > m_colors.size()) // Seed fill VBO
                shader->set_uniform("uniform_color", TriangleSelectorGUI::get_seed_fill_color(color_idx == (m_colors.size() + 1) ? m_default_volume_color : m_colors[color_idx - (m_colors.size() + 1) - 1]));
            else {                           // Normal VBO
                // Prusa Colorin (Orca 0.9.18 render_preview_gradient_model): gradient mixed filaments
                // are drawn in horizontal bands along world Z, bottom to top, clipped by z_range.
                if (color_idx >= 1 && color_idx <= m_colors.size() && color_idx - 1 < m_colorin_gradients.size() &&
                    m_colorin_gradients[color_idx - 1].size() >= 2 && colorin_render_gradient(color_idx, matrix))
                    continue;
                shader->set_uniform("uniform_color", color_idx == 0 ? m_default_volume_color : m_colors[color_idx - 1]);
            }

            m_gizmo_scene.render(color_idx);
        }
    }

    render_paint_contour(matrix);
    m_update_render_data = false;
}

// Prusa Colorin: helpers of Orca FullSpectrum 0.9.18 3DScene.cpp (preview gradient sampling).
static ColorRGBA colorin_interpolate_gradient_color(const ColorRGBA &a, const ColorRGBA &b, double local)
{
    local = std::clamp(local, 0.0, 1.0);
    return ColorRGBA(float(double(a.r()) * (1.0 - local) + double(b.r()) * local),
                     float(double(a.g()) * (1.0 - local) + double(b.g()) * local),
                     float(double(a.b()) * (1.0 - local) + double(b.b()) * local),
                     float(double(a.a()) * (1.0 - local) + double(b.a()) * local));
}

static ColorRGBA colorin_sample_gradient_color(const std::vector<ColorRGBA> &colors, double t)
{
    if (colors.empty())
        return ColorRGBA::WHITE();
    if (colors.size() == 1)
        return colors.front();
    const double scaled = std::clamp(t, 0.0, std::nextafter(1.0, 0.0)) * double(colors.size() - 1);
    const size_t idx    = std::min<size_t>(colors.size() - 2, size_t(std::floor(scaled)));
    return colorin_interpolate_gradient_color(colors[idx], colors[idx + 1], scaled - double(idx));
}

bool TriangleSelectorMmGui::colorin_render_gradient(size_t color_idx, const Transform3d &matrix)
{
    auto *shader = wxGetApp().get_current_shader();
    if (shader == nullptr || !m_colorin_bbox_valid)
        return false;

    double z_min = DBL_MAX, z_max = -DBL_MAX;
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3d p((corner & 1) ? m_colorin_bbox_max.x() : m_colorin_bbox_min.x(),
                      (corner & 2) ? m_colorin_bbox_max.y() : m_colorin_bbox_min.y(),
                      (corner & 4) ? m_colorin_bbox_max.z() : m_colorin_bbox_min.z());
        const double z = (matrix * p).z();
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
    }
    const double z_height = z_max - z_min;
    const double clip_min = std::max<double>(z_min, colorin_z_range[0]);
    const double clip_max = std::min<double>(z_max, colorin_z_range[1]);
    if (z_height <= EPSILON || clip_max <= clip_min + EPSILON)
        return false;

    const std::vector<ColorRGBA> &colors = m_colorin_gradients[color_idx - 1];
    const float  alpha      = m_colors[color_idx - 1].a();
    const size_t band_count = std::clamp<size_t>(colors.size() * 16, 24, 64);
    for (size_t band_idx = 0; band_idx < band_count; ++band_idx) {
        const double band_min = z_min + z_height * double(band_idx) / double(band_count);
        const double band_max = z_min + z_height * double(band_idx + 1) / double(band_count);
        const double draw_min = std::max<double>(band_min, clip_min);
        const double draw_max = std::min<double>(band_max, clip_max);
        if (draw_max <= draw_min + EPSILON)
            continue;
        ColorRGBA band_color = colorin_sample_gradient_color(colors, (double(band_idx) + 0.5) / double(band_count));
        band_color.a(alpha);
        shader->set_uniform("uniform_color", band_color);
        shader->set_uniform("z_range", std::array<float, 2>{ float(draw_min), float(draw_max) });
        m_gizmo_scene.render(color_idx);
    }
    shader->set_uniform("z_range", colorin_z_range);
    return true;
}

void TriangleSelectorMmGui::update_render_data()
{
    // Prusa Colorin: model-space bounding box for the gradient bands.
    m_colorin_bbox_valid = !m_vertices.empty();
    if (m_colorin_bbox_valid) {
        m_colorin_bbox_min = m_vertices.front().v.cast<double>();
        m_colorin_bbox_max = m_colorin_bbox_min;
        for (const Vertex &vr : m_vertices) {
            const Vec3d v = vr.v.cast<double>();
            m_colorin_bbox_min = m_colorin_bbox_min.cwiseMin(v);
            m_colorin_bbox_max = m_colorin_bbox_max.cwiseMax(v);
        }
    }
    m_gizmo_scene.release_geometry();
    m_vertices.reserve(m_vertices.size() * 3);
    for (const Vertex &vr : m_vertices) {
        m_gizmo_scene.vertices.emplace_back(vr.v.x());
        m_gizmo_scene.vertices.emplace_back(vr.v.y());
        m_gizmo_scene.vertices.emplace_back(vr.v.z());
    }
    m_gizmo_scene.finalize_vertices();

    for (const Triangle &tr : m_triangles)
        if (tr.valid() && !tr.is_split()) {
            int               color = int(tr.get_state()) <= int(m_colors.size()) ? int(tr.get_state()) : 0;
            assert(m_colors.size() + 1 + color < m_gizmo_scene.triangle_indices.size());
            std::vector<int> &iva   = m_gizmo_scene.triangle_indices[color + tr.is_selected_by_seed_fill() * (m_colors.size() + 1)];

            if (iva.size() + 3 > iva.capacity())
                iva.reserve(next_highest_power_of_2(iva.size() + 3));

            iva.emplace_back(tr.verts_idxs[0]);
            iva.emplace_back(tr.verts_idxs[1]);
            iva.emplace_back(tr.verts_idxs[2]);
        }

    for (size_t color_idx = 0; color_idx < m_gizmo_scene.triangle_indices.size(); ++color_idx)
        m_gizmo_scene.triangle_indices_sizes[color_idx] = m_gizmo_scene.triangle_indices[color_idx].size();

    m_gizmo_scene.finalize_triangle_indices();
    update_paint_contour();
}

wxString GLGizmoMmuSegmentation::handle_snapshot_action_name(bool shift_down, GLGizmoPainterBase::Button button_down) const
{
    wxString action_name;
    if (shift_down)
        action_name = _L("Remove painted color");
    else {
        if (button_down == Button::Right) { // Prusa Colorin: right mouse button erases (Orca 0.9.18)
            action_name = _L("Remove painted color");
        } else {
            size_t extruder_id = m_first_selected_extruder_idx + 1;
            action_name        = GUI::format(_L("Painted using: Extruder %1%"), extruder_id);
        }
    }
    return action_name;
}

void GLMmSegmentationGizmo3DScene::release_geometry() {
    if (this->vertices_VBO_id) {
        glsafe(::glDeleteBuffers(1, &this->vertices_VBO_id));
        this->vertices_VBO_id = 0;
    }
    for(auto &triangle_indices_VBO_id : triangle_indices_VBO_ids) {
        glsafe(::glDeleteBuffers(1, &triangle_indices_VBO_id));
        triangle_indices_VBO_id = 0;
    }
#if !SLIC3R_OPENGL_ES
    if (OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
        if (this->vertices_VAO_id > 0) {
            glsafe(::glDeleteVertexArrays(1, &this->vertices_VAO_id));
            this->vertices_VAO_id = 0;
        }
#if !SLIC3R_OPENGL_ES
    }
#endif // !SLIC3R_OPENGL_ES

    this->clear();
}

void GLMmSegmentationGizmo3DScene::render(size_t triangle_indices_idx) const
{
    assert(triangle_indices_idx < this->triangle_indices_VBO_ids.size());
    assert(this->triangle_indices_sizes.size() == this->triangle_indices_VBO_ids.size());
#if !SLIC3R_OPENGL_ES
    if (OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
        assert(this->vertices_VAO_id != 0);
#if !SLIC3R_OPENGL_ES
    }
#endif // !SLIC3R_OPENGL_ES
    assert(this->vertices_VBO_id != 0);
    assert(this->triangle_indices_VBO_ids[triangle_indices_idx] != 0);

    GLShaderProgram* shader = wxGetApp().get_current_shader();
    if (shader == nullptr)
        return;

#if !SLIC3R_OPENGL_ES
    if (OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
        glsafe(::glBindVertexArray(this->vertices_VAO_id));
#if !SLIC3R_OPENGL_ES
    }
#endif // !SLIC3R_OPENGL_ES
    // the following binding is needed to set the vertex attributes
    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, this->vertices_VBO_id));
    const GLint position_id = shader->get_attrib_location("v_position");
    if (position_id != -1) {
        glsafe(::glVertexAttribPointer(position_id, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)nullptr));
        glsafe(::glEnableVertexAttribArray(position_id));
    }

    // Render using the Vertex Buffer Objects.
    if (this->triangle_indices_VBO_ids[triangle_indices_idx] != 0 &&
        this->triangle_indices_sizes[triangle_indices_idx] > 0) {
        glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->triangle_indices_VBO_ids[triangle_indices_idx]));
        glsafe(::glDrawElements(GL_TRIANGLES, GLsizei(this->triangle_indices_sizes[triangle_indices_idx]), GL_UNSIGNED_INT, nullptr));
        glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    }

    if (position_id != -1)
        glsafe(::glDisableVertexAttribArray(position_id));

    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));
#if !SLIC3R_OPENGL_ES
    if (OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
        glsafe(::glBindVertexArray(0));
#if !SLIC3R_OPENGL_ES
    }
#endif // !SLIC3R_OPENGL_ES
}

void GLMmSegmentationGizmo3DScene::finalize_vertices()
{
#if !SLIC3R_OPENGL_ES
    if (OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
        assert(this->vertices_VAO_id == 0);
#if !SLIC3R_OPENGL_ES
    }
#endif // !SLIC3R_OPENGL_ES
    assert(this->vertices_VBO_id == 0);
    if (!this->vertices.empty()) {
#if !SLIC3R_OPENGL_ES
        if (OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
            glsafe(::glGenVertexArrays(1, &this->vertices_VAO_id));
            glsafe(::glBindVertexArray(this->vertices_VAO_id));
#if !SLIC3R_OPENGL_ES
        }
#endif // !SLIC3R_OPENGL_ES

        glsafe(::glGenBuffers(1, &this->vertices_VBO_id));
        glsafe(::glBindBuffer(GL_ARRAY_BUFFER, this->vertices_VBO_id));
        glsafe(::glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(float), this->vertices.data(), GL_STATIC_DRAW));
        glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));
        this->vertices.clear();

#if !SLIC3R_OPENGL_ES
        if (OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
            glsafe(::glBindVertexArray(0));
#if !SLIC3R_OPENGL_ES
        }
#endif // !SLIC3R_OPENGL_ES
    }
}

void GLMmSegmentationGizmo3DScene::finalize_triangle_indices()
{
    assert(std::all_of(triangle_indices_VBO_ids.cbegin(), triangle_indices_VBO_ids.cend(), [](const auto &ti_VBO_id) { return ti_VBO_id == 0; }));

    assert(this->triangle_indices.size() == this->triangle_indices_VBO_ids.size());
    for (size_t buffer_idx = 0; buffer_idx < this->triangle_indices.size(); ++buffer_idx) {
        if (!this->triangle_indices[buffer_idx].empty()) {
            glsafe(::glGenBuffers(1, &this->triangle_indices_VBO_ids[buffer_idx]));
            glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->triangle_indices_VBO_ids[buffer_idx]));
            glsafe(::glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->triangle_indices[buffer_idx].size() * sizeof(int), this->triangle_indices[buffer_idx].data(), GL_STATIC_DRAW));
            glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
            this->triangle_indices[buffer_idx].clear();
        }
    }
}

} // namespace Slic3r
