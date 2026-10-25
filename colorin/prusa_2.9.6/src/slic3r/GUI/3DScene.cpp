///|/ Copyright (c) Prusa Research 2016 - 2023 Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966, Oleksandra Iushchenko @YuSanka, Tomáš Mészáros @tamasmeszaros, Vojtěch Bubník @bubnikv, Filip Sykala @Jony01, Lukáš Hejl @hejllukas, David Kocík @kocikdav, Vojtěch Král @vojtechkral
///|/ Copyright (c) 2017 Eyal Soha @eyal0
///|/ Copyright (c) Slic3r 2015 Alessandro Ranellucci @alranel
///|/
///|/ ported from lib/Slic3r/GUI/3DScene.pm:
///|/ Copyright (c) Prusa Research 2016 - 2019 Vojtěch Bubník @bubnikv, Enrico Turri @enricoturri1966, Oleksandra Iushchenko @YuSanka
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2013 Guillaume Seguin @iXce
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <GL/glew.h>

#include "3DScene.hpp"
#include "GLShader.hpp"
#include "GUI_App.hpp"
#include "Plater.hpp"
#include "BitmapCache.hpp"
#include "Camera.hpp"
#include "wxExtensions.hpp"

#include "Gizmos/GLGizmoMmuSegmentation.hpp"

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/MultipleBeds.hpp"
#include "libslic3r/ImageMap/VolumeData.hpp"
// Prusa Colorin (Orca FullSpectrum 0.9.18): image-map / mixed-filament preview.
#include "MixedColorMatchHelpers.hpp"
#include "libslic3r/MixedFilament.hpp"
#include "libslic3r/ImageMap/ContinuousColorSolver.hpp"
#include "libslic3r/ImageMap/SimplePmCalibration.hpp"
#include "libslic3r/ImageMap/FacetRasterizer.hpp"
#include "libslic3r/ImageMap/Sampling.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <thread>
#include <tuple>
#include <unordered_map>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <boost/log/trivial.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <Eigen/Dense>

#ifdef HAS_GLSAFE
void glAssertRecentCallImpl(const char* file_name, unsigned int line, const char* function_name)
{
#if defined(NDEBUG)
    // In release mode, only show OpenGL errors if sufficiently high loglevel.
    if (Slic3r::get_logging_level() < 5)
        return;
#endif // NDEBUG

    GLenum err = glGetError();
    if (err == GL_NO_ERROR)
        return;
    const char* sErr = 0;
    switch (err) {
    case GL_INVALID_ENUM:       sErr = "Invalid Enum";      break;
    case GL_INVALID_VALUE:      sErr = "Invalid Value";     break;
    // be aware that GL_INVALID_OPERATION is generated if glGetError is executed between the execution of glBegin and the corresponding execution of glEnd 
    case GL_INVALID_OPERATION:  sErr = "Invalid Operation"; break;
    case GL_STACK_OVERFLOW:     sErr = "Stack Overflow";    break;
    case GL_STACK_UNDERFLOW:    sErr = "Stack Underflow";   break;
    case GL_OUT_OF_MEMORY:      sErr = "Out Of Memory";     break;
    default:                    sErr = "Unknown";           break;
    }
    BOOST_LOG_TRIVIAL(error) << "OpenGL error in " << file_name << ":" << line << ", function " << function_name << "() : " << (int)err << " - " << sErr;
    assert(false);
}
#endif // HAS_GLSAFE

namespace Slic3r {

const float GLVolume::SinkingContours::HalfWidth = 0.25f;

void GLVolume::SinkingContours::render()
{
    update();

    GLShaderProgram* shader = GUI::wxGetApp().get_current_shader();
    if (shader == nullptr)
        return;

    const GUI::Camera& camera = GUI::wxGetApp().plater()->get_camera();
    shader->set_uniform("view_model_matrix", camera.get_view_matrix() * Geometry::translation_transform(m_shift));
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());
    m_model.render();
}

void GLVolume::SinkingContours::update()
{
    const int object_idx = m_parent.object_idx();
    const Model& model = GUI::wxGetApp().plater()->model();

    if (object_idx < 0 ||
        object_idx >= int(model.objects.size()) ||
        !m_parent.is_sinking() ||
        m_parent.is_below_printbed()){
        m_model.reset();
        return;    
    }

    const BoundingBoxf3& box = m_parent.transformed_convex_hull_bounding_box();
    if (m_old_box.size().isApprox(box.size()) &&
        m_old_box.min.z() == box.min.z()){
        // Fix it !!! It is not working all the time
        m_shift = box.center() - m_old_box.center();
        return;
    }    
    
    m_old_box = box;
    m_shift = Vec3d::Zero();

    const TriangleMesh& mesh = model.objects[object_idx]->volumes[m_parent.volume_idx()]->mesh();

    m_model.reset();
    GUI::GLModel::Geometry init_data;
    init_data.format = { GUI::GLModel::Geometry::EPrimitiveType::Triangles, GUI::GLModel::Geometry::EVertexLayout::P3 };
    init_data.color = ColorRGBA::WHITE();
    unsigned int vertices_counter = 0;
    MeshSlicingParams slicing_params;
    slicing_params.trafo = m_parent.world_matrix();
    const Polygons polygons = union_(slice_mesh(mesh.its, 0.0f, slicing_params));
    if (polygons.empty()) return;

    for (const ExPolygon& expoly : diff_ex(expand(polygons, float(scale_(HalfWidth))), shrink(polygons, float(scale_(HalfWidth))))) {
        const std::vector<Vec3d> triangulation = triangulate_expolygon_3d(expoly);
        init_data.reserve_vertices(init_data.vertices_count() + triangulation.size());
        init_data.reserve_indices(init_data.indices_count() + triangulation.size());
        for (const Vec3d& v : triangulation) {
            init_data.add_vertex((Vec3f)(v.cast<float>() + 0.015f * Vec3f::UnitZ())); // add a small positive z to avoid z-fighting
            ++vertices_counter;
            if (vertices_counter % 3 == 0)
                init_data.add_triangle(vertices_counter - 3, vertices_counter - 2, vertices_counter - 1);
        }
    }

    if (init_data.vertices_count() > 0)
      m_model.init_from(std::move(init_data));
}

void GLVolume::NonManifoldEdges::render()
{
    update();

#if !SLIC3R_OPENGL_ES
    if (!GUI::OpenGLManager::get_gl_info().is_core_profile())
        glsafe(::glLineWidth(2.0f));
#endif // !SLIC3R_OPENGL_ES

    GLShaderProgram* shader = GUI::wxGetApp().get_current_shader();
    if (shader == nullptr)
        return;

    const GUI::Camera& camera = GUI::wxGetApp().plater()->get_camera();
    shader->set_uniform("view_model_matrix", camera.get_view_matrix() * m_parent.world_matrix());
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());
#if !SLIC3R_OPENGL_ES
    if (GUI::OpenGLManager::get_gl_info().is_core_profile()) {
#endif // !SLIC3R_OPENGL_ES
        const std::array<int, 4>& viewport = camera.get_viewport();
        shader->set_uniform("viewport_size", Vec2d(double(viewport[2]), double(viewport[3])));
        shader->set_uniform("width", 0.5f);
        shader->set_uniform("gap_size", 0.0f);
#if !SLIC3R_OPENGL_ES
    }
#endif // !SLIC3R_OPENGL_ES
    m_model.set_color(complementary(m_parent.render_color));
    m_model.render();
}

void GLVolume::NonManifoldEdges::update()
{
    if (!m_update_needed)
        return;

    m_model.reset();
    const int object_idx = m_parent.object_idx();
    const Model& model = GUI::wxGetApp().plater()->model();
    if (0 <= object_idx && object_idx < int(model.objects.size())) {
        const ModelObject* model_object = model.objects[object_idx];
        const int volume_idx = m_parent.volume_idx();
        if (0 <= volume_idx && volume_idx < int(model_object->volumes.size())) {
            const ModelVolume* model_volume = model_object->volumes[volume_idx];
            const TriangleMesh& mesh = model_volume->mesh();
            const std::vector<std::pair<int, int>> edges = its_get_open_edges(mesh.its);
            if (!edges.empty()) {
                GUI::GLModel::Geometry init_data;
                init_data.format = { GUI::GLModel::Geometry::EPrimitiveType::Lines, GUI::GLModel::Geometry::EVertexLayout::P3 };
                init_data.reserve_vertices(2 * edges.size());
                init_data.reserve_indices(2 * edges.size());

                // vertices + indices
                unsigned int vertices_count = 0;
                for (const std::pair<int, int>& edge : edges) {
                    init_data.add_vertex((Vec3f)mesh.its.vertices[edge.first].cast<float>());
                    init_data.add_vertex((Vec3f)mesh.its.vertices[edge.second].cast<float>());
                    vertices_count += 2;
                    init_data.add_line(vertices_count - 2, vertices_count - 1);
                }
                m_model.init_from(std::move(init_data));
            }
        }
    }

    m_update_needed = false;
}

const ColorRGBA GLVolume::SELECTED_COLOR         = ColorRGBA::GREEN();
const ColorRGBA GLVolume::HOVER_SELECT_COLOR     = { 0.4f, 0.9f, 0.1f, 1.0f };
const ColorRGBA GLVolume::HOVER_DESELECT_COLOR   = { 1.0f, 0.75f, 0.75f, 1.0f };
const ColorRGBA GLVolume::OUTSIDE_COLOR          = { 0.0f, 0.38f, 0.8f, 1.0f };
const ColorRGBA GLVolume::SELECTED_OUTSIDE_COLOR = { 0.19f, 0.58f, 1.0f, 1.0f };
const ColorRGBA GLVolume::DISABLED_COLOR         = ColorRGBA::DARK_GRAY();
const ColorRGBA GLVolume::SLA_SUPPORT_COLOR      = ColorRGBA::LIGHT_GRAY();
const ColorRGBA GLVolume::SLA_PAD_COLOR          = { 0.0f, 0.2f, 0.0f, 1.0f };
const ColorRGBA GLVolume::NEUTRAL_COLOR          = { 0.9f, 0.9f, 0.9f, 1.0f };
const std::array<ColorRGBA, 4> GLVolume::MODEL_COLOR = { {
    ColorRGBA::YELLOW(),
    { 1.0f, 0.5f, 0.5f, 1.0f },
    { 0.5f, 1.0f, 0.5f, 1.0f },
    { 0.5f, 0.5f, 1.0f, 1.0f }
} };
const ColorRGBA GLVolume::NEGATIVE_VOLUME_COLOR     = { 0.2f, 0.2f, 0.2f, 0.5f };
const ColorRGBA GLVolume::PARAMETER_MODIFIER_COLOR  = { 1.0, 1.0f, 0.2f, 0.5f };
const ColorRGBA GLVolume::SUPPORT_BLOCKER_COLOR     = { 1.0f, 0.2f, 0.2f, 0.5f };
const ColorRGBA GLVolume::SUPPORT_ENFORCER_COLOR    = { 0.2f, 0.2f, 1.0f, 0.5f };

namespace GUI {

namespace {
constexpr const char* IMAGE_MAP_PREVIEW_PREDICTED_COLORS_KEY = "image_map_preview_predicted_colors";
}

bool image_map_preview_predicted_colors()
{
    return wxGetApp().app_config != nullptr && wxGetApp().app_config->get_bool(IMAGE_MAP_PREVIEW_PREDICTED_COLORS_KEY);
}

void set_image_map_preview_predicted_colors(bool enabled)
{
    if (wxGetApp().app_config != nullptr)
        wxGetApp().app_config->set(IMAGE_MAP_PREVIEW_PREDICTED_COLORS_KEY, enabled ? "1" : "0");
}

} // namespace GUI

// Prusa Colorin (Orca FullSpectrum 0.9.18): background job that prepares the source-colour preview.
enum class SourceColorPreviewStatus : unsigned char { Running, Ready, Uploading, Uploaded, Cancelled, Failed };

struct SourceTexturePreviewPart
{
    std::unique_ptr<GUI::GLModel::Geometry> geometry;
    std::vector<unsigned char>              rgba;
    unsigned int                            width{0};
    unsigned int                            height{0};
    ImageMap::WrapMode                      wrap_u{ImageMap::WrapMode::Repeat};
    ImageMap::WrapMode                      wrap_v{ImageMap::WrapMode::Repeat};
};

struct SourceColorPreviewJob
{
    explicit SourceColorPreviewJob(std::shared_ptr<const ImageMap::VolumeData> source_data) : data(std::move(source_data)) {}

    std::shared_ptr<const ImageMap::VolumeData> data;
    std::atomic<SourceColorPreviewStatus>       status{SourceColorPreviewStatus::Running};
    std::atomic<int>                            progress{1};
    std::atomic<bool>                           cancel{false};
    std::atomic<bool>                           ready_presented{false};
    std::atomic<bool>                           uploaded_presented{false};
    std::mutex                                  result_mutex;
    std::unique_ptr<GUI::GLModel::Geometry>     geometry;
    std::vector<SourceTexturePreviewPart>       texture_parts;
};

// Prusa Colorin (Orca FullSpectrum 0.9.18): source-colour / texture preview helpers (V2a).
namespace {

// Preserve enough facial/texture detail for large scanned OBJs. Rasterization
// is bounded, cancellable and runs off the UI thread, so this can be materially
// higher than the old emergency cap without bringing back transform stalls.
constexpr size_t       k_source_color_preview_triangle_cap = 200'000;
constexpr size_t       k_source_color_preview_cache_cap    = 65'536;
constexpr unsigned int k_source_texture_preview_max_edge   = 2'048;
constexpr size_t       k_source_texture_preview_max_pixels = size_t(2'048) * size_t(2'048);
constexpr unsigned int k_interactive_texture_preview_max_edge   = 2'048;
constexpr size_t       k_interactive_texture_preview_max_pixels = size_t(2) * size_t(1'024) * size_t(1'024);

struct SourceColorPreviewAssignment
{
    uint32_t                                        zone_index{0};
    RGBA                                            target_color{1.f, 1.f, 1.f, 1.f};
    unsigned int                                    filament_id{0};
    RGBA                                            display_color{1.f, 1.f, 1.f, 1.f};
    std::vector<ImageMap::ContinuousColorComponent> components;
};

const SourceColorPreviewAssignment* nearest_source_color_assignment(const std::vector<SourceColorPreviewAssignment>& assignments,
                                                                    const RGBA&                                      color,
                                                                    std::optional<uint32_t> zone_index = std::nullopt)
{
    const SourceColorPreviewAssignment* best_assignment = nullptr;
    double                              best_distance   = std::numeric_limits<double>::infinity();
    for (const SourceColorPreviewAssignment& assignment : assignments) {
        if (zone_index && assignment.zone_index != *zone_index)
            continue;
        double distance = 0.0;
        for (size_t component = 0; component < 3; ++component) {
            const double delta = double(color[component]) - double(assignment.target_color[component]);
            distance += delta * delta;
        }
        if (distance < best_distance) {
            best_distance   = distance;
            best_assignment = &assignment;
        }
    }
    return best_assignment;
}

unsigned int nearest_source_color_filament(const std::vector<SourceColorPreviewAssignment>& assignments,
                                           const RGBA&                                      color,
                                           std::optional<uint32_t>                          zone_index = std::nullopt)
{
    const SourceColorPreviewAssignment* assignment = nearest_source_color_assignment(assignments, color, zone_index);
    return assignment != nullptr ? assignment->filament_id : 0;
}

size_t source_color_preview_signature(const std::vector<ImageMap::ContinuousColorComponent>& components)
{
    size_t seed         = components.size();
    auto   hash_combine = [&seed](size_t value) { seed ^= value + size_t(0x9e3779b9) + (seed << 6) + (seed >> 2); };
    hash_combine(size_t(ImageMap::simple_pm_calibration_revision()));
    for (const ImageMap::ContinuousColorComponent& component : components) {
        hash_combine(std::hash<std::string>{}(component.color_hex));
        hash_combine(component.transmission_distance_mm ? std::hash<double>{}(*component.transmission_distance_mm) : 0u);
        hash_combine(component.material_id ? std::hash<std::string>{}(*component.material_id) : 0u);
    }
    return seed;
}

size_t source_color_preview_zone_signature(const ImageMap::VolumeData& data)
{
    size_t seed = data.zones.size();
    auto hash_combine = [&seed](size_t value) { seed ^= value + size_t(0x9e3779b9) + (seed << 6) + (seed >> 2); };
    auto hash_scalar = [&hash_combine](double value) { hash_combine(std::hash<double>{}(value)); };
    for (const ImageMap::Zone& zone : data.zones) {
        hash_combine(size_t(zone.enabled));
        hash_combine(static_cast<size_t>(zone.render_mode));
        hash_combine(static_cast<size_t>(zone.color_mix_model));
        hash_scalar(zone.modulation_sample_spacing_mm);
        hash_combine(size_t(zone.disable_broad_path_smoothing));
        hash_scalar(zone.gaussian_smoothing_strength);
        hash_scalar(zone.first_path_smoothing_strength);
        hash_scalar(zone.second_path_smoothing_strength);
        hash_scalar(zone.corner_smoothing_radius_mm);
        hash_scalar(zone.tone_gamma);
        hash_scalar(zone.overhang_contrast_percent);
        hash_scalar(zone.image_exposure_ev);
        hash_scalar(zone.image_contrast_percent);
        hash_scalar(zone.image_saturation_percent);
        hash_scalar(zone.image_edge_boost_percent);
    }
    return seed;
}

bool source_color_preview_components_equal(const std::vector<ImageMap::ContinuousColorComponent>& lhs,
                                           const std::vector<ImageMap::ContinuousColorComponent>& rhs)
{
    if (lhs.size() != rhs.size())
        return false;
    for (size_t index = 0; index < lhs.size(); ++index) {
        if (lhs[index].color_hex != rhs[index].color_hex ||
            lhs[index].transmission_distance_mm != rhs[index].transmission_distance_mm ||
            lhs[index].material_id != rhs[index].material_id)
            return false;
    }
    return true;
}

std::shared_ptr<const ImageMap::ContinuousColorSolver> source_color_preview_solver(
    const std::vector<ImageMap::ContinuousColorComponent>& components,
    ImageMap::ColorMixModel                                color_mix_model)
{
    struct CacheEntry
    {
        ImageMap::ColorMixModel                         color_mix_model{ImageMap::ColorMixModel::FullSpectrumKmKs};
        uint64_t                                        calibration_revision{0};
        std::vector<ImageMap::ContinuousColorComponent> components;
        std::shared_ptr<const ImageMap::ContinuousColorSolver> solver;
    };
    struct CacheState
    {
        std::mutex              mutex;
        std::vector<CacheEntry> entries;
    };

    // Solver construction includes generating and predicting the complete
    // printable candidate set. Live controls used to launch that same work in
    // every replacement job; cancellation cannot interrupt the constructor,
    // so those obsolete jobs accumulated at 2%. Serialize cache misses and
    // retain a small set covering the active filament/model configurations.
    // Preview workers are detached and may still be winding down during GUI
    // shutdown, so keep this process-lifetime state out of static destruction.
    static CacheState*          cache = new CacheState;
    std::lock_guard<std::mutex> lock(cache->mutex);
    const uint64_t calibration_revision = ImageMap::simple_pm_calibration_revision();
    const auto cached = std::find_if(cache->entries.begin(), cache->entries.end(), [&components, color_mix_model, calibration_revision](const CacheEntry& entry) {
        return entry.color_mix_model == color_mix_model && entry.calibration_revision == calibration_revision &&
               source_color_preview_components_equal(entry.components, components);
    });
    if (cached != cache->entries.end())
        return cached->solver;

    auto solver = std::make_shared<ImageMap::ContinuousColorSolver>(components, color_mix_model);
    if (!solver->valid())
        return {};
    constexpr size_t maximum_cached_solvers = 12;
    if (cache->entries.size() >= maximum_cached_solvers)
        cache->entries.erase(cache->entries.begin());
    cache->entries.push_back(CacheEntry{color_mix_model, calibration_revision, components, solver});
    return solver;
}

std::vector<ImageMap::ContinuousColorComponent> source_color_preview_components()
{
    std::vector<ImageMap::ContinuousColorComponent> components;
    // Prusa Colorin: PrusaSlicer keeps the per-extruder filament colours in the Plater config
    // (Orca keeps them in preset_bundle->project_config).
    const GUI::Plater* plater = GUI::wxGetApp().plater();
    const DynamicPrintConfig* config = plater != nullptr ? plater->config() : nullptr;
    if (config == nullptr)
        return components;

    const ConfigOptionStrings* colors = config->option<ConfigOptionStrings>("filament_colour");
    if (colors == nullptr)
        return components;
    const ConfigOptionFloats* transmission_distances = config->option<ConfigOptionFloats>("filament_transmission_distance");
    const ConfigOptionStrings* material_ids = config->option<ConfigOptionStrings>("filament_full_spectrum_material_id");

    components.reserve(colors->values.size());
    for (size_t index = 0; index < colors->values.size(); ++index) {
        ImageMap::ContinuousColorComponent component;
        component.color_hex = colors->values[index];
        if (MixedFilamentManager::use_td_for_color_prediction() && transmission_distances != nullptr &&
            index < transmission_distances->values.size()) {
            const double td_mm = transmission_distances->values[index];
            if (std::isfinite(td_mm) && td_mm > EPSILON)
                component.transmission_distance_mm = td_mm;
        }
        if (material_ids != nullptr && index < material_ids->values.size() && !material_ids->values[index].empty())
            component.material_id = material_ids->values[index];
        components.emplace_back(std::move(component));
    }
    return components;
}

uint32_t preview_color_key(const RGBA& color)
{
    auto channel = [](float value) { return uint32_t(std::lround(std::clamp(value, 0.f, 1.f) * 255.f)); };
    return (channel(color[0]) << 16) | (channel(color[1]) << 8) | channel(color[2]);
}

RGBA source_color_preview_rgba(const std::string& color_hex)
{
    const wxColour color = GUI::parse_mixed_color(color_hex);
    return {float(color.Red()) / 255.f, float(color.Green()) / 255.f, float(color.Blue()) / 255.f, 1.f};
}

size_t source_color_preview_assignment_signature(const std::vector<SourceColorPreviewAssignment>& assignments)
{
    size_t seed         = assignments.size();
    auto   hash_combine = [&seed](size_t value) { seed ^= value + size_t(0x9e3779b9) + (seed << 6) + (seed >> 2); };
    for (const SourceColorPreviewAssignment& assignment : assignments) {
        hash_combine(assignment.zone_index);
        hash_combine(preview_color_key(assignment.target_color));
        hash_combine(assignment.filament_id);
        hash_combine(preview_color_key(assignment.display_color));
        for (const ImageMap::ContinuousColorComponent& component : assignment.components) {
            hash_combine(std::hash<std::string>{}(component.color_hex));
            hash_combine(component.transmission_distance_mm ? std::hash<double>{}(*component.transmission_distance_mm) : 0u);
            hash_combine(component.material_id ? std::hash<std::string>{}(*component.material_id) : 0u);
        }
    }
    return seed;
}

std::array<Vec2f, 3> unwrap_source_texture_uvs(const ImageMap::SurfaceSource& source)
{
    std::array<Vec2f, 3> out         = source.uvs;
    auto                 unwrap_axis = [&out](bool use_u_axis, ImageMap::WrapMode wrap_mode) {
        if (wrap_mode != ImageMap::WrapMode::Repeat)
            return;
        std::array<float, 3> values{use_u_axis ? out[0].x() : out[0].y(), use_u_axis ? out[1].x() : out[1].y(),
                                    use_u_axis ? out[2].x() : out[2].y()};
        if (!std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); }))
            return;
        const bool has_repeat_evidence = std::any_of(values.begin(), values.end(), [](float value) {
            constexpr float epsilon = 1e-6f;
            return value < -epsilon || value > 1.f + epsilon;
        });
        auto       span                = [](const std::array<float, 3>& coordinates) {
            return std::max({coordinates[0], coordinates[1], coordinates[2]}) - std::min({coordinates[0], coordinates[1], coordinates[2]});
        };
        const float original_span = span(values);
        if (!has_repeat_evidence || original_span <= 0.5f)
            return;

        std::array<float, 3> best      = values;
        float                best_span = original_span;
        for (size_t anchor = 0; anchor < values.size(); ++anchor) {
            std::array<float, 3> candidate = values;
            for (size_t index = 0; index < candidate.size(); ++index) {
                const float delta = values[index] - values[anchor];
                candidate[index]  = values[anchor] + delta - std::round(delta);
            }
            const float candidate_span = span(candidate);
            if (candidate_span + 1e-6f < best_span) {
                best      = candidate;
                best_span = candidate_span;
            }
        }
        if (best_span >= original_span - 1e-6f)
            return;
        for (size_t index = 0; index < out.size(); ++index) {
            if (use_u_axis)
                out[index].x() = best[index];
            else
                out[index].y() = best[index];
        }
    };
    unwrap_axis(true, source.wrap_u);
    unwrap_axis(false, source.wrap_v);
    return out;
}

std::pair<unsigned int, unsigned int> source_texture_preview_size(const ImageMap::TextureAsset& asset, bool interactive_preview)
{
    if (!asset.valid())
        return {0, 0};
    const unsigned int maximum_edge = interactive_preview ? k_interactive_texture_preview_max_edge : k_source_texture_preview_max_edge;
    const size_t maximum_pixels = interactive_preview ? k_interactive_texture_preview_max_pixels : k_source_texture_preview_max_pixels;
    double scale = std::min({1.0, double(maximum_edge) / double(asset.width), double(maximum_edge) / double(asset.height),
                             std::sqrt(double(maximum_pixels) / (double(asset.width) * double(asset.height)))});
    return {std::max(1u, unsigned(std::floor(double(asset.width) * scale))),
            std::max(1u, unsigned(std::floor(double(asset.height) * scale)))};
}

double source_texture_preview_mm_per_pixel(const indexed_triangle_set&                         its,
                                           const std::vector<const ImageMap::TriangleBinding*>& bindings,
                                           unsigned int                                        preview_width,
                                           unsigned int                                        preview_height)
{
    double model_length_sum = 0.;
    double pixel_length_sum = 0.;
    for (const ImageMap::TriangleBinding* binding : bindings) {
        if (binding == nullptr || binding->triangle_index >= its.indices.size())
            continue;
        const stl_triangle_vertex_indices& indices = its.indices[binding->triangle_index];
        if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0 || size_t(indices[0]) >= its.vertices.size() ||
            size_t(indices[1]) >= its.vertices.size() || size_t(indices[2]) >= its.vertices.size())
            continue;
        const std::array<Vec2f, 3> uvs = unwrap_source_texture_uvs(binding->source);
        for (size_t edge = 0; edge < 3; ++edge) {
            const size_t next = (edge + 1) % 3;
            const Vec2f  pixel_delta((uvs[next].x() - uvs[edge].x()) * float(preview_width),
                                    (uvs[next].y() - uvs[edge].y()) * float(preview_height));
            const double pixel_length = double(pixel_delta.norm());
            const double model_length = double((its.vertices[size_t(indices[next])] - its.vertices[size_t(indices[edge])]).norm());
            if (!std::isfinite(pixel_length) || !std::isfinite(model_length) || pixel_length <= EPSILON || model_length <= EPSILON)
                continue;
            pixel_length_sum += pixel_length;
            model_length_sum += model_length;
        }
    }
    return pixel_length_sum > EPSILON ? std::clamp(model_length_sum / pixel_length_sum, 0.002, 2.0) : 0.05;
}

struct PreviewFilterLayout
{
    // An almost-horizontal mapped surface lies in one slice plane, so its
    // reconstruction neighbourhood is genuinely two-dimensional. Side walls
    // instead use the equal-Z scan lines below; filtering across those lines
    // would mix different print layers in the prepare preview.
    bool                             gaussian_is_two_dimensional{false};
    double                           mm_per_step{0.05};
    std::vector<std::vector<size_t>> equal_z_lines;
};

PreviewFilterLayout source_texture_preview_filter_layout(
    const indexed_triangle_set&                         its,
    const std::vector<const ImageMap::TriangleBinding*>& bindings,
    unsigned int                                        preview_width,
    unsigned int                                        preview_height,
    const Transform3d&                                  mesh_to_world,
    const std::atomic<bool>&                            cancel)
{
    PreviewFilterLayout result;
    if (preview_width == 0 || preview_height == 0)
        return result;

    // Fit the affine texture-pixel -> world-position map represented by the
    // bound triangles. Projection UVs are affine even when triangle edges
    // split the image, and the Z rows remain affine on common curved targets
    // such as cylinders. Using all corners makes the estimate insensitive to
    // the mesh tessellation and gives us the direction tangent to a slice.
    Eigen::Matrix3d normal_matrix = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d right_hand    = Eigen::Matrix3d::Zero();
    size_t          sample_count  = 0;
    for (size_t binding_index = 0; binding_index < bindings.size(); ++binding_index) {
        if ((binding_index & 0x3ffu) == 0u && cancel.load(std::memory_order_relaxed))
            return result;
        const ImageMap::TriangleBinding* binding = bindings[binding_index];
        if (binding == nullptr || binding->triangle_index >= its.indices.size())
            continue;
        const stl_triangle_vertex_indices& indices = its.indices[binding->triangle_index];
        if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0 || size_t(indices[0]) >= its.vertices.size() ||
            size_t(indices[1]) >= its.vertices.size() || size_t(indices[2]) >= its.vertices.size())
            continue;
        const std::array<Vec2f, 3> uvs = unwrap_source_texture_uvs(binding->source);
        for (size_t corner = 0; corner < 3; ++corner) {
            const Eigen::Vector3d predictor(double(uvs[corner].x()) * double(preview_width),
                                            double(uvs[corner].y()) * double(preview_height), 1.0);
            const Vec3d world = mesh_to_world * its.vertices[size_t(indices[corner])].cast<double>();
            if (!predictor.allFinite() || !world.allFinite())
                continue;
            normal_matrix.noalias() += predictor * predictor.transpose();
            right_hand.noalias() += predictor * world.transpose();
            ++sample_count;
        }
    }

    Vec3d world_per_x_pixel = Vec3d::Zero();
    Vec3d world_per_y_pixel = Vec3d::Zero();
    bool  fitted            = false;
    if (sample_count >= 3) {
        Eigen::FullPivLU<Eigen::Matrix3d> decomposition(normal_matrix);
        if (decomposition.rank() == 3) {
            const Eigen::Matrix3d coefficients = decomposition.solve(right_hand);
            if (coefficients.allFinite()) {
                world_per_x_pixel = coefficients.row(0).transpose();
                world_per_y_pixel = coefficients.row(1).transpose();
                fitted            = true;
            }
        }
    }

    const double fallback_mm_per_pixel = source_texture_preview_mm_per_pixel(its, bindings, preview_width, preview_height);
    Vec2d        equal_z_tangent(1.0, 0.0);
    if (fitted) {
        const Vec2d  z_gradient(world_per_x_pixel.z(), world_per_y_pixel.z());
        const double xy_scale = std::max(world_per_x_pixel.head<2>().norm(), world_per_y_pixel.head<2>().norm());
        // When Z changes by less than five percent of an in-plane pixel, the
        // mapped patch is effectively coplanar with a layer and the slicer's
        // world-XY Gaussian really can gather samples in both texture axes.
        result.gaussian_is_two_dimensional = z_gradient.norm() <= std::max(1e-7, 0.05 * xy_scale);
        if (!result.gaussian_is_two_dimensional && z_gradient.squaredNorm() > 1e-14)
            equal_z_tangent = Vec2d(z_gradient.y(), -z_gradient.x()).normalized();
    }

    const bool advance_x = std::abs(equal_z_tangent.x()) >= std::abs(equal_z_tangent.y());
    if (advance_x) {
        const double slope       = std::abs(equal_z_tangent.x()) > 1e-12 ? equal_z_tangent.y() / equal_z_tangent.x() : 0.0;
        const double end_offset  = -slope * double(preview_width - 1);
        const int    minimum_bin = int(std::floor(std::min(0.0, end_offset))) - 1;
        const int    maximum_bin = int(std::ceil(double(preview_height - 1) + std::max(0.0, end_offset))) + 1;
        result.equal_z_lines.resize(size_t(maximum_bin - minimum_bin + 1));
        // X-major insertion leaves every digital scan line in path order.
        for (unsigned int x = 0; x < preview_width; ++x) {
            if ((x & 0x1fu) == 0u && cancel.load(std::memory_order_relaxed))
                return result;
            for (unsigned int y = 0; y < preview_height; ++y) {
                const int bin = int(std::lround(double(y) - slope * double(x)));
                result.equal_z_lines[size_t(bin - minimum_bin)].emplace_back(size_t(y) * size_t(preview_width) + x);
            }
        }
        if (fitted)
            result.mm_per_step = (world_per_x_pixel + slope * world_per_y_pixel).head<2>().norm();
    } else {
        const double slope       = equal_z_tangent.x() / equal_z_tangent.y();
        const double end_offset  = -slope * double(preview_height - 1);
        const int    minimum_bin = int(std::floor(std::min(0.0, end_offset))) - 1;
        const int    maximum_bin = int(std::ceil(double(preview_width - 1) + std::max(0.0, end_offset))) + 1;
        result.equal_z_lines.resize(size_t(maximum_bin - minimum_bin + 1));
        // Y-major insertion leaves every digital scan line in path order.
        for (unsigned int y = 0; y < preview_height; ++y) {
            if ((y & 0x1fu) == 0u && cancel.load(std::memory_order_relaxed))
                return result;
            for (unsigned int x = 0; x < preview_width; ++x) {
                const int bin = int(std::lround(double(x) - slope * double(y)));
                result.equal_z_lines[size_t(bin - minimum_bin)].emplace_back(size_t(y) * size_t(preview_width) + x);
            }
        }
        if (fitted)
            result.mm_per_step = (slope * world_per_x_pixel + world_per_y_pixel).head<2>().norm();
    }
    if (!std::isfinite(result.mm_per_step) || result.mm_per_step <= 0.002)
        result.mm_per_step = fallback_mm_per_pixel;
    result.mm_per_step = std::clamp(result.mm_per_step, 0.002, 2.0);
    return result;
}

bool box_blur_weight_field(const std::vector<float>& source,
                           std::vector<float>&       destination,
                           unsigned int              width,
                           unsigned int              height,
                           size_t                    components,
                           int                       radius,
                           bool                      horizontal,
                           const std::atomic<bool>&  cancel)
{
    if (radius <= 0 || source.empty()) {
        destination = source;
        return true;
    }
    destination.resize(source.size());
    const int extent = horizontal ? int(width) : int(height);
    const int lines  = horizontal ? int(height) : int(width);
    auto index = [width, components, horizontal](int line, int position, size_t component) {
        const size_t x = size_t(horizontal ? position : line);
        const size_t y = size_t(horizontal ? line : position);
        return (y * size_t(width) + x) * components + component;
    };
    const double scale = 1. / double(2 * radius + 1);
    for (int line = 0; line < lines; ++line) {
        if ((line & 0x1f) == 0 && cancel.load(std::memory_order_relaxed))
            return false;
        for (size_t component = 0; component < components; ++component) {
            double sum = double(radius + 1) * source[index(line, 0, component)];
            for (int tap = 1; tap <= radius; ++tap)
                sum += source[index(line, std::min(tap, extent - 1), component)];
            for (int position = 0; position < extent; ++position) {
                destination[index(line, position, component)] = float(sum * scale);
                const int remove_position = std::clamp(position - radius, 0, extent - 1);
                const int add_position    = std::clamp(position + radius + 1, 0, extent - 1);
                sum += source[index(line, add_position, component)] - source[index(line, remove_position, component)];
            }
        }
    }
    return true;
}

bool box_blur_weight_lines(const std::vector<float>&              source,
                           std::vector<float>&                    destination,
                           const std::vector<std::vector<size_t>>& lines,
                           size_t                                 components,
                           int                                    radius,
                           const std::atomic<bool>&                cancel)
{
    if (radius <= 0 || source.empty()) {
        destination = source;
        return true;
    }
    destination.resize(source.size());
    const double scale = 1. / double(2 * radius + 1);
    for (size_t line_index = 0; line_index < lines.size(); ++line_index) {
        if ((line_index & 0x1fu) == 0u && cancel.load(std::memory_order_relaxed))
            return false;
        const std::vector<size_t>& line = lines[line_index];
        if (line.empty())
            continue;
        const int extent = int(line.size());
        for (size_t component = 0; component < components; ++component) {
            auto value = [&source, &line, components, component](int position) {
                return source[line[size_t(position)] * components + component];
            };
            double sum = double(radius + 1) * value(0);
            for (int tap = 1; tap <= radius; ++tap)
                sum += value(std::min(tap, extent - 1));
            for (int position = 0; position < extent; ++position) {
                destination[line[size_t(position)] * components + component] = float(sum * scale);
                const int remove_position = std::clamp(position - radius, 0, extent - 1);
                const int add_position    = std::clamp(position + radius + 1, 0, extent - 1);
                sum += value(add_position) - value(remove_position);
            }
        }
    }
    return true;
}

bool gaussian_blur_weight_field(std::vector<float>&       field,
                                unsigned int              width,
                                unsigned int              height,
                                size_t                    components,
                                double                    sigma_pixels,
                                const PreviewFilterLayout& layout,
                                const std::atomic<bool>&  cancel)
{
    if (sigma_pixels <= 0.15 || field.empty())
        return true;
    constexpr int box_count = 3;
    const double ideal_width = std::sqrt(12. * sigma_pixels * sigma_pixels / double(box_count) + 1.);
    int          lower_width = std::max(1, int(std::floor(ideal_width)));
    if ((lower_width & 1) == 0)
        --lower_width;
    const int upper_width = lower_width + 2;
    const double numerator = 12. * sigma_pixels * sigma_pixels - double(box_count * lower_width * lower_width) -
                             double(4 * box_count * lower_width + 3 * box_count);
    const int lower_count = std::clamp(int(std::lround(numerator / double(-4 * lower_width - 4))), 0, box_count);

    std::vector<float> temporary;
    for (int pass = 0; pass < box_count; ++pass) {
        const int radius = ((pass < lower_count ? lower_width : upper_width) - 1) / 2;
        if (layout.gaussian_is_two_dimensional) {
            if (!box_blur_weight_field(field, temporary, width, height, components, radius, true, cancel) ||
                !box_blur_weight_field(temporary, field, width, height, components, radius, false, cancel))
                return false;
        } else if (!box_blur_weight_lines(field, temporary, layout.equal_z_lines, components, radius, cancel)) {
            return false;
        } else {
            field.swap(temporary);
        }
    }
    return true;
}

void compact_preview_weights(std::vector<float>& field, size_t components)
{
    for (size_t offset = 0; offset < field.size(); offset += components) {
        float maximum = 0.f;
        for (size_t component = 0; component < components; ++component)
            maximum = std::max(maximum, field[offset + component]);
        if (maximum <= EPSILON)
            continue;
        for (size_t component = 0; component < components; ++component)
            field[offset + component] = std::clamp(field[offset + component] / maximum, 0.f, 1.f);
    }
}

bool triangular_preview_smoothing(std::vector<float>&       field,
                                  const PreviewFilterLayout& layout,
                                  size_t                    components,
                                  double                    radius_steps,
                                  const std::atomic<bool>&  cancel)
{
    if (radius_steps <= 0.5 || field.empty())
        return true;
    std::vector<float>  smoothed(field);
    auto range_sum = [](const std::vector<double>& values, int begin, int end) {
        return end >= begin ? values[size_t(end + 1)] - values[size_t(begin)] : 0.;
    };
    for (size_t line_index = 0; line_index < layout.equal_z_lines.size(); ++line_index) {
        if ((line_index & 0x1fu) == 0u && cancel.load(std::memory_order_relaxed))
            return false;
        const std::vector<size_t>& line = layout.equal_z_lines[line_index];
        if (line.size() < 2)
            continue;
        std::vector<double> prefix(line.size() + 1);
        std::vector<double> indexed_prefix(line.size() + 1);
        for (size_t component = 0; component < components; ++component) {
            prefix[0] = indexed_prefix[0] = 0.;
            for (size_t position = 0; position < line.size(); ++position) {
                const double inset = 1. - double(field[line[position] * components + component]);
                prefix[position + 1]         = prefix[position] + inset;
                indexed_prefix[position + 1] = indexed_prefix[position] + inset * double(position);
            }
            for (size_t position = 0; position < line.size(); ++position) {
                const int center = int(position);
                const int left   = std::max(0, int(std::ceil(double(center) - radius_steps)));
                const int right  = std::min(int(line.size()) - 1, int(std::floor(double(center) + radius_steps)));
                const double left_sum     = range_sum(prefix, left, center);
                const double left_indexed = range_sum(indexed_prefix, left, center);
                const double right_sum     = range_sum(prefix, center + 1, right);
                const double right_indexed = range_sum(indexed_prefix, center + 1, right);
                const double weighted_sum = left_sum * (1. - double(center) / radius_steps) + left_indexed / radius_steps +
                                            right_sum * (1. + double(center) / radius_steps) - right_indexed / radius_steps;
                const double left_count   = double(center - left + 1);
                const double right_count  = double(std::max(0, right - center));
                const double left_indices = 0.5 * double(left + center) * left_count;
                const double right_indices = right_count > 0. ? 0.5 * double(center + 1 + right) * right_count : 0.;
                const double weight_sum = left_count * (1. - double(center) / radius_steps) + left_indices / radius_steps +
                                          right_count * (1. + double(center) / radius_steps) - right_indices / radius_steps;
                const size_t offset = line[position] * components + component;
                const double inset  = 1. - double(field[offset]);
                const double filtered_inset = weight_sum > EPSILON ? std::min(inset, weighted_sum / weight_sum) : inset;
                smoothed[offset] = float(std::clamp(1. - filtered_inset, 0., 1.));
            }
        }
    }
    field.swap(smoothed);
    return true;
}

bool slope_limit_preview_weights(std::vector<float>&       field,
                                 const PreviewFilterLayout& layout,
                                 size_t                    components,
                                 double                    sample_spacing_mm,
                                 const std::atomic<bool>&  cancel)
{
    if (field.empty())
        return true;
    constexpr double max_displacement_mm = 0.63;
    // Slice-time limiting runs on the resampled perimeter, where every
    // sample-to-sample edge receives the 0.015 mm transition allowance. A
    // preview texel may cover several of those edges (especially in 0.02 mm
    // Ultra mode), so charging the allowance only once per texel stretches a
    // transition and makes the prepare result visibly softer than the sliced
    // path. Integrate the same allowance over the represented distance.
    const double bounded_sample_spacing = std::clamp(sample_spacing_mm, 0.02, 2.0);
    const double represented_steps      = layout.mm_per_step / bounded_sample_spacing;
    const float  limit                  = float((layout.mm_per_step * 0.35 + 0.015 * represented_steps) /
                               max_displacement_mm);
    for (int pass = 0; pass < 4; ++pass) {
        for (size_t line_index = 0; line_index < layout.equal_z_lines.size(); ++line_index) {
            if ((line_index & 0x1fu) == 0u && cancel.load(std::memory_order_relaxed))
                return false;
            const std::vector<size_t>& line = layout.equal_z_lines[line_index];
            if (line.size() < 2)
                continue;
            for (size_t component = 0; component < components; ++component) {
                for (size_t position = 1; position < line.size(); ++position) {
                    const size_t offset          = line[position] * components + component;
                    const size_t previous_offset = line[position - 1] * components + component;
                    field[offset] = std::max(field[offset], field[previous_offset] - limit);
                }
                for (size_t position = line.size() - 1; position-- > 0;) {
                    const size_t offset      = line[position] * components + component;
                    const size_t next_offset = line[position + 1] * components + component;
                    field[offset] = std::max(field[offset], field[next_offset] - limit);
                }
            }
        }
    }
    return true;
}

bool build_perimeter_modulation_texture_preview(const ImageMap::TextureAsset&                      asset,
                                                const ImageMap::Zone&                              zone,
                                                const ImageMap::ContinuousColorSolver&             solver,
                                                const indexed_triangle_set&                        its,
                                                const std::vector<const ImageMap::TriangleBinding*>& bindings,
                                                unsigned int                                       width,
                                                unsigned int                                       height,
                                                ImageMap::WrapMode                                 wrap_u,
                                                ImageMap::WrapMode                                 wrap_v,
                                                const Transform3d&                                 mesh_to_world,
                                                std::atomic<bool>&                                 cancel,
                                                const std::function<void(float)>&                   progress,
                                                std::vector<unsigned char>&                        output)
{
    const size_t components = solver.component_count();
    if (!solver.valid() || components == 0 || width == 0 || height == 0)
        return false;
    const size_t pixel_count = size_t(width) * size_t(height);
    std::vector<float> weights(pixel_count * components, 0.f);
    std::vector<float> alpha(pixel_count, 1.f);
    constexpr size_t quantized_color_count = 32u * 32u * 32u;
    std::vector<uint16_t> pixel_color_keys(pixel_count, 0);
    std::vector<uint8_t>  used_color_keys(quantized_color_count, 0);

    // Sampling is deliberately separate from physical-mixture lookup. A
    // photographic row may contain thousands of new colours; doing both in
    // one loop left progress at exactly 10% until all those KD searches had
    // completed, which looked like a hung worker.
    for (unsigned int y = 0; y < height; ++y) {
        if ((y & 0x1fu) == 0u && cancel.load(std::memory_order_relaxed))
            return false;
        if ((y & 0x07u) == 0u && progress)
            progress(0.18f * float(y) / float(height));
        for (unsigned int x = 0; x < width; ++x) {
            const Vec2f uv((float(x) + 0.5f) / float(width), (float(y) + 0.5f) / float(height));
            RGBA sampled = ImageMap::sample_processed_texture(asset, uv, wrap_u, wrap_v, zone);
            const size_t pixel = size_t(y) * size_t(width) + x;
            alpha[pixel] = sampled[3];
            for (size_t channel = 0; channel < 3; ++channel)
                sampled[channel] = sampled[channel] * sampled[3] + (1.f - sampled[3]);
            sampled[3] = 1.f;
            const RGBA target = ImageMap::adjusted_modulation_target_color(sampled, zone);
            auto quantized_channel = [&target](size_t channel) {
                return uint32_t(std::lround(std::clamp(target[channel], 0.f, 1.f) * 31.f));
            };
            const uint32_t key = (quantized_channel(0) << 10) | (quantized_channel(1) << 5) | quantized_channel(2);
            pixel_color_keys[pixel] = uint16_t(key);
            used_color_keys[key]    = 1;
        }
    }
    if (progress)
        progress(0.18f);

    std::vector<uint16_t> unique_color_keys;
    unique_color_keys.reserve(quantized_color_count);
    for (size_t key = 0; key < used_color_keys.size(); ++key)
        if (used_color_keys[key] != 0)
            unique_color_keys.emplace_back(uint16_t(key));

    std::vector<float> solved_weights(quantized_color_count * components, 0.f);
    std::atomic<size_t> next_color{0};
    std::atomic<size_t> completed_colors{0};
    const size_t worker_count = std::min<size_t>(4, unique_color_keys.size());
    std::vector<std::thread> solve_workers;
    solve_workers.reserve(worker_count);
    for (size_t worker = 0; worker < worker_count; ++worker) {
        solve_workers.emplace_back([&]() {
            for (;;) {
                const size_t color_index = next_color.fetch_add(1, std::memory_order_relaxed);
                if (color_index >= unique_color_keys.size() || cancel.load(std::memory_order_relaxed))
                    return;
                const uint32_t key   = unique_color_keys[color_index];
                const RGBA target{float((key >> 10) & 31u) / 31.f, float((key >> 5) & 31u) / 31.f,
                                  float(key & 31u) / 31.f, 1.f};
                // The shared solver retains this candidate index, making
                // later slider edits a constant-time lookup.
                std::vector<double> solved = solver.solve_quantized_5bit(target);
                ImageMap::apply_modulation_component_contrast(solved, zone);
                if (solved.size() == components) {
                    const size_t offset = size_t(key) * components;
                    for (size_t component = 0; component < components; ++component)
                        solved_weights[offset + component] = float(solved[component]);
                }
                const size_t completed = completed_colors.fetch_add(1, std::memory_order_relaxed) + 1;
                if ((completed & 0x1fu) == 0u && progress)
                    progress(0.18f + 0.30f * float(completed) / float(std::max<size_t>(unique_color_keys.size(), 1)));
            }
        });
    }
    for (std::thread& worker : solve_workers)
        worker.join();
    if (cancel.load(std::memory_order_relaxed))
        return false;
    if (progress)
        progress(0.48f);

    for (unsigned int y = 0; y < height; ++y) {
        if ((y & 0x1fu) == 0u && cancel.load(std::memory_order_relaxed))
            return false;
        if ((y & 0x0fu) == 0u && progress)
            progress(0.48f + 0.07f * float(y) / float(height));
        for (unsigned int x = 0; x < width; ++x) {
            const size_t pixel         = size_t(y) * size_t(width) + x;
            const size_t source_offset = size_t(pixel_color_keys[pixel]) * components;
            const size_t output_offset = pixel * components;
            std::copy_n(solved_weights.begin() + source_offset, components, weights.begin() + output_offset);
        }
    }
    if (progress)
        progress(0.55f);

    const PreviewFilterLayout filter_layout =
        source_texture_preview_filter_layout(its, bindings, width, height, mesh_to_world, cancel);
    if (cancel.load(std::memory_order_relaxed))
        return false;
    if (progress)
        progress(0.60f);
    const double gaussian_strength = std::clamp(double(zone.gaussian_smoothing_strength), 0., 4.);
    const double base_sigma_mm      = std::max(0.04, 0.45 * double(zone.modulation_sample_spacing_mm));
    if (!gaussian_blur_weight_field(weights, width, height, components,
                                    base_sigma_mm * gaussian_strength / filter_layout.mm_per_step, filter_layout, cancel))
        return false;
    compact_preview_weights(weights, components);
    if (progress)
        progress(0.69f);

    if (!zone.disable_broad_path_smoothing) {
        const double smoothing_base_mm = std::max(0.42, double(zone.corner_smoothing_radius_mm));
        const double first_radius_mm   = std::max(0.30, 1.15 * smoothing_base_mm) *
                                       std::clamp(double(zone.first_path_smoothing_strength), 0., 4.);
        const double second_radius_mm = std::max(0.20, 0.45 * smoothing_base_mm) *
                                        std::clamp(double(zone.second_path_smoothing_strength), 0., 4.);
        if (!triangular_preview_smoothing(weights, filter_layout, components, first_radius_mm / filter_layout.mm_per_step, cancel))
            return false;
        if (progress)
            progress(0.75f);
        if (!slope_limit_preview_weights(weights, filter_layout, components, zone.modulation_sample_spacing_mm, cancel))
            return false;
        if (progress)
            progress(0.80f);
        if (!triangular_preview_smoothing(weights, filter_layout, components, second_radius_mm / filter_layout.mm_per_step, cancel))
            return false;
    } else {
        if (!slope_limit_preview_weights(weights, filter_layout, components, zone.modulation_sample_spacing_mm, cancel))
            return false;
    }
    if (progress)
        progress(0.84f);

    output.resize(pixel_count * 4);
    std::unordered_map<uint64_t, RGBA> prediction_cache;
    prediction_cache.reserve(8192);
    std::vector<double> quantized_weights(components);
    for (size_t pixel = 0; pixel < pixel_count; ++pixel) {
        if ((pixel & 0xffffu) == 0u && cancel.load(std::memory_order_relaxed))
            return false;
        if ((pixel & 0xffffu) == 0u && progress)
            progress(0.84f + 0.16f * float(double(pixel) / double(pixel_count)));
        uint64_t key = 0;
        for (size_t component = 0; component < components; ++component) {
            const uint64_t quantized = uint64_t(std::lround(std::clamp(weights[pixel * components + component], 0.f, 1.f) * 31.f));
            quantized_weights[component] = double(quantized) / 31.;
            key = key * 33u + quantized;
        }
        auto predicted = prediction_cache.find(key);
        if (predicted == prediction_cache.end()) {
            const RGBA color = solver.predict_weights(quantized_weights).value_or(RGBA{1.f, 1.f, 1.f, 1.f});
            predicted = prediction_cache.emplace(key, color).first;
        }
        for (size_t channel = 0; channel < 3; ++channel)
            output[pixel * 4 + channel] = uint8_t(std::lround(std::clamp(predicted->second[channel], 0.f, 1.f) * 255.f));
        output[pixel * 4 + 3] = uint8_t(std::lround(std::clamp(alpha[pixel], 0.f, 1.f) * 255.f));
    }
    if (progress)
        progress(1.f);
    return true;
}

std::shared_ptr<SourceColorPreviewJob> start_source_color_preview(std::shared_ptr<const TriangleMesh>             mesh,
                                                                  std::shared_ptr<const ImageMap::VolumeData>     data,
                                                                  std::vector<ImageMap::ContinuousColorComponent> components,
                                                                  std::vector<SourceColorPreviewAssignment>       assignments,
                                                                  std::string                                     volume_name,
                                                                  ImageMap::RenderMode                            render_mode,
                                                                  bool                                            quantize_to_palette,
                                                                  bool                                            predicted_colors,
                                                                  Transform3d                                     mesh_to_world,
                                                                  bool                                            interactive_preview)
{
    auto job = std::make_shared<SourceColorPreviewJob>(data);
    std::thread([job, mesh = std::move(mesh), data = std::move(data), components = std::move(components),
                 assignments = std::move(assignments), volume_name = std::move(volume_name), render_mode, quantize_to_palette,
                 predicted_colors, mesh_to_world = std::move(mesh_to_world), interactive_preview]() mutable {
        const auto started_at = std::chrono::steady_clock::now();
        try {
            job->progress.store(2, std::memory_order_relaxed);
            const bool adaptive_cycle_preview = predicted_colors && render_mode == ImageMap::RenderMode::AdaptiveLocalizedCycles;
            quantize_to_palette                = predicted_colors && quantize_to_palette;
            auto color_model_index = [](ImageMap::ColorMixModel model) {
                return std::min<size_t>(size_t(model), size_t(ImageMap::ColorMixModel::FilamentMixer));
            };
            std::array<std::shared_ptr<const ImageMap::ContinuousColorSolver>, 2> shared_solvers;
            constexpr size_t                                                     no_solver = std::numeric_limits<size_t>::max();
            std::vector<std::shared_ptr<const ImageMap::ContinuousColorSolver>>   cycle_solvers;
            std::vector<size_t> assignment_solver_indices(assignments.size(), no_solver);
            if (adaptive_cycle_preview) {
                std::unordered_map<uint64_t, size_t> solver_by_filament;
                solver_by_filament.reserve(assignments.size());
                for (size_t assignment_index = 0; assignment_index < assignments.size(); ++assignment_index) {
                    if (job->cancel.load(std::memory_order_relaxed)) {
                        job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                        return;
                    }
                    const unsigned int filament_id = assignments[assignment_index].filament_id;
                    const ImageMap::ColorMixModel color_mix_model = assignments[assignment_index].zone_index < data->zones.size() ?
                                                                        data->zones[assignments[assignment_index].zone_index].color_mix_model :
                                                                        ImageMap::ColorMixModel::FullSpectrumKmKs;
                    const uint64_t solver_key = uint64_t(filament_id) | (uint64_t(color_model_index(color_mix_model)) << 32);
                    const auto existing = solver_by_filament.find(solver_key);
                    if (existing != solver_by_filament.end()) {
                        assignment_solver_indices[assignment_index] = existing->second;
                    } else if (assignments[assignment_index].components.size() >= 2) {
                        const size_t solver_index = cycle_solvers.size();
                        std::shared_ptr<const ImageMap::ContinuousColorSolver> solver =
                            source_color_preview_solver(assignments[assignment_index].components, color_mix_model);
                        if (solver && solver->valid()) {
                            cycle_solvers.emplace_back(std::move(solver));
                            solver_by_filament.emplace(solver_key, solver_index);
                            assignment_solver_indices[assignment_index] = solver_index;
                        }
                    }
                    job->progress.store(2 + int(8 * (assignment_index + 1) / std::max<size_t>(assignments.size(), 1)),
                                        std::memory_order_relaxed);
                }
                // Every palette target sharing a cycle reuses the same solver.
                // Store a guaranteed-attainable fallback as well, so a failed
                // lookup can never leak the original texture RGB into the
                // printable-result preview.
                for (size_t assignment_index = 0; assignment_index < assignments.size(); ++assignment_index) {
                    const size_t solver_index = assignment_solver_indices[assignment_index];
                    if (solver_index == no_solver || solver_index >= cycle_solvers.size())
                        continue;
                    if (const std::optional<RGBA> predicted = cycle_solvers[solver_index]->predict_modulation_color(
                            assignments[assignment_index].target_color))
                        assignments[assignment_index].display_color = *predicted;
                }
            } else if (!quantize_to_palette) {
                for (const ImageMap::Zone& zone : data->zones) {
                    if (job->cancel.load(std::memory_order_relaxed)) {
                        job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                        return;
                    }
                    if (!zone.enabled || zone.render_mode != render_mode)
                        continue;
                    const size_t index = color_model_index(zone.color_mix_model);
                    if (!shared_solvers[index])
                        shared_solvers[index] = source_color_preview_solver(components, zone.color_mix_model);
                }
            }
            if (job->cancel.load(std::memory_order_relaxed)) {
                job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                return;
            }
            job->progress.store(10, std::memory_order_relaxed);

            // Keep texture mappings on their original triangles and let the
            // GPU interpolate the image. Projected decals are intentionally
            // partial and use transparent UV wrapping, so they need a simple
            // background surface plus a clipped texture overlay rather than
            // the subdivided vertex-colour fallback below.
            const indexed_triangle_set&                   its = mesh->its;
            std::vector<const ImageMap::TriangleBinding*> selected(its.indices.size(), nullptr);
            for (const ImageMap::TriangleBinding& binding : data->triangle_bindings) {
                if (binding.triangle_index >= selected.size() || binding.zone_index >= data->zones.size())
                    continue;
                const ImageMap::Zone& zone = data->zones[binding.zone_index];
                if (!zone.enabled || zone.render_mode != render_mode)
                    continue;
                const ImageMap::TriangleBinding* current = selected[binding.triangle_index];
                if (current == nullptr || data->zones[current->zone_index].priority < zone.priority)
                    selected[binding.triangle_index] = &binding;
            }

            bool texture_preview_eligible       = false;
            bool texture_preview_needs_background = false;
            for (const ImageMap::TriangleBinding* binding : selected) {
                if (binding == nullptr) {
                    texture_preview_needs_background = true;
                    continue;
                }
                if (binding->source.kind != ImageMap::SourceKind::Texture || binding->source.texture_asset_index < 0 ||
                    size_t(binding->source.texture_asset_index) >= data->texture_assets.size() ||
                    !data->texture_assets[size_t(binding->source.texture_asset_index)].valid() ||
                    !std::all_of(binding->source.uvs.begin(), binding->source.uvs.end(),
                                 [](const Vec2f& uv) { return std::isfinite(uv.x()) && std::isfinite(uv.y()); })) {
                    texture_preview_eligible = false;
                    break;
                }
                texture_preview_eligible = true;
                texture_preview_needs_background |= binding->source.wrap_u == ImageMap::WrapMode::Transparent ||
                                                    binding->source.wrap_v == ImageMap::WrapMode::Transparent;
            }

            if (texture_preview_eligible) {
                using TextureGroupKey = std::tuple<int32_t, uint32_t, uint8_t, uint8_t>;
                std::map<TextureGroupKey, std::vector<const ImageMap::TriangleBinding*>> groups;
                for (const ImageMap::TriangleBinding* binding : selected) {
                    if (binding == nullptr)
                        continue;
                    groups[{binding->source.texture_asset_index, binding->zone_index, uint8_t(binding->source.wrap_u),
                            uint8_t(binding->source.wrap_v)}]
                        .push_back(binding);
                }

                std::unique_ptr<GUI::GLModel::Geometry> background_geometry;
                if (texture_preview_needs_background) {
                    background_geometry = std::make_unique<GUI::GLModel::Geometry>();
                    background_geometry->format = {GUI::GLModel::Geometry::EPrimitiveType::Triangles,
                                                   GUI::GLModel::Geometry::EVertexLayout::P3N3C4};
                    background_geometry->reserve_vertices(its.indices.size() * 3);
                    background_geometry->reserve_indices(its.indices.size() * 3);
                    background_geometry->color = ColorRGBA::WHITE();
                    unsigned int next_vertex = 0;
                    for (size_t triangle_index = 0; triangle_index < its.indices.size(); ++triangle_index) {
                        const stl_triangle_vertex_indices& indices = its.indices[triangle_index];
                        if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0 || size_t(indices[0]) >= its.vertices.size() ||
                            size_t(indices[1]) >= its.vertices.size() || size_t(indices[2]) >= its.vertices.size())
                            continue;
                        const Vec3f& p0            = its.vertices[size_t(indices[0])];
                        const Vec3f& p1            = its.vertices[size_t(indices[1])];
                        const Vec3f& p2            = its.vertices[size_t(indices[2])];
                        Vec3f        normal        = (p1 - p0).cross(p2 - p0);
                        const float  normal_length = normal.norm();
                        if (normal_length <= EPSILON)
                            continue;
                        normal /= normal_length;
                        const ImageMap::TriangleBinding* binding = selected[triangle_index];
                        const std::array<RGBA, 3> colors = binding != nullptr ? binding->source.corner_colors :
                                                                                 std::array<RGBA, 3>{RGBA{1.f, 1.f, 1.f, 1.f},
                                                                                                     RGBA{1.f, 1.f, 1.f, 1.f},
                                                                                                     RGBA{1.f, 1.f, 1.f, 1.f}};
                        background_geometry->add_vertex(p0, normal, colors[0]);
                        background_geometry->add_vertex(p1, normal, colors[1]);
                        background_geometry->add_vertex(p2, normal, colors[2]);
                        background_geometry->add_triangle(next_vertex, next_vertex + 1, next_vertex + 2);
                        next_vertex += 3;
                    }
                    if (background_geometry->is_empty())
                        background_geometry.reset();
                }

                size_t total_pixels = 0;
                for (const auto& group : groups) {
                    const auto [width, height] =
                        source_texture_preview_size(data->texture_assets[size_t(std::get<0>(group.first))], interactive_preview);
                    total_pixels += size_t(width) * size_t(height);
                }
                size_t                                processed_pixels = 0;
                std::vector<SourceTexturePreviewPart> texture_parts;
                texture_parts.reserve(groups.size());

                for (const auto& [group_key, bindings] : groups) {
                    if (job->cancel.load(std::memory_order_relaxed)) {
                        job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                        return;
                    }
                    const int32_t                 asset_index  = std::get<0>(group_key);
                    const uint32_t                zone_index   = std::get<1>(group_key);
                    const ImageMap::TextureAsset& asset        = data->texture_assets[size_t(asset_index)];
                    const ImageMap::ContinuousColorSolver* shared_solver = nullptr;
                    if (zone_index < data->zones.size()) {
                        const size_t solver_index = color_model_index(data->zones[zone_index].color_mix_model);
                        if (shared_solvers[solver_index] && shared_solvers[solver_index]->valid())
                            shared_solver = shared_solvers[solver_index].get();
                    }
                    const auto [preview_width, preview_height] = source_texture_preview_size(asset, interactive_preview);
                    if (preview_width == 0 || preview_height == 0)
                        continue;

                    SourceTexturePreviewPart part;
                    part.width            = preview_width;
                    part.height           = preview_height;
                    part.wrap_u           = ImageMap::WrapMode(std::get<2>(group_key));
                    part.wrap_v           = ImageMap::WrapMode(std::get<3>(group_key));
                    part.geometry         = std::make_unique<GUI::GLModel::Geometry>();
                    part.geometry->format = {GUI::GLModel::Geometry::EPrimitiveType::Triangles,
                                             GUI::GLModel::Geometry::EVertexLayout::P3N3T2};
                    part.geometry->reserve_vertices(bindings.size() * 3);
                    part.geometry->reserve_indices(bindings.size() * 3);
                    part.geometry->color = ColorRGBA::WHITE();

                    unsigned int next_vertex = 0;
                    for (const ImageMap::TriangleBinding* binding : bindings) {
                        if (binding->triangle_index >= its.indices.size())
                            continue;
                        const stl_triangle_vertex_indices& indices = its.indices[binding->triangle_index];
                        if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0 || size_t(indices[0]) >= its.vertices.size() ||
                            size_t(indices[1]) >= its.vertices.size() || size_t(indices[2]) >= its.vertices.size())
                            continue;
                        const Vec3f& p0            = its.vertices[size_t(indices[0])];
                        const Vec3f& p1            = its.vertices[size_t(indices[1])];
                        const Vec3f& p2            = its.vertices[size_t(indices[2])];
                        Vec3f        normal        = (p1 - p0).cross(p2 - p0);
                        const float  normal_length = normal.norm();
                        if (normal_length <= EPSILON)
                            continue;
                        normal /= normal_length;
                        const std::array<Vec2f, 3> uvs = unwrap_source_texture_uvs(binding->source);
                        part.geometry->add_vertex(p0, normal, uvs[0]);
                        part.geometry->add_vertex(p1, normal, uvs[1]);
                        part.geometry->add_vertex(p2, normal, uvs[2]);
                        part.geometry->add_triangle(next_vertex, next_vertex + 1, next_vertex + 2);
                        next_vertex += 3;
                    }
                    if (part.geometry->is_empty())
                        continue;

                    struct CachedTextureColor
                    {
                        RGBA         color{1.f, 1.f, 1.f, 1.f};
                        unsigned int filament_id{0};
                    };
                    std::unordered_map<uint32_t, CachedTextureColor> color_cache;
                    color_cache.reserve(32u * 32u * 32u);
                    part.rgba.resize(size_t(preview_width) * size_t(preview_height) * 4);

                    auto source_channel = [&asset, preview_width, preview_height](unsigned int x, unsigned int y, size_t channel) {
                        if (preview_width == asset.width && preview_height == asset.height)
                            return float(asset.rgba[(size_t(y) * size_t(asset.width) + x) * 4 + channel]);
                        const float  source_x = std::clamp((float(x) + 0.5f) * float(asset.width) / float(preview_width) - 0.5f, 0.f,
                                                           float(asset.width - 1));
                        const float  source_y = std::clamp((float(y) + 0.5f) * float(asset.height) / float(preview_height) - 0.5f, 0.f,
                                                           float(asset.height - 1));
                        const size_t x0       = size_t(std::floor(source_x));
                        const size_t y0       = size_t(std::floor(source_y));
                        const size_t x1       = std::min(x0 + 1, size_t(asset.width - 1));
                        const size_t y1       = std::min(y0 + 1, size_t(asset.height - 1));
                        const float  tx       = source_x - float(x0);
                        const float  ty       = source_y - float(y0);
                        auto         value    = [&asset, channel](size_t sx, size_t sy) {
                            return float(asset.rgba[(sy * size_t(asset.width) + sx) * 4 + channel]);
                        };
                        const float top    = value(x0, y0) + (value(x1, y0) - value(x0, y0)) * tx;
                        const float bottom = value(x0, y1) + (value(x1, y1) - value(x0, y1)) * tx;
                        return top + (bottom - top) * ty;
                    };
                    const bool raw_source_texture = assignments.empty() && (shared_solver == nullptr || !shared_solver->valid());
                    const ImageMap::Zone* preview_zone = zone_index < data->zones.size() ? &data->zones[zone_index] : nullptr;
                    if (predicted_colors && preview_zone != nullptr &&
                        preview_zone->render_mode == ImageMap::RenderMode::PerimeterModulationV2 && shared_solver != nullptr &&
                        shared_solver->valid()) {
                        const size_t part_pixels = size_t(preview_width) * size_t(preview_height);
                        auto update_part_progress = [job, processed_pixels, part_pixels, total_pixels](float part_progress) {
                            const double completed = double(processed_pixels) +
                                                     double(std::clamp(part_progress, 0.f, 1.f)) * double(part_pixels);
                            const int requested = 10 + int(88. * completed / double(std::max<size_t>(total_pixels, 1)));
                            int       current   = job->progress.load(std::memory_order_relaxed);
                            while (current < requested &&
                                   !job->progress.compare_exchange_weak(current, requested, std::memory_order_relaxed))
                                ;
                        };
                        if (build_perimeter_modulation_texture_preview(asset, *preview_zone, *shared_solver, its, bindings,
                                                                       preview_width, preview_height, part.wrap_u, part.wrap_v,
                                                                       mesh_to_world, job->cancel, update_part_progress, part.rgba)) {
                            processed_pixels += part_pixels;
                            texture_parts.emplace_back(std::move(part));
                            continue;
                        }
                        if (job->cancel.load(std::memory_order_relaxed)) {
                            job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                            return;
                        }
                    }

                    for (unsigned int y = 0; y < preview_height; ++y) {
                        if ((y & 0x1fu) == 0u) {
                            if (job->cancel.load(std::memory_order_relaxed)) {
                                job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                                return;
                            }
                            job->progress.store(10 + int(88 * processed_pixels / std::max<size_t>(total_pixels, 1)),
                                                std::memory_order_relaxed);
                        }
                        for (unsigned int x = 0; x < preview_width; ++x) {
                            RGBA source_color;
                            if (preview_zone != nullptr) {
                                const Vec2f uv((float(x) + 0.5f) / float(preview_width),
                                               (float(y) + 0.5f) / float(preview_height));
                                source_color = ImageMap::sample_processed_texture(asset, uv, part.wrap_u, part.wrap_v, *preview_zone);
                            } else {
                                source_color = {source_channel(x, y, 0) / 255.f, source_channel(x, y, 1) / 255.f,
                                                source_channel(x, y, 2) / 255.f, source_channel(x, y, 3) / 255.f};
                            }
                            const uint8_t source_r     = uint8_t(std::lround(std::clamp(source_color[0], 0.f, 1.f) * 255.f));
                            const uint8_t source_g     = uint8_t(std::lround(std::clamp(source_color[1], 0.f, 1.f) * 255.f));
                            const uint8_t source_b     = uint8_t(std::lround(std::clamp(source_color[2], 0.f, 1.f) * 255.f));
                            const float   source_alpha = source_color[3];
                            RGBA          target{(float(source_r) / 255.f) * source_alpha + (1.f - source_alpha),
                                                 (float(source_g) / 255.f) * source_alpha + (1.f - source_alpha),
                                                 (float(source_b) / 255.f) * source_alpha + (1.f - source_alpha), 1.f};
                            if (preview_zone != nullptr && preview_zone->render_mode != ImageMap::RenderMode::NormalMix)
                                target = ImageMap::adjusted_modulation_target_color(target, *preview_zone);
                            const uint32_t cache_key = (uint32_t(std::lround(target[0] * 31.f)) << 10) |
                                                       (uint32_t(std::lround(target[1] * 31.f)) << 5) |
                                                       uint32_t(std::lround(target[2] * 31.f));
                            auto cached = color_cache.find(cache_key);
                            if (cached == color_cache.end()) {
                                CachedTextureColor                  result{target, 0};
                                const SourceColorPreviewAssignment* assignment = nearest_source_color_assignment(assignments, target,
                                                                                                                 zone_index);
                                if (quantize_to_palette) {
                                    if (assignment != nullptr) {
                                        result.color       = assignment->display_color;
                                        result.filament_id = assignment->filament_id;
                                    }
                                } else if (adaptive_cycle_preview) {
                                    if (assignment != nullptr) {
                                        result.color                  = assignment->display_color;
                                        result.filament_id            = assignment->filament_id;
                                        const size_t assignment_index = size_t(assignment - assignments.data());
                                        const size_t solver_index     = assignment_index < assignment_solver_indices.size() ?
                                                                            assignment_solver_indices[assignment_index] :
                                                                            no_solver;
                                        if (solver_index != no_solver && solver_index < cycle_solvers.size() &&
                                            cycle_solvers[solver_index] != nullptr && cycle_solvers[solver_index]->valid()) {
                                            if (const std::optional<RGBA> predicted = cycle_solvers[solver_index]->predict_modulation_color(
                                                    target))
                                                result.color = *predicted;
                                        }
                                    }
                                } else if (shared_solver != nullptr && shared_solver->valid()) {
                                    if (const std::optional<RGBA> predicted = shared_solver->predict_modulation_color(target))
                                        result.color = *predicted;
                                    result.filament_id = nearest_source_color_filament(assignments, target, zone_index);
                                }
                                cached = color_cache.emplace(cache_key, result).first;
                            }
                            const size_t output_offset = (size_t(y) * size_t(preview_width) + x) * 4;
                            if (raw_source_texture) {
                                part.rgba[output_offset + 0] = source_r;
                                part.rgba[output_offset + 1] = source_g;
                                part.rgba[output_offset + 2] = source_b;
                            } else {
                                for (size_t channel = 0; channel < 3; ++channel)
                                    part.rgba[output_offset + channel] = uint8_t(
                                        std::lround(std::clamp(cached->second.color[channel], 0.f, 1.f) * 255.f));
                            }
                            part.rgba[output_offset + 3] = assignments.empty() ?
                                                                 uint8_t(std::lround(std::clamp(source_alpha, 0.f, 1.f) * 255.f)) :
                                                                 uint8_t(std::min(cached->second.filament_id, 255u));
                            ++processed_pixels;
                        }
                    }
                    texture_parts.emplace_back(std::move(part));
                }

                if (!texture_parts.empty()) {
                    {
                        std::lock_guard<std::mutex> lock(job->result_mutex);
                        job->geometry      = std::move(background_geometry);
                        job->texture_parts = std::move(texture_parts);
                    }
                    job->progress.store(99, std::memory_order_relaxed);
                    job->status.store(SourceColorPreviewStatus::Ready, std::memory_order_release);
                    const auto elapsed_ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started_at).count();
                    BOOST_LOG_TRIVIAL(info) << "Image-map UV texture preview prepared"
                                            << " volume=\"" << volume_name << "\""
                                            << " groups=" << groups.size() << " source_triangles=" << its.indices.size()
                                            << " preview_pixels=" << processed_pixels << " elapsed_ms=" << elapsed_ms;
                    return;
                }
            }

            ImageMap::SourceColorRasterizationOptions options;
            options.max_leaf_triangles    = k_source_color_preview_triangle_cap;
            options.source_data_validated = true;
            options.cancelled             = [job]() { return job->cancel.load(std::memory_order_relaxed); };
            options.progress              = [job](int progress) {
                int current = job->progress.load(std::memory_order_relaxed);
                while (current < progress && !job->progress.compare_exchange_weak(current, progress, std::memory_order_relaxed))
                    ;
            };

            ImageMap::SourceColorRasterization rasterized = ImageMap::rasterize_source_colors(*mesh, *data, render_mode,
                                                                                              RGBA{1.f, 1.f, 1.f, 1.f}, options);
            if (job->cancel.load(std::memory_order_relaxed)) {
                job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                return;
            }

            auto geometry                  = std::make_unique<GUI::GLModel::Geometry>();
            geometry->format.type          = GUI::GLModel::Geometry::EPrimitiveType::Triangles;
            geometry->format.vertex_layout = GUI::GLModel::Geometry::EVertexLayout::P3N3C4;
            geometry->reserve_vertices(rasterized.vertices.size());
            geometry->reserve_indices(rasterized.indices.size());
            geometry->indices = std::move(rasterized.indices);
            geometry->color   = ColorRGBA::WHITE();

            const size_t vertex_count  = rasterized.vertices.size();
            int          last_progress = 85;
            struct CachedPreviewColor
            {
                RGBA         color{1.f, 1.f, 1.f, 1.f};
                unsigned int filament_id{0};
            };
            std::unordered_map<uint32_t, CachedPreviewColor> color_cache;
            color_cache.reserve(std::min(vertex_count, k_source_color_preview_cache_cap));
            const ImageMap::ContinuousColorSolver* shared_solver = nullptr;
            for (const ImageMap::Zone& zone : data->zones) {
                if (!zone.enabled || zone.render_mode != render_mode)
                    continue;
                const size_t solver_index = color_model_index(zone.color_mix_model);
                if (shared_solvers[solver_index] && shared_solvers[solver_index]->valid())
                    shared_solver = shared_solvers[solver_index].get();
                break;
            }
            for (size_t vertex_idx = 0; vertex_idx < vertex_count; ++vertex_idx) {
                if ((vertex_idx & 0x3fffu) == 0u) {
                    if (job->cancel.load(std::memory_order_relaxed)) {
                        job->status.store(SourceColorPreviewStatus::Cancelled, std::memory_order_release);
                        return;
                    }
                    const int progress = vertex_count == 0 ? 99 : 85 + int(14 * vertex_idx / vertex_count);
                    if (progress != last_progress) {
                        job->progress.store(progress, std::memory_order_relaxed);
                        last_progress = progress;
                    }
                }
                const ImageMap::SourceColorVertex& vertex = rasterized.vertices[vertex_idx];
                RGBA         result_color = adaptive_cycle_preview ? RGBA{0.55f, 0.55f, 0.55f, vertex.color[3]} : vertex.color;
                unsigned int filament_id  = 0;
                if (quantize_to_palette) {
                    const uint32_t key    = preview_color_key(vertex.color);
                    const auto     cached = color_cache.find(key);
                    if (cached != color_cache.end()) {
                        result_color = cached->second.color;
                        filament_id  = cached->second.filament_id;
                    } else {
                        const RGBA quantized_target{float((key >> 16) & 0xffu) / 255.f, float((key >> 8) & 0xffu) / 255.f,
                                                    float(key & 0xffu) / 255.f, vertex.color[3]};
                        if (const SourceColorPreviewAssignment* assignment = nearest_source_color_assignment(assignments, quantized_target);
                            assignment != nullptr) {
                            result_color = assignment->display_color;
                            filament_id  = assignment->filament_id;
                        }
                        if (color_cache.size() < k_source_color_preview_cache_cap) {
                            RGBA cached_color = result_color;
                            cached_color[3]   = 1.f;
                            color_cache.emplace(key, CachedPreviewColor{cached_color, filament_id});
                        }
                    }
                } else if (adaptive_cycle_preview) {
                    const uint32_t key    = preview_color_key(vertex.color);
                    const auto     cached = color_cache.find(key);
                    if (cached != color_cache.end()) {
                        result_color = cached->second.color;
                        filament_id  = cached->second.filament_id;
                    } else {
                        const RGBA quantized_target{float((key >> 16) & 0xffu) / 255.f, float((key >> 8) & 0xffu) / 255.f,
                                                    float(key & 0xffu) / 255.f, vertex.color[3]};
                        const SourceColorPreviewAssignment* assignment = nearest_source_color_assignment(assignments, quantized_target);
                        if (assignment != nullptr) {
                            filament_id                   = assignment->filament_id;
                            result_color                  = assignment->display_color;
                            const size_t assignment_index = size_t(assignment - assignments.data());
                            const size_t solver_index     = assignment_index < assignment_solver_indices.size() ?
                                                                assignment_solver_indices[assignment_index] :
                                                                no_solver;
                            if (solver_index != no_solver && solver_index < cycle_solvers.size() &&
                                cycle_solvers[solver_index] != nullptr && cycle_solvers[solver_index]->valid()) {
                                if (const std::optional<RGBA> predicted = cycle_solvers[solver_index]->predict_modulation_color(
                                        quantized_target))
                                    result_color = *predicted;
                            }
                        }
                        if (color_cache.size() < k_source_color_preview_cache_cap) {
                            RGBA cached_color = result_color;
                            cached_color[3]   = 1.f;
                            color_cache.emplace(key, CachedPreviewColor{cached_color, filament_id});
                        }
                    }
                } else if (shared_solver != nullptr && shared_solver->valid()) {
                    const uint32_t key    = preview_color_key(vertex.color);
                    const auto     cached = color_cache.find(key);
                    if (cached != color_cache.end()) {
                        result_color = cached->second.color;
                        filament_id  = cached->second.filament_id;
                    } else {
                        RGBA quantized_target{float((key >> 16) & 0xffu) / 255.f, float((key >> 8) & 0xffu) / 255.f,
                                              float(key & 0xffu) / 255.f, vertex.color[3]};
                        if (const std::optional<RGBA> predicted = shared_solver->predict_modulation_color(quantized_target))
                            result_color = *predicted;
                        filament_id = nearest_source_color_filament(assignments, quantized_target);
                        if (color_cache.size() < k_source_color_preview_cache_cap) {
                            RGBA cached_color = result_color;
                            cached_color[3]   = 1.f;
                            color_cache.emplace(key, CachedPreviewColor{cached_color, filament_id});
                        }
                    }
                } else {
                    filament_id = nearest_source_color_filament(assignments, vertex.color);
                }
                // The source-preview shader treats alpha as a compact, transient
                // palette assignment. It restores the volume alpha before
                // lighting, so normal image-map rendering remains opaque.
                result_color[3] = assignments.empty() ? vertex.color[3] : float(std::min(filament_id, 255u)) / 255.f;
                geometry->add_vertex(vertex.position, vertex.normal, result_color);
            }

            {
                std::lock_guard<std::mutex> lock(job->result_mutex);
                job->geometry = std::move(geometry);
            }
            // Keep the 99% overlay visible for one frame before the UI thread
            // initializes and uploads the GL model.
            job->progress.store(99, std::memory_order_relaxed);
            job->status.store(SourceColorPreviewStatus::Ready, std::memory_order_release);

            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started_at)
                                        .count();
            BOOST_LOG_TRIVIAL(info) << "Image-map result preview prepared"
                                    << " volume=\"" << volume_name << "\""
                                    << " mode=\"" << (quantize_to_palette ? "quantized" : "optical") << "\""
                                    << " source_triangles=" << rasterized.source_triangle_count
                                    << " preview_triangles=" << rasterized.sampled_leaf_count << " lod_passes=" << rasterized.lod_pass_count
                                    << " elapsed_ms=" << elapsed_ms;
        } catch (const std::exception& error) {
            BOOST_LOG_TRIVIAL(error) << "KM/K-S image-map result preview failed"
                                     << " volume=\"" << volume_name << "\""
                                     << " error=\"" << error.what() << "\"";
            job->status.store(SourceColorPreviewStatus::Failed, std::memory_order_release);
        } catch (...) {
            BOOST_LOG_TRIVIAL(error) << "KM/K-S image-map result preview failed"
                                     << " volume=\"" << volume_name << "\""
                                     << " error=\"unknown exception\"";
            job->status.store(SourceColorPreviewStatus::Failed, std::memory_order_release);
        }
    }).detach();
    return job;
}

} // namespace

// Prusa Colorin (Orca FullSpectrum 0.9.18): mixed-filament gradient and image-map palette helpers (V2b).
// Prusa Colorin (Orca 0.9.18 / BBS): colour adjustment used by the image-map and mixed-filament preview.
static ColorRGBA adjust_color_for_rendering(const ColorRGBA& colors)
{
    constexpr float fully_transparent_threshold = 0.1f;
    constexpr float fully_transparent_alpha     = 0.3f;
    constexpr float full_black_threshold        = 0.2f;
    if (colors.a() < fully_transparent_threshold)
        return {1.f, 1.f, 1.f, fully_transparent_alpha};
    if (colors.r() < full_black_threshold && colors.g() < full_black_threshold && colors.b() < full_black_threshold)
        return {full_black_threshold, full_black_threshold, full_black_threshold, colors.a()};
    return colors;
}

static std::vector<ColorRGBA> mixed_filament_preview_gradient_colors(
    const MixedFilamentDefinition&     definition,
    const MixedFilamentDisplayContext& display_context)
{
    const std::vector<wxColour> preview_colors =
        GUI::build_mixed_filament_gradient_preview(definition, display_context).sampled_colors;
    std::vector<ColorRGBA> colors;
    colors.reserve(preview_colors.size());
    for (const wxColour& color : preview_colors) {
        colors.emplace_back(float(color.Red()) / 255.f,
                            float(color.Green()) / 255.f,
                            float(color.Blue()) / 255.f,
                            1.f);
    }
    return colors;
}

static ColorRGBA interpolate_preview_gradient_color(const ColorRGBA& a, const ColorRGBA& b, double local)
{
    ColorRGBA out;
    local = std::clamp(local, 0.0, 1.0);
    for (size_t channel = 0; channel < 4; ++channel)
        out[channel] = float(double(a[channel]) * (1.0 - local) + double(b[channel]) * local);
    return out;
}

static ColorRGBA sample_preview_gradient_color(const std::vector<ColorRGBA>& colors,
                                               const std::vector<double>&    positions,
                                               double                        t)
{
    if (colors.empty())
        return ColorRGBA::WHITE();
    if (colors.size() == 1)
        return colors.front();

    const size_t expected_stops = 2 * colors.size() - 1;
    if (positions.size() == expected_stops) {
        const double progress = std::clamp(t, 0.0, std::nextafter(1.0, 0.0));
        for (size_t idx = 0; idx + 1 < colors.size(); ++idx) {
            const double p_start = std::clamp(positions[2 * idx], 0.0, 1.0);
            const double p_mid   = std::clamp(positions[2 * idx + 1], p_start, 1.0);
            const double p_end   = std::clamp(positions[2 * idx + 2], p_mid, 1.0);
            if (progress > p_end && idx + 2 < colors.size())
                continue;

            double local = 0.0;
            if (p_end <= p_start + EPSILON)
                local = 0.0;
            else if (progress <= p_mid)
                local = 0.5 * std::clamp((progress - p_start) / std::max(EPSILON, p_mid - p_start), 0.0, 1.0);
            else
                local = 0.5 + 0.5 * std::clamp((progress - p_mid) / std::max(EPSILON, p_end - p_mid), 0.0, 1.0);
            return interpolate_preview_gradient_color(colors[idx], colors[idx + 1], local);
        }
        return colors.back();
    }

    const double scaled = std::clamp(t, 0.0, std::nextafter(1.0, 0.0)) * double(colors.size() - 1);
    const size_t idx    = std::min<size_t>(colors.size() - 2, size_t(std::floor(scaled)));
    const double local  = scaled - double(idx);
    return interpolate_preview_gradient_color(colors[idx], colors[idx + 1], local);
}

static bool render_preview_gradient_model(GUI::GLModel&                         model,
                                          const std::pair<size_t, size_t>&       tverts_range,
                                          GLShaderProgram*                      shader,
                                          const BoundingBoxf3&                  box,
                                          const std::array<float, 2>&           z_range,
                                          const std::vector<ColorRGBA>&         colors,
                                          const std::vector<double>&            positions,
                                          float                                 alpha)
{
    if (shader == nullptr || !model.is_initialized() || colors.size() < 2)
        return false;

    const double z_min = box.min.z();
    const double z_max = box.max.z();
    const double z_height = z_max - z_min;
    const double clip_min = std::max<double>(z_min, z_range[0]);
    const double clip_max = std::min<double>(z_max, z_range[1]);
    if (z_height <= EPSILON || clip_max <= clip_min + EPSILON)
        return false;

    bool rendered = false;
    const size_t band_count = std::clamp<size_t>(colors.size() * 16, 24, 64);
    for (size_t band_idx = 0; band_idx < band_count; ++band_idx) {
        const double band_min = z_min + z_height * double(band_idx) / double(band_count);
        const double band_max = z_min + z_height * double(band_idx + 1) / double(band_count);
        const double draw_min = std::max<double>(band_min, clip_min);
        const double draw_max = std::min<double>(band_max, clip_max);
        if (draw_max <= draw_min + EPSILON)
            continue;

        ColorRGBA band_color = adjust_color_for_rendering(sample_preview_gradient_color(colors, positions,
                                                                                       (double(band_idx) + 0.5) / double(band_count)));
        band_color.a(alpha);
        model.set_color(band_color);
        shader->set_uniform("z_range", std::array<float, 2>{float(draw_min), float(draw_max)});
        if (tverts_range == std::make_pair<size_t, size_t>(0, -1))
            model.render();
        else
            model.render(tverts_range);
        rendered = true;
    }
    return rendered;
}

static bool volume_has_surface_segmentation(const GLVolume& volume)
{
    ModelObjectPtrs& model_objects = GUI::wxGetApp().model().objects;
    if (volume.object_idx() < 0 || volume.object_idx() >= model_objects.size())
        return false;

    const ModelObject* model_object = model_objects[volume.object_idx()];
    if (model_object == nullptr || volume.volume_idx() < 0 || volume.volume_idx() >= model_object->volumes.size())
        return false;

    const ModelVolume* model_volume = model_object->volumes[volume.volume_idx()];
    return model_volume != nullptr && model_volume->is_mm_painted();
}

struct ImageMapPreviewPalette
{
    const MixedFilamentManager* manager{nullptr};
    size_t                      num_physical{0};
    size_t                      num_total{0};

    unsigned int resolve(const ImageMap::PaletteEntry& entry) const
    {
        if (manager != nullptr && entry.mixed_filament_stable_id != 0) {
            const std::optional<unsigned int> stable = manager->filament_id_from_stable_id(entry.mixed_filament_stable_id, num_physical);
            if (stable && *stable >= 1 && *stable <= num_total)
                return *stable;
        }
        return entry.fallback_filament_id >= 1 && entry.fallback_filament_id <= num_total ? entry.fallback_filament_id : 0u;
    }

    size_t signature(const ImageMap::VolumeData& data, unsigned int base_filament_id) const
    {
        size_t seed = base_filament_id;
        auto hash_combine = [&seed](size_t value) {
            seed ^= value + size_t(0x9e3779b9) + (seed << 6) + (seed >> 2);
        };
        hash_combine(num_physical);
        hash_combine(num_total);
        for (const ImageMap::Zone& zone : data.zones) {
            hash_combine(zone.enabled ? 1u : 0u);
            for (const ImageMap::PaletteEntry& entry : zone.palette)
                hash_combine(resolve(entry));
        }
        return seed;
    }
};

static ImageMapPreviewPalette image_map_preview_palette()
{
    ImageMapPreviewPalette palette;
    if (GUI::wxGetApp().preset_bundle == nullptr)
        return palette;
    palette.manager      = &GUI::wxGetApp().preset_bundle->mixed_filaments;
    palette.num_physical = size_t(std::max(GUI::wxGetApp().extruders_edited_cnt(), 0));
    palette.num_total    = palette.manager->total_filaments(palette.num_physical);
    return palette;
}

GLVolume::GLVolume(float r, float g, float b, float a)
    : m_sla_shift_z(0.0)
    , m_sinking_contours(*this)
    , m_non_manifold_edges(*this)
    // geometry_id == 0 -> invalid
    , geometry_id(std::pair<size_t, size_t>(0, 0))
    , extruder_id(0)
    , selected(false)
    , disabled(false)
    , printable(true)
    , is_active(true)
    , zoom_to_volumes(true)
    , shader_outside_printer_detection_enabled(false)
    , is_outside(false)
    , hover(HS_None)
    , is_modifier(false)
    , is_extrusion_path(false)
    , force_native_color(false)
    , force_neutral_color(false)
    , force_sinking_contours(false)
    , tverts_range(0, size_t(-1))
{
    color = { r, g, b, a };
    set_render_color(color);
}

GLVolume::~GLVolume()
{
    if (image_map_source_preview_job)
        image_map_source_preview_job->cancel.store(true, std::memory_order_relaxed);
}

std::optional<float> GLVolume::source_color_preview_progress() const
{
    if (!image_map_source_preview_job)
        return std::nullopt;
    const SourceColorPreviewStatus status = image_map_source_preview_job->status.load(std::memory_order_acquire);
    if (status != SourceColorPreviewStatus::Running && status != SourceColorPreviewStatus::Ready &&
        status != SourceColorPreviewStatus::Uploading && status != SourceColorPreviewStatus::Uploaded)
        return std::nullopt;
    return float(std::clamp(image_map_source_preview_job->progress.load(std::memory_order_relaxed), 0, 100)) / 100.f;
}

void GLVolume::set_render_color(bool force_transparent)
{
    bool outside = is_outside || (!is_modifier && is_below_printbed());

    if (force_native_color || force_neutral_color) {
        if (outside && shader_outside_printer_detection_enabled)
            set_render_color(OUTSIDE_COLOR);
        else {
            if (force_native_color)
                set_render_color(color);
            else
                set_render_color(NEUTRAL_COLOR);
        }
    }
    else {
        if (hover == HS_Deselect)
            set_render_color(HOVER_DESELECT_COLOR);
        else if (hover == HS_Select || selected) {
            const ColorRGBA rc = outside ? SELECTED_OUTSIDE_COLOR : SELECTED_COLOR;
            if (color == NEGATIVE_VOLUME_COLOR || color == PARAMETER_MODIFIER_COLOR || color == SUPPORT_BLOCKER_COLOR || color == SUPPORT_ENFORCER_COLOR)
                set_render_color(ColorRGBA(rc.r() * color.r(), rc.g() * color.g(), rc.b() * color.b(), rc.a() * color.a()));
            else
                set_render_color(rc);
        }
        else if (disabled)
            set_render_color(DISABLED_COLOR);
        else if (outside && shader_outside_printer_detection_enabled)
            set_render_color(OUTSIDE_COLOR);
        else
            set_render_color(color);
    }

    if (!printable)
        render_color = saturate(render_color, 0.25f);

    if (force_transparent)
        render_color.a(color.a());
}

ColorRGBA color_from_model_volume(const ModelVolume& model_volume)
{
    ColorRGBA color;
    if (model_volume.is_negative_volume())
        color = GLVolume::NEGATIVE_VOLUME_COLOR;
    else if (model_volume.is_modifier())
        color = GLVolume::PARAMETER_MODIFIER_COLOR;
    else if (model_volume.is_support_blocker())
        color = GLVolume::SUPPORT_BLOCKER_COLOR;
    else if (model_volume.is_support_enforcer())
        color = GLVolume::SUPPORT_ENFORCER_COLOR;

    return color;
}

Transform3d GLVolume::world_matrix() const
{
    Transform3d m = m_instance_transformation.get_matrix() * m_volume_transformation.get_matrix();
    m.translation()(2) += m_sla_shift_z;
    return m;
}

bool GLVolume::is_left_handed() const
{
    const Vec3d &m1 = m_instance_transformation.get_mirror();
    const Vec3d &m2 = m_volume_transformation.get_mirror();
    return m1.x() * m1.y() * m1.z() * m2.x() * m2.y() * m2.z() < 0.;
}

const BoundingBoxf3& GLVolume::transformed_bounding_box() const
{
    if (!m_transformed_bounding_box.has_value()) {
        const BoundingBoxf3& box = bounding_box();
        assert(box.defined || box.min.x() >= box.max.x() || box.min.y() >= box.max.y() || box.min.z() >= box.max.z());
        std::optional<BoundingBoxf3>* trans_box = const_cast<std::optional<BoundingBoxf3>*>(&m_transformed_bounding_box);
        *trans_box = box.transformed(world_matrix());
    }
    return *m_transformed_bounding_box;
}

const BoundingBoxf3& GLVolume::transformed_convex_hull_bounding_box() const
{
    if (!m_transformed_convex_hull_bounding_box.has_value()) {
        std::optional<BoundingBoxf3>* trans_box = const_cast<std::optional<BoundingBoxf3>*>(&m_transformed_convex_hull_bounding_box);
        *trans_box = transformed_convex_hull_bounding_box(world_matrix());
    }
    return *m_transformed_convex_hull_bounding_box;
}

BoundingBoxf3 GLVolume::transformed_convex_hull_bounding_box(const Transform3d &trafo) const
{
	return (m_convex_hull && ! m_convex_hull->empty()) ?
		m_convex_hull->transformed_bounding_box(trafo) :
        bounding_box().transformed(trafo);
}

BoundingBoxf3 GLVolume::transformed_non_sinking_bounding_box(const Transform3d& trafo) const
{
    return GUI::wxGetApp().plater()->model().objects[object_idx()]->volumes[volume_idx()]->mesh().transformed_bounding_box(trafo, 0.0);
}

const BoundingBoxf3& GLVolume::transformed_non_sinking_bounding_box() const
{
    if (!m_transformed_non_sinking_bounding_box.has_value()) {
        std::optional<BoundingBoxf3>* trans_box = const_cast<std::optional<BoundingBoxf3>*>(&m_transformed_non_sinking_bounding_box);
        const Transform3d& trafo = world_matrix();
        *trans_box = transformed_non_sinking_bounding_box(trafo);
    }
    return *m_transformed_non_sinking_bounding_box;
}

void GLVolume::set_range(double min_z, double max_z)
{
    this->tverts_range.first = 0;
    this->tverts_range.second = this->model.indices_count();

    if (!this->print_zs.empty()) {
        // The Z layer range is specified.
        // First test whether the Z span of this object is not out of (min_z, max_z) completely.
        if (this->print_zs.front() > max_z || this->print_zs.back() < min_z)
            this->tverts_range.second = 0;
        else {
            // Then find the lowest layer to be displayed.
            size_t i = 0;
            for (; i < this->print_zs.size() && this->print_zs[i] < min_z; ++i);
            if (i == this->print_zs.size())
                // This shall not happen.
                this->tverts_range.second = 0;
            else {
                // Remember start of the layer.
                this->tverts_range.first = this->offsets[i];
                // Some layers are above $min_z. Which?
                for (; i < this->print_zs.size() && this->print_zs[i] <= max_z; ++i);
                if (i < this->print_zs.size())
                    this->tverts_range.second = this->offsets[i];
            }
        }
    }
}

void GLVolume::render()
{
    if (!is_active)
        return;

    GLShaderProgram* shader = GUI::wxGetApp().get_current_shader();
    if (shader == nullptr)
        return;
    
    const bool is_left_handed = this->is_left_handed();

    if (is_left_handed)
        glsafe(::glFrontFace(GL_CW));
    glsafe(::glCullFace(GL_BACK));

    if (tverts_range == std::make_pair<size_t, size_t>(0, -1))
        model.render();
    else
        model.render(this->tverts_range);

    if (is_left_handed)
        glsafe(::glFrontFace(GL_CCW));
}

bool GLVolume::is_sla_support() const { return this->composite_id.volume_id == -int(slaposSupportTree); }
bool GLVolume::is_sla_pad() const { return this->composite_id.volume_id == -int(slaposPad); }

bool GLVolume::is_sinking() const
{
    if (is_modifier || GUI::wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology() == ptSLA)
        return false;
    const BoundingBoxf3& box = transformed_convex_hull_bounding_box();
    return box.min.z() < SINKING_Z_THRESHOLD && box.max.z() >= SINKING_Z_THRESHOLD;
}

bool GLVolume::is_below_printbed() const
{
    return transformed_convex_hull_bounding_box().max.z() < 0.0;
}

void GLVolume::render_sinking_contours()
{
    m_sinking_contours.render();
}

void GLVolume::render_non_manifold_edges()
{
    m_non_manifold_edges.render();
}

std::vector<int> GLVolumeCollection::load_object(
    const ModelObject*      model_object,
    int                     obj_idx,
    const std::vector<int>& instance_idxs)
{
    std::vector<int> volumes_idx;
    for (int volume_idx = 0; volume_idx < int(model_object->volumes.size()); ++volume_idx)
        for (int instance_idx : instance_idxs)
           volumes_idx.emplace_back(this->GLVolumeCollection::load_object_volume(model_object, obj_idx, volume_idx, instance_idx));
    return volumes_idx;
}

int GLVolumeCollection::load_object_volume(
    const ModelObject* model_object,
    int                obj_idx,
    int                volume_idx,
    int                instance_idx)
{
    const ModelVolume   *model_volume = model_object->volumes[volume_idx];
    const int            extruder_id  = model_volume->extruder_id();
    const ModelInstance *instance 	  = model_object->instances[instance_idx];
    std::shared_ptr<const TriangleMesh> mesh = model_volume->mesh_ptr();
    this->volumes.emplace_back(new GLVolume());
    GLVolume& v = *this->volumes.back();
    v.set_color(color_from_model_volume(*model_volume));
    // apply printable value from the instance
    v.printable = instance->printable;
#if ENABLE_SMOOTH_NORMALS
    v.model.init_from(*mesh, true);
    if (m_use_raycasters)
      v.mesh_raycaster = std::make_unique<GUI::MeshRaycaster>(mesh);
#else
    v.model.init_from(*mesh);
    if (m_use_raycasters)
      v.mesh_raycaster = std::make_unique<GUI::MeshRaycaster>(mesh);
#endif // ENABLE_SMOOTH_NORMALS
    v.composite_id = GLVolume::CompositeID(obj_idx, volume_idx, instance_idx);
    if (model_volume->is_model_part()) {
        // GLVolume will reference a convex hull from model_volume!
        v.set_convex_hull(model_volume->get_convex_hull_shared_ptr());
        if (extruder_id != -1)
            v.extruder_id = extruder_id;
    }
    v.is_modifier = !model_volume->is_model_part();
    v.shader_outside_printer_detection_enabled = model_volume->is_model_part();
    v.set_instance_transformation(instance->get_transformation());
    v.set_volume_transformation(model_volume->get_transformation());

    return int(this->volumes.size() - 1);
}

#if SLIC3R_OPENGL_ES
GLVolume* GLVolumeCollection::load_wipe_tower_preview(
    float pos_x, float pos_y, float width, float depth, const std::vector<std::pair<float, float>>& z_and_depth_pairs, float height, float cone_angle,
    float rotation_angle, bool size_unknown, float brim_width, size_t idx, TriangleMesh* out_mesh)
#else
GLVolume* GLVolumeCollection::load_wipe_tower_preview(
    float pos_x, float pos_y, float width, float depth, const std::vector<std::pair<float, float>>& z_and_depth_pairs, float height, float cone_angle,
    float rotation_angle, bool size_unknown, float brim_width, size_t idx)
#endif // SLIC3R_OPENGL_ES
{
    if (height == 0.0f)
        height = 0.1f;

    // Because the GLVolume is also used for arrangement, it must be safely larger
    // than the actual extruded tower, otherwise the arranged tower ends up out of bed.
    const float offset = 0.3f;
    pos_x -= offset;
    pos_y -= offset;
    width += 2.f * offset;
    depth += 2.f * offset;
    brim_width += offset;

    static const float brim_height = 0.2f;
//    const float scaled_brim_height = brim_height / height;

    TriangleMesh mesh;
    ColorRGBA color = ColorRGBA::DARK_YELLOW();

    // In case we don't know precise dimensions of the wipe tower yet, we'll draw
    // the box with different color with one side jagged:
    if (size_unknown) {
        color.r(0.9f);
        color.g(0.6f);

        // Too narrow tower would interfere with the teeth. The estimate is not precise anyway.
        depth = std::max(depth, 10.f);
        float min_width = 30.f;

        // We'll now create the box with jagged edge. y-coordinates of the pre-generated model
        // are shifted so that the front edge has y=0 and centerline of the back edge has y=depth:
        float out_points_idx[][3] = { { 0, -depth, 0 }, { 0, 0, 0 }, { 38.453f, 0, 0 }, { 61.547f, 0, 0 }, { 100.0f, 0, 0 }, { 100.0f, -depth, 0 }, { 55.7735f, -10.0f, 0 }, { 44.2265f, 10.0f, 0 },
            { 38.453f, 0, 1 }, { 0, 0, 1 }, { 0, -depth, 1 }, { 100.0f, -depth, 1 }, { 100.0f, 0, 1 }, { 61.547f, 0, 1 }, { 55.7735f, -10.0f, 1 }, { 44.2265f, 10.0f, 1 } };
        static constexpr const int out_facets_idx[][3] = {
            { 0, 1, 2 }, { 3, 4, 5 }, { 6, 5, 0 }, { 3, 5, 6 }, { 6, 2, 7 }, { 6, 0, 2 }, { 8, 9, 10 }, { 11, 12, 13 }, { 10, 11, 14 }, { 14, 11, 13 }, { 15, 8, 14 },
            { 8, 10, 14 }, { 3, 12, 4 }, { 3, 13, 12 }, { 6, 13, 3 }, { 6, 14, 13 }, { 7, 14, 6 }, { 7, 15, 14 }, { 2, 15, 7 }, { 2, 8, 15 }, { 1, 8, 2 }, { 1, 9, 8 },
            { 0, 9, 1 }, { 0, 10, 9 }, { 5, 10, 0 }, { 5, 11, 10 }, { 4, 11, 5 }, { 4, 12, 11 } };
        indexed_triangle_set its;
        for (int i = 0; i < 16; ++i)
            its.vertices.emplace_back(out_points_idx[i][0] / (100.f / min_width), out_points_idx[i][1] + depth, out_points_idx[i][2]);
        its.indices.reserve(28);
        for (const int* face : out_facets_idx)
            its.indices.emplace_back(face);
        TriangleMesh tooth_mesh(std::move(its));

        // We have the mesh ready. It has one tooth and width of min_width. We will now
        // append several of these together until we are close to the required width
        // of the block. Than we can scale it precisely.
        size_t n = std::max(1, int(width / min_width)); // How many shall be merged?
        for (size_t i = 0; i < n; ++i) {
            mesh.merge(tooth_mesh);
            tooth_mesh.translate(min_width, 0.f, 0.f);
        }

        mesh.scale(Vec3f(width / (n * min_width), 1.f, height)); // Scaling to proper width
    }
    else {
        for (size_t i=1; i<z_and_depth_pairs.size(); ++i) {
            TriangleMesh m = make_cube(width, z_and_depth_pairs[i-1].second, z_and_depth_pairs[i].first-z_and_depth_pairs[i-1].first);
            m.translate(0.f, -z_and_depth_pairs[i-1].second/2.f + z_and_depth_pairs[0].second/2.f, z_and_depth_pairs[i-1].first);
            mesh.merge(m);
        }
    }

    // We'll make another mesh to show the brim (fixed layer height):
    TriangleMesh brim_mesh = make_cube(width + 2.f * brim_width, depth + 2.f * brim_width, 0.2f);
    brim_mesh.translate(-brim_width, -brim_width, 0.f);
    mesh.merge(brim_mesh);

    // Now the stabilization cone and its base.
    const auto [R, scale_x] = WipeTower::get_wipe_tower_cone_base(width, height, depth, cone_angle);
    if (R > 0.) {
        TriangleMesh cone_mesh(its_make_cone(R, height));
        cone_mesh.scale(Vec3f(1.f/scale_x, 1.f, 1.f));

        TriangleMesh disk_mesh(its_make_cylinder(R, brim_height));
        disk_mesh.scale(Vec3f(1. / scale_x, 1., 1.)); // Now it matches the base, which may be elliptic.
        disk_mesh.scale(Vec3f(1.f + scale_x*brim_width/R, 1.f + brim_width/R, 1.f)); // Scale so the brim is not deformed.
        cone_mesh.merge(disk_mesh);
        cone_mesh.translate(width / 2., depth / 2., 0.);
        mesh.merge(cone_mesh);
    }

    GLVolume* result{new GLVolume(color)};
    GLVolume& v = *result;
#if SLIC3R_OPENGL_ES
    if (out_mesh != nullptr)
        *out_mesh = mesh;
#endif // SLIC3R_OPENGL_ES
    v.model.init_from(mesh);
    v.model.set_color(color);
    v.mesh_raycaster = std::make_unique<GUI::MeshRaycaster>(std::make_shared<const TriangleMesh>(mesh));
    v.set_convex_hull(mesh.convex_hull_3d());
    v.set_volume_offset(Vec3d(pos_x, pos_y, 0.0));
    v.set_volume_rotation(Vec3d(0., 0., (M_PI / 180.) * rotation_angle));
    v.composite_id = GLVolume::CompositeID(INT_MAX - idx, 0, 0);
    v.geometry_id.first = 0;
    v.geometry_id.second = wipe_tower_instance_id(idx).id;
    v.wipe_tower_bed_index = idx;
    v.shader_outside_printer_detection_enabled = !size_unknown;

    return result;
}

// Load SLA auxiliary GLVolumes (for support trees or pad).
// This function produces volumes for multiple instances in a single shot,
// as some object specific mesh conversions may be expensive.
void GLVolumeCollection::load_object_auxiliary(
    const SLAPrintObject*           print_object,
    int                             obj_idx,
    // pairs of <instance_idx, print_instance_idx>
    const std::vector<std::pair<size_t, size_t>>& instances,
    SLAPrintObjectStep              milestone,
    // Timestamp of the last change of the milestone
    size_t                          timestamp)
{
    if (print_object->get_mesh_to_print() == nullptr)
        return;
    const Transform3d mesh_trafo_inv = print_object->trafo().inverse();

    auto add_volume = [this, timestamp](int obj_idx, int inst_idx, const ModelInstance& model_instance, SLAPrintObjectStep step,
        const TriangleMesh& mesh, const ColorRGBA& color, std::optional<const TriangleMesh> convex_hull = std::nullopt) {
        if (mesh.empty())
            return;

        GLVolume& v = *this->volumes.emplace_back(new GLVolume(color));
#if ENABLE_SMOOTH_NORMALS
        v.model.init_from(mesh, true);
#else
        v.model.init_from(mesh);
        v.model.set_color(color);
        v.mesh_raycaster = std::make_unique<GUI::MeshRaycaster>(std::make_shared<const TriangleMesh>(mesh));
#endif // ENABLE_SMOOTH_NORMALS
        v.composite_id = GLVolume::CompositeID(obj_idx, -int(step), inst_idx);
        v.geometry_id = std::pair<size_t, size_t>(timestamp, model_instance.id().id);
        if (convex_hull.has_value())
            v.set_convex_hull(*convex_hull);
        v.is_modifier = false;
        v.shader_outside_printer_detection_enabled = (step == slaposSupportTree || step == slaposDrillHoles);
        v.set_instance_transformation(model_instance.get_transformation());
    };
 
    if (milestone == SLAPrintObjectStep::slaposDrillHoles) {
        if (print_object->get_parts_to_slice().size() > 1) {
            // Get the mesh.
            TriangleMesh backend_mesh;
            std::shared_ptr<const indexed_triangle_set> preview_mesh_ptr = print_object->get_mesh_to_print();
            if (preview_mesh_ptr != nullptr)
                backend_mesh = TriangleMesh(*preview_mesh_ptr);
            backend_mesh.translate(s_multiple_beds.get_bed_translation(s_multiple_beds.get_active_bed()).cast<float>());
            if (!backend_mesh.empty()) {
                backend_mesh.transform(mesh_trafo_inv);
                TriangleMesh convex_hull = backend_mesh.convex_hull_3d();
                for (const std::pair<size_t, size_t>& instance_idx : instances) {
                    const ModelInstance& model_instance = *print_object->model_object()->instances[instance_idx.first];
                    add_volume(obj_idx, (int)instance_idx.first, model_instance, slaposDrillHoles, backend_mesh, GLVolume::MODEL_COLOR[0], convex_hull);
                }
            }
        }
    }

    // Get the support mesh.
    if (milestone == SLAPrintObjectStep::slaposSupportTree) {
        TriangleMesh supports_mesh = print_object->support_mesh();
        supports_mesh.translate(s_multiple_beds.get_bed_translation(s_multiple_beds.get_active_bed()).cast<float>());
        if (!supports_mesh.empty()) {
            supports_mesh.transform(mesh_trafo_inv);
            TriangleMesh convex_hull = supports_mesh.convex_hull_3d();
            for (const std::pair<size_t, size_t>& instance_idx : instances) {
                const ModelInstance& model_instance = *print_object->model_object()->instances[instance_idx.first];
                add_volume(obj_idx, (int)instance_idx.first, model_instance, slaposSupportTree, supports_mesh, GLVolume::SLA_SUPPORT_COLOR, convex_hull);
            }
        }
    }

    // Get the pad mesh.
    if (milestone == SLAPrintObjectStep::slaposPad) {
        TriangleMesh pad_mesh = print_object->pad_mesh();
        pad_mesh.translate(s_multiple_beds.get_bed_translation(s_multiple_beds.get_active_bed()).cast<float>());
        if (!pad_mesh.empty()) {
            pad_mesh.transform(mesh_trafo_inv);
            TriangleMesh convex_hull = pad_mesh.convex_hull_3d();
            for (const std::pair<size_t, size_t>& instance_idx : instances) {
                const ModelInstance& model_instance = *print_object->model_object()->instances[instance_idx.first];
                add_volume(obj_idx, (int)instance_idx.first, model_instance, slaposPad, pad_mesh, GLVolume::SLA_PAD_COLOR, convex_hull);
            }
        }
    }
}

GLVolume* GLVolumeCollection::new_toolpath_volume(const ColorRGBA& rgba)
{
    GLVolume* out = new_nontoolpath_volume(rgba);
    out->is_extrusion_path = true;
    return out;
}

GLVolume* GLVolumeCollection::new_nontoolpath_volume(const ColorRGBA& rgba)
{
    GLVolume* out = new GLVolume(rgba);
    out->is_extrusion_path = false;
    this->volumes.emplace_back(out);
    return out;
}

GLVolumeWithIdAndZList volumes_to_render(const GLVolumePtrs& volumes, GLVolumeCollection::ERenderType type, const Transform3d& view_matrix, std::function<bool(const GLVolume&)> filter_func)
{
    GLVolumeWithIdAndZList list;
    list.reserve(volumes.size());

    for (unsigned int i = 0; i < (unsigned int)volumes.size(); ++i) {
        GLVolume* volume = volumes[i];
        bool is_transparent = volume->render_color.is_transparent();
        if (((type == GLVolumeCollection::ERenderType::Opaque && !is_transparent) ||
             (type == GLVolumeCollection::ERenderType::Transparent && is_transparent) ||
             type == GLVolumeCollection::ERenderType::All) &&
            (! filter_func || filter_func(*volume)))
            list.emplace_back(std::make_pair(volume, std::make_pair(i, 0.0)));
    }

    if (type == GLVolumeCollection::ERenderType::Transparent && list.size() > 1) {
        for (GLVolumeWithIdAndZ& volume : list) {
            volume.second.second = volume.first->bounding_box().transformed(view_matrix * volume.first->world_matrix()).max(2);
        }

        std::sort(list.begin(), list.end(),
            [](const GLVolumeWithIdAndZ& v1, const GLVolumeWithIdAndZ& v2) -> bool { return v1.second.second < v2.second.second; }
        );
    }
    else if (type == GLVolumeCollection::ERenderType::Opaque && list.size() > 1) {
        std::sort(list.begin(), list.end(),
            [](const GLVolumeWithIdAndZ& v1, const GLVolumeWithIdAndZ& v2) -> bool { return v1.first->selected && !v2.first->selected; }
        );
    }

    return list;
}

void GLVolumeCollection::render(GLVolumeCollection::ERenderType type, bool disable_cullface, const Transform3d& view_matrix, const Transform3d& projection_matrix,
    std::function<bool(const GLVolume&)> filter_func) const
{
    GLVolumeWithIdAndZList to_render = volumes_to_render(volumes, type, view_matrix, filter_func);
    if (to_render.empty())
        return;

    GLShaderProgram* curr_shader = GUI::wxGetApp().get_current_shader();
    GLShaderProgram* sink_shader = GUI::wxGetApp().get_shader("flat");
#if SLIC3R_OPENGL_ES
    GLShaderProgram* edges_shader = GUI::wxGetApp().get_shader("dashed_lines");
#else
    GLShaderProgram* edges_shader = GUI::OpenGLManager::get_gl_info().is_core_profile() ? GUI::wxGetApp().get_shader("dashed_thick_lines") : GUI::wxGetApp().get_shader("flat");
#endif // SLIC3R_OPENGL_ES
    GLShaderProgram* mmu_painted_shader = GUI::wxGetApp().get_shader("mm_gouraud");
    if (curr_shader == nullptr || sink_shader == nullptr || edges_shader == nullptr || mmu_painted_shader == nullptr)
        return;

    GLShaderProgram* shader = curr_shader;
    shader->stop_using();

    if (type == ERenderType::Transparent) {
        glsafe(::glEnable(GL_BLEND));
        glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
        glsafe(::glDepthMask(false));
    }

    glsafe(::glCullFace(GL_BACK));
    if (disable_cullface)
        glsafe(::glDisable(GL_CULL_FACE));


    // This block is here to render the pained triangles. It is not very nice, but it works.
    // There is a cache that holds the OpenGL models of the painted areas to render, one for
    // each ModelVolume. The cache is invalidated based on changes in extruder_colors,
    // default extruder idx and timestamp of the painted data. The data belonging to objects
    // // which no longer exist are removed from the cache periodically.
    const ModelObjectPtrs& model_objects = GUI::wxGetApp().model().objects;
    const std::vector<ColorRGBA> extruders_colors = GUI::wxGetApp().plater()->get_extruder_colors_from_plater_config();
    const bool is_render_as_mmu_painted_enabled = !model_objects.empty() && !extruders_colors.empty();

    if (m_mm_paint_cache.extruders_colors != extruders_colors) {
        m_mm_paint_cache.extruders_colors = extruders_colors;
        m_mm_paint_cache.volume_data.clear();
    }
    auto time_now = std::chrono::system_clock::now();




    for (GLVolumeWithIdAndZ& volume : to_render) {
        if (!volume.first->is_active)
            continue;

        const Transform3d world_matrix = volume.first->world_matrix();
        const Matrix3d world_matrix_inv_transp = world_matrix.linear().inverse().transpose();
        const Matrix3d view_normal_matrix = view_matrix.linear() * world_matrix_inv_transp;
        const int obj_idx = volume.first->object_idx();
        const int vol_idx = volume.first->volume_idx();
        const bool render_as_mmu_painted = is_render_as_mmu_painted_enabled && !volume.first->selected &&
            !volume.first->is_outside && volume.first->hover == GLVolume::HS_None && !volume.first->is_wipe_tower() && obj_idx >= 0 && vol_idx >= 0 &&
            !model_objects[obj_idx]->volumes[vol_idx]->mm_segmentation_facets.empty() &&
            type != GLVolumeCollection::ERenderType::Transparent; // to filter out shells (not very nice)
        volume.first->set_render_color(true);

        // render sinking contours of non-hovered volumes
        if (sink_shader != nullptr) {
            sink_shader->start_using();
            if (m_show_sinking_contours) {
                if (volume.first->is_sinking() && !volume.first->is_below_printbed() &&
                    volume.first->hover == GLVolume::HS_None && !volume.first->force_sinking_contours) {
                    volume.first->render_sinking_contours();
                }
            }
            sink_shader->stop_using();
        }

        if (render_as_mmu_painted && shader != mmu_painted_shader)
            shader = mmu_painted_shader;
        else if (!render_as_mmu_painted && shader != curr_shader)
            shader = curr_shader;

        if (render_as_mmu_painted) {
            shader->start_using();
            const std::array<float, 4> clp_data = { 0.0f, 0.0f, 1.0f, FLT_MAX };
            const std::array<float, 2> z_range = { -FLT_MAX, FLT_MAX };
            const bool is_left_handed = volume.first->is_left_handed();
            shader->set_uniform("volume_world_matrix", world_matrix);
            shader->set_uniform("volume_mirrored", is_left_handed);
            shader->set_uniform("clipping_plane", clp_data);
            shader->set_uniform("z_range", z_range);
            shader->set_uniform("view_model_matrix", view_matrix * world_matrix);
            shader->set_uniform("projection_matrix", projection_matrix);
            shader->set_uniform("view_normal_matrix", view_normal_matrix);

            if (is_left_handed)
                glsafe(::glFrontFace(GL_CW));

            const ModelVolume& model_volume = *model_objects[obj_idx]->volumes[vol_idx];
            const int physical_cnt          = GUI::wxGetApp().extruders_edited_cnt();
            const FullSpectrum::VirtualExtruders& virtual_extruders =
                model_objects[obj_idx]->get_model()->virtual_extruders;
            const size_t extruder_idx =
                GUI::get_extruder_color_idx(model_volume, physical_cnt, virtual_extruders);

            // This block retrieves the painted geometry from the cache or adds it to it.
            ObjectID vol_id = model_volume.id();
            auto it = m_mm_paint_cache.volume_data.find(vol_id);
            GUI::TriangleSelectorMmGui* ts = nullptr;
            uint64_t timestamp = model_volume.mm_segmentation_facets.timestamp();
            if (it == m_mm_paint_cache.volume_data.end() || it->second.extruder_id != extruder_idx || timestamp != it->second.mm_timestamp) {
                auto ts_uptr = std::make_unique<GUI::TriangleSelectorMmGui>(model_volume.mesh(), m_mm_paint_cache.extruders_colors, m_mm_paint_cache.extruders_colors[extruder_idx]);
                ts = ts_uptr.get();
                ts->deserialize(model_volume.mm_segmentation_facets.get_data(), true);
                ts->request_update_render_data();
                m_mm_paint_cache.volume_data[vol_id] = MMPaintCachePerVolume{ extruder_idx, std::move(ts_uptr), std::chrono::system_clock::now(), timestamp };
            }
            else {
                ts = it->second.triangle_selector_mm.get();
                it->second.time_used = time_now;
            }

            
            ts->render(nullptr, world_matrix);

            if (is_left_handed)
                glsafe(::glFrontFace(GL_CCW));

            shader->stop_using();
        }
        else {
            shader->start_using();
            shader->set_uniform("z_range", m_z_range);
            shader->set_uniform("clipping_plane", m_clipping_plane);
            shader->set_uniform("use_color_clip_plane", m_use_color_clip_plane);
            shader->set_uniform("color_clip_plane", m_color_clip_plane);
            shader->set_uniform("uniform_color_clip_plane_1", m_color_clip_plane_colors[0]);
            shader->set_uniform("uniform_color_clip_plane_2", m_color_clip_plane_colors[1]);
            shader->set_uniform("print_volume.type", static_cast<int>(m_print_volume.type));
            shader->set_uniform("print_volume.xy_data", m_print_volume.data);
            shader->set_uniform("print_volume.z_data", m_print_volume.zs);
            shader->set_uniform("volume_world_matrix", world_matrix);
            shader->set_uniform("slope.actived", m_slope.active && !volume.first->is_modifier && !volume.first->is_wipe_tower());
            shader->set_uniform("slope.volume_world_normal_matrix", static_cast<Matrix3f>(world_matrix_inv_transp.cast<float>()));
            shader->set_uniform("slope.normal_z", m_slope.normal_z);

#if ENABLE_ENVIRONMENT_MAP
            unsigned int environment_texture_id = GUI::wxGetApp().plater()->get_environment_texture_id();
            bool use_environment_texture = environment_texture_id > 0 && GUI::wxGetApp().app_config->get_bool("use_environment_map");
            shader->set_uniform("use_environment_tex", use_environment_texture);
            if (use_environment_texture)
                glsafe(::glBindTexture(GL_TEXTURE_2D, environment_texture_id));
#endif // ENABLE_ENVIRONMENT_MAP
            glcheck();

            volume.first->model.set_color(volume.first->render_color);
            shader->set_uniform("view_model_matrix", view_matrix * world_matrix);
            shader->set_uniform("projection_matrix", projection_matrix);
            shader->set_uniform("view_normal_matrix", view_normal_matrix);
            volume.first->render();

#if ENABLE_ENVIRONMENT_MAP
            if (use_environment_texture)
                glsafe(::glBindTexture(GL_TEXTURE_2D, 0));
#endif // ENABLE_ENVIRONMENT_MAP

            glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));
            glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
            shader->stop_using();
        }
    }


    // Purge the painted triangles cache from everything that was not used for some time.
    // Only do this occasionally (once a second).
    using namespace std::chrono_literals;
    static auto time_since_last_check = time_now;
    if (time_now - time_since_last_check > 1000ms)
        for (auto it = m_mm_paint_cache.volume_data.begin(); it != m_mm_paint_cache.volume_data.end(); ) {
            auto it_delete = it; // The iterator to the deleted element will be invalidated, the others will not.
            ++it;
            if (time_now - it_delete->second.time_used > 5000ms)
                m_mm_paint_cache.volume_data.erase(it_delete);
    }


    if (m_show_sinking_contours) {
        if (sink_shader != nullptr) {
            sink_shader->start_using();
            for (GLVolumeWithIdAndZ& volume : to_render) {
                // render sinking contours of hovered/displaced volumes
                if (volume.first->is_sinking() && !volume.first->is_below_printbed() &&
                    (volume.first->hover != GLVolume::HS_None || volume.first->force_sinking_contours)) {
                    glsafe(::glDepthFunc(GL_ALWAYS));
                    volume.first->render_sinking_contours();
                    glsafe(::glDepthFunc(GL_LESS));
                }
            }
            sink_shader->stop_using();
        }
    }

    if (edges_shader != nullptr) {
        edges_shader->start_using();
        if (m_show_non_manifold_edges && GUI::wxGetApp().app_config->get_bool("non_manifold_edges")) {
            for (GLVolumeWithIdAndZ& volume : to_render) {
                volume.first->render_non_manifold_edges();
            }
        }
        edges_shader->stop_using();
    }

    curr_shader->start_using();

    if (disable_cullface)
        glsafe(::glEnable(GL_CULL_FACE));

    if (type == ERenderType::Transparent) {
        glsafe(::glDisable(GL_BLEND));
        glsafe(::glDepthMask(true));
    }
}

void GLVolumeCollection::reset_outside_state()
{
    for (GLVolume* volume : this->volumes) {
        if (volume != nullptr)
            volume->is_outside = false;
    }
}

void GLVolumeCollection::update_colors_by_extruder(const DynamicPrintConfig* config)
{
    using ColorItem = std::pair<std::string, ColorRGB>;
    std::vector<ColorItem> colors;

    if (static_cast<PrinterTechnology>(config->opt_int("printer_technology")) == ptSLA) {
        const std::string& txt_color = config->opt_string("material_colour").empty() ? 
                                       print_config_def.get("material_colour")->get_default_value<ConfigOptionString>()->value : 
                                       config->opt_string("material_colour");
        ColorRGB rgb;
        if (decode_color(txt_color, rgb))
            colors.push_back({ txt_color, rgb });
    }
    else {
        const ConfigOptionStrings* extruders_opt = dynamic_cast<const ConfigOptionStrings*>(config->option("extruder_colour"));
        if (extruders_opt == nullptr)
            return;

        const ConfigOptionStrings* filamemts_opt = dynamic_cast<const ConfigOptionStrings*>(config->option("filament_colour"));
        if (filamemts_opt == nullptr)
            return;

        size_t colors_count = std::max(extruders_opt->values.size(), filamemts_opt->values.size());
        if (colors_count == 0)
            return;
        colors.resize(colors_count);

        for (unsigned int i = 0; i < colors_count; ++i) {
            const std::string& ext_color = config->opt_string("extruder_colour", i);
            ColorRGB rgb;
            if (decode_color(ext_color, rgb))
                colors[i] = { ext_color, rgb };
            else {
                const std::string& fil_color = config->opt_string("filament_colour", i);
                if (decode_color(fil_color, rgb))
                    colors[i] = { fil_color, rgb };
            }
        }
    }

    const size_t physical_cnt = colors.size();
    const FullSpectrum::VirtualExtruders& virtual_extruders =
        GUI::wxGetApp().plater()->model().virtual_extruders;
    if (!virtual_extruders.empty()) {
        std::vector<std::string> phys_color_strs;
        phys_color_strs.reserve(physical_cnt);
        for (const ColorItem& c : colors) {
            phys_color_strs.push_back(c.first);
        }

        for (const FullSpectrum::VirtualExtruder& ve : virtual_extruders) {
            const std::string eff = ve.effective_color(phys_color_strs);
            ColorRGB rgb;
            if (!eff.empty() && decode_color(eff, rgb)) {
                colors.push_back({eff, rgb});
            } else {
                colors.push_back({"#808080", {0.5f, 0.5f, 0.5f}});
            }
        }
    }

    for (GLVolume* volume : volumes) {
        if (volume == nullptr || volume->is_modifier || volume->is_wipe_tower() || volume->is_sla_pad() || volume->is_sla_support())
            continue;

        const int ext_id = volume->extruder_id;
        size_t color_idx = 0;
        if (ext_id > 0 && size_t(ext_id) <= physical_cnt) {
            color_idx = size_t(ext_id - 1);
        } else if (ext_id > 0) {
            for (size_t i = 0; i < virtual_extruders.size(); ++i) {
                if (int(virtual_extruders[i].id) == ext_id) {
                    color_idx = physical_cnt + i;
                    break;
                }
            }
        }

        if (color_idx < colors.size() && !colors[color_idx].first.empty()) {
            volume->color = to_rgba(colors[color_idx].second, volume->color.a());
        }
    }
}

std::vector<double> GLVolumeCollection::get_current_print_zs(bool active_only) const
{
    // Collect layer top positions of all volumes.
    std::vector<double> print_zs;
    for (GLVolume *vol : this->volumes)
    {
        if (!active_only || vol->is_active)
            append(print_zs, vol->print_zs);
    }
    std::sort(print_zs.begin(), print_zs.end());

    // Replace intervals of layers with similar top positions with their average value.
    int n = int(print_zs.size());
    int k = 0;
    for (int i = 0; i < n;) {
        int j = i + 1;
        coordf_t zmax = print_zs[i] + EPSILON;
        for (; j < n && print_zs[j] <= zmax; ++ j) ;
        print_zs[k ++] = (j > i + 1) ? (0.5 * (print_zs[i] + print_zs[j - 1])) : print_zs[i];
        i = j;
    }
    if (k < n)
        print_zs.erase(print_zs.begin() + k, print_zs.end());

    return print_zs;
}

size_t GLVolumeCollection::cpu_memory_used() const 
{
	size_t memsize = sizeof(*this) + this->volumes.capacity() * sizeof(GLVolume);
	for (const GLVolume *volume : this->volumes)
		memsize += volume->cpu_memory_used();
	return memsize;
}

size_t GLVolumeCollection::gpu_memory_used() const 
{
	size_t memsize = 0;
	for (const GLVolume *volume : this->volumes)
		memsize += volume->gpu_memory_used();
	return memsize;
}

std::string GLVolumeCollection::log_memory_info() const 
{ 
	return " (GLVolumeCollection RAM: " + format_memsize_MB(this->cpu_memory_used()) + " GPU: " + format_memsize_MB(this->gpu_memory_used()) + " Both: " + format_memsize_MB(this->gpu_memory_used()) + ")";
}

static void thick_lines_to_geometry(
    const Lines&               lines,
    const std::vector<double>& widths,
    const std::vector<double>& heights,
    bool                       closed,
    double                     top_z,
    GUI::GLModel::Geometry&    geometry)
{
    assert(!lines.empty());
    if (lines.empty())
        return;

    enum Direction : unsigned char
    {
        Left,
        Right,
        Top,
        Bottom
    };

    // right, left, top, bottom
    std::array<int, 4> idx_prev    = { -1, -1, -1, -1 };
    std::array<int, 4> idx_initial = { -1, -1, -1, -1 };

    double bottom_z_prev = 0.0;
    Vec2d  b1_prev(Vec2d::Zero());
    Vec2d  v_prev(Vec2d::Zero());
    double len_prev = 0.0;
    double width_initial = 0.0;
    double bottom_z_initial = 0.0;

    // Reserve for a smooth path. Likley the path will not be that smooth, but better than nothing.
    // Allocated 1.5x more data than minimum.
    // Number of indices, not triangles.
    geometry.reserve_more_indices((lines.size() * 8 * 3) * 3 / 2);
    // Number of vertices, not floats.
    geometry.reserve_more_vertices(((lines.size() + 1) * 4) * 3 / 2);

    // loop once more in case of closed loops
    const size_t lines_end = closed ? (lines.size() + 1) : lines.size();
    for (size_t ii = 0; ii < lines_end; ++ii) {
        const size_t i = (ii == lines.size()) ? 0 : ii;
        const Line& line = lines[i];
        const double bottom_z = top_z - heights[i];
        const double middle_z = 0.5 * (top_z + bottom_z);
        const double width = widths[i];

        const bool is_first = (ii == 0);
        const bool is_last = (ii == lines_end - 1);
        const bool is_closing = closed && is_last;

        const Vec2d v = unscale(line.vector()).normalized();
        const double len = unscale<double>(line.length());

        const Vec2d a = unscale(line.a);
        const Vec2d b = unscale(line.b);
        Vec2d a1 = a;
        Vec2d a2 = a;
        Vec2d b1 = b;
        Vec2d b2 = b;
        {
            const double dist = 0.5 * width;  // scaled
            const double dx = dist * v.x();
            const double dy = dist * v.y();
            a1 += Vec2d(+dy, -dx);
            a2 += Vec2d(-dy, +dx);
            b1 += Vec2d(+dy, -dx);
            b2 += Vec2d(-dy, +dx);
        }

        // calculate new XY normals
        const Vec2d xy_right_normal = unscale(line.normal()).normalized();

        std::array<int, 4> idx_a = { 0, 0, 0, 0 };
        std::array<int, 4> idx_b = { 0, 0, 0, 0 };
        int idx_last = int(geometry.vertices_count());

        const bool bottom_z_different = bottom_z_prev != bottom_z;
        bottom_z_prev = bottom_z;

        if (!is_first && bottom_z_different) {
            // Found a change of the layer thickness -> Add a cap at the end of the previous segment.
            geometry.add_triangle(idx_b[Bottom], idx_b[Left], idx_b[Top]);
            geometry.add_triangle(idx_b[Bottom], idx_b[Top], idx_b[Right]);
        }

        // Share top / bottom vertices if possible.
        if (is_first) {
            idx_a[Top] = idx_last++;
            geometry.add_vertex(Vec3f(a.x(), a.y(), top_z), Vec3f(0.0f, 0.0f, 1.0f));
        }
        else
            idx_a[Top] = idx_prev[Top];

        if (is_first || bottom_z_different) {
            // Start of the 1st line segment or a change of the layer thickness while maintaining the print_z.
            idx_a[Bottom] = idx_last++;
            geometry.add_vertex(Vec3f(a.x(), a.y(), bottom_z), Vec3f(0.0f, 0.0f, -1.0f));
            idx_a[Left] = idx_last++;
            geometry.add_vertex(Vec3f(a2.x(), a2.y(), middle_z), Vec3f(-xy_right_normal.x(), -xy_right_normal.y(), 0.0f));
            idx_a[Right] = idx_last++;
            geometry.add_vertex(Vec3f(a1.x(), a1.y(), middle_z), Vec3f(xy_right_normal.x(), xy_right_normal.y(), 0.0f));
        }
        else
            idx_a[Bottom] = idx_prev[Bottom];

        if (is_first) {
            // Start of the 1st line segment.
            width_initial = width;
            bottom_z_initial = bottom_z;
            idx_initial = idx_a;
        }
        else {
            // Continuing a previous segment.
            // Share left / right vertices if possible.
            const double v_dot = v_prev.dot(v);
            // To reduce gpu memory usage, we try to reuse vertices
            // To reduce the visual artifacts, due to averaged normals, we allow to reuse vertices only when any of two adjacent edges 
            // is longer than a fixed threshold.
            // The following value is arbitrary, it comes from tests made on a bunch of models showing the visual artifacts
            const double len_threshold = 2.5;

            // Generate new vertices if the angle between adjacent edges is greater than 45 degrees or thresholds conditions are met
            const bool sharp = (v_dot < 0.707) || (len_prev > len_threshold) || (len > len_threshold);
            if (sharp) {
                if (!bottom_z_different) {
                    // Allocate new left / right points for the start of this segment as these points will receive their own normals to indicate a sharp turn.
                    idx_a[Right] = idx_last++;
                    geometry.add_vertex(Vec3f(a1.x(), a1.y(), middle_z), Vec3f(xy_right_normal.x(), xy_right_normal.y(), 0.0f));
                    idx_a[Left] = idx_last++;
                    geometry.add_vertex(Vec3f(a2.x(), a2.y(), middle_z), Vec3f(-xy_right_normal.x(), -xy_right_normal.y(), 0.0f));
                    if (cross2(v_prev, v) > 0.0) {
                        // Right turn. Fill in the right turn wedge.
                        geometry.add_triangle(idx_prev[Right], idx_a[Right], idx_prev[Top]);
                        geometry.add_triangle(idx_prev[Right], idx_prev[Bottom], idx_a[Right]);
                    }
                    else {
                        // Left turn. Fill in the left turn wedge.
                        geometry.add_triangle(idx_prev[Left], idx_prev[Top], idx_a[Left]);
                        geometry.add_triangle(idx_prev[Left], idx_a[Left], idx_prev[Bottom]);
                    }
                }
            }
            else {
                if (!bottom_z_different) {
                    // The two successive segments are nearly collinear.
                    idx_a[Left]  = idx_prev[Left];
                    idx_a[Right] = idx_prev[Right];
                }
            }
            if (is_closing) {
                if (!sharp) {
                    if (!bottom_z_different) {
                        // Closing a loop with smooth transition. Unify the closing left / right vertices.
                        geometry.set_vertex(idx_initial[Left], geometry.extract_position_3(idx_prev[Left]), geometry.extract_normal_3(idx_prev[Left]));
                        geometry.set_vertex(idx_initial[Right], geometry.extract_position_3(idx_prev[Right]), geometry.extract_normal_3(idx_prev[Right]));
                        geometry.remove_vertex(geometry.vertices_count() - 1);
                        geometry.remove_vertex(geometry.vertices_count() - 1);
                        // Replace the left / right vertex indices to point to the start of the loop.
                        const size_t indices_count = geometry.indices_count();
                        for (size_t u = indices_count - 24; u < indices_count; ++u) {
                            const unsigned int id = geometry.extract_index(u);
                            if (id == (unsigned int)idx_prev[Left])
                                geometry.set_index(u, (unsigned int)idx_initial[Left]);
                            else if (id == (unsigned int)idx_prev[Right])
                                geometry.set_index(u, (unsigned int)idx_initial[Right]);
                        }
                    }
                }
                // This is the last iteration, only required to solve the transition.
                break;
            }
        }

        // Only new allocate top / bottom vertices, if not closing a loop.
        if (is_closing)
            idx_b[Top] = idx_initial[Top];
        else {
            idx_b[Top] = idx_last++;
            geometry.add_vertex(Vec3f(b.x(), b.y(), top_z), Vec3f(0.0f, 0.0f, 1.0f));
        }

        if (is_closing && width == width_initial && bottom_z == bottom_z_initial)
            idx_b[Bottom] = idx_initial[Bottom];
        else {
            idx_b[Bottom] = idx_last++;
            geometry.add_vertex(Vec3f(b.x(), b.y(), bottom_z), Vec3f(0.0f, 0.0f, -1.0f));
        }
        // Generate new vertices for the end of this line segment.
        idx_b[Left] = idx_last++;
        geometry.add_vertex(Vec3f(b2.x(), b2.y(), middle_z), Vec3f(-xy_right_normal.x(), -xy_right_normal.y(), 0.0f));
        idx_b[Right] = idx_last++;
        geometry.add_vertex(Vec3f(b1.x(), b1.y(), middle_z), Vec3f(xy_right_normal.x(), xy_right_normal.y(), 0.0f));

        idx_prev = idx_b;
        bottom_z_prev = bottom_z;
        b1_prev = b1;
        v_prev = v;
        len_prev = len;

        if (bottom_z_different && (closed || (!is_first && !is_last))) {
            // Found a change of the layer thickness -> Add a cap at the beginning of this segment.
            geometry.add_triangle(idx_a[Bottom], idx_a[Right], idx_a[Top]);
            geometry.add_triangle(idx_a[Bottom], idx_a[Top], idx_a[Left]);
        }

        if (!closed) {
            // Terminate open paths with caps.
            if (is_first) {
                geometry.add_triangle(idx_a[Bottom], idx_a[Right], idx_a[Top]);
                geometry.add_triangle(idx_a[Bottom], idx_a[Top], idx_a[Left]);
            }
            // We don't use 'else' because both cases are true if we have only one line.
            if (is_last) {
                geometry.add_triangle(idx_b[Bottom], idx_b[Left], idx_b[Top]);
                geometry.add_triangle(idx_b[Bottom], idx_b[Top], idx_b[Right]);
            }
        }

        // Add quads for a straight hollow tube-like segment.
        // bottom-right face
        geometry.add_triangle(idx_a[Bottom], idx_b[Bottom], idx_b[Right]);
        geometry.add_triangle(idx_a[Bottom], idx_b[Right], idx_a[Right]);
        // top-right face
        geometry.add_triangle(idx_a[Right], idx_b[Right], idx_b[Top]);
        geometry.add_triangle(idx_a[Right], idx_b[Top], idx_a[Top]);
        // top-left face
        geometry.add_triangle(idx_a[Top], idx_b[Top], idx_b[Left]);
        geometry.add_triangle(idx_a[Top], idx_b[Left], idx_a[Left]);
        // bottom-left face
        geometry.add_triangle(idx_a[Left], idx_b[Left], idx_b[Bottom]);
        geometry.add_triangle(idx_a[Left], idx_b[Bottom], idx_a[Bottom]);
    }
}

// caller is responsible for supplying NO lines with zero length
static void thick_lines_to_geometry(
    const Lines3&              lines,
    const std::vector<double>& widths,
    const std::vector<double>& heights,
    bool                       closed,
    GUI::GLModel::Geometry&    geometry)
{
    assert(!lines.empty());
    if (lines.empty())
        return;

    enum Direction : unsigned char
    {
        Left,
        Right,
        Top,
        Bottom
    };

    // left, right, top, bottom
    std::array<int, 4> idx_prev    = { -1, -1, -1, -1 };
    std::array<int, 4> idx_initial = { -1, -1, -1, -1 };

    double z_prev = 0.0;
    double len_prev = 0.0;
    Vec3d  n_right_prev = Vec3d::Zero();
    Vec3d  n_top_prev = Vec3d::Zero();
    Vec3d  unit_v_prev = Vec3d::Zero();
    double width_initial = 0.0;

    // new vertices around the line endpoints
    // left, right, top, bottom
    std::array<Vec3d, 4> a = { Vec3d::Zero(), Vec3d::Zero(), Vec3d::Zero(), Vec3d::Zero() };
    std::array<Vec3d, 4> b = { Vec3d::Zero(), Vec3d::Zero(), Vec3d::Zero(), Vec3d::Zero() };

    // loop once more in case of closed loops
    const size_t lines_end = closed ? (lines.size() + 1) : lines.size();
    for (size_t ii = 0; ii < lines_end; ++ii) {
        const size_t i = (ii == lines.size()) ? 0 : ii;

        const Line3& line = lines[i];
        const double height = heights[i];
        const double width = widths[i];

        const Vec3d unit_v = unscale(line.vector()).normalized();
        const double len = unscale<double>(line.length());

        Vec3d n_top = Vec3d::Zero();
        Vec3d n_right = Vec3d::Zero();

        if (line.a.x() == line.b.x() && line.a.y() == line.b.y()) {
            // vertical segment
            n_top = Vec3d::UnitY();
            n_right = Vec3d::UnitX();
            if (line.a.z() < line.b.z())
                n_right = -n_right;
        }
        else {
            // horizontal segment
            n_right = unit_v.cross(Vec3d::UnitZ()).normalized();
            n_top = n_right.cross(unit_v).normalized();
        }

        const Vec3d rl_displacement = 0.5 * width * n_right;
        const Vec3d tb_displacement = 0.5 * height * n_top;
        const Vec3d l_a = unscale(line.a);
        const Vec3d l_b = unscale(line.b);

        a[Right]  = l_a + rl_displacement;
        a[Left]   = l_a - rl_displacement;
        a[Top]    = l_a + tb_displacement;
        a[Bottom] = l_a - tb_displacement;
        b[Right]  = l_b + rl_displacement;
        b[Left]   = l_b - rl_displacement;
        b[Top]    = l_b + tb_displacement;
        b[Bottom] = l_b - tb_displacement;

        const Vec3d n_bottom = -n_top;
        const Vec3d n_left = -n_right;

        std::array<int, 4> idx_a = { 0, 0, 0, 0};
        std::array<int, 4> idx_b = { 0, 0, 0, 0 };
        int idx_last = int(geometry.vertices_count());

        const bool z_different = (z_prev != l_a.z());
        z_prev = l_b.z();

        // Share top / bottom vertices if possible.
        if (ii == 0) {
            idx_a[Top] = idx_last++;
            geometry.add_vertex((Vec3f)a[Top].cast<float>(), (Vec3f)n_top.cast<float>());
        }
        else
            idx_a[Top] = idx_prev[Top];

        if (ii == 0 || z_different) {
            // Start of the 1st line segment or a change of the layer thickness while maintaining the print_z.
            idx_a[Bottom] = idx_last++;
            geometry.add_vertex((Vec3f)a[Bottom].cast<float>(), (Vec3f)n_bottom.cast<float>());
            idx_a[Left] = idx_last++;
            geometry.add_vertex((Vec3f)a[Left].cast<float>(), (Vec3f)n_left.cast<float>());
            idx_a[Right] = idx_last++;
            geometry.add_vertex((Vec3f)a[Right].cast<float>(), (Vec3f)n_right.cast<float>());
        }
        else
            idx_a[Bottom] = idx_prev[Bottom];

        if (ii == 0) {
            // Start of the 1st line segment.
            width_initial = width;
            idx_initial =  idx_a;
        }
        else {
            // Continuing a previous segment.
            // Share left / right vertices if possible.
            const double v_dot = unit_v_prev.dot(unit_v);
            const bool is_right_turn = n_top_prev.dot(unit_v_prev.cross(unit_v)) > 0.0;

            // To reduce gpu memory usage, we try to reuse vertices
            // To reduce the visual artifacts, due to averaged normals, we allow to reuse vertices only when any of two adjacent edges 
            // is longer than a fixed threshold.
            // The following value is arbitrary, it comes from tests made on a bunch of models showing the visual artifacts
            const double len_threshold = 2.5;

            // Generate new vertices if the angle between adjacent edges is greater than 45 degrees or thresholds conditions are met
            const bool is_sharp = v_dot < 0.707 || len_prev > len_threshold || len > len_threshold;
            if (is_sharp) {
                // Allocate new left / right points for the start of this segment as these points will receive their own normals to indicate a sharp turn.
                idx_a[Right] = idx_last++;
                geometry.add_vertex((Vec3f)a[Right].cast<float>(), (Vec3f)n_right.cast<float>());
                idx_a[Left] = idx_last++;
                geometry.add_vertex((Vec3f)a[Left].cast<float>(), (Vec3f)n_left.cast<float>());

                if (is_right_turn) {
                    // Right turn. Fill in the right turn wedge.
                    geometry.add_triangle(idx_prev[Right], idx_a[Right], idx_prev[Top]);
                    geometry.add_triangle(idx_prev[Right], idx_prev[Bottom], idx_a[Right]);
                }
                else {
                    // Left turn. Fill in the left turn wedge.
                    geometry.add_triangle(idx_prev[Left], idx_prev[Top], idx_a[Left]);
                    geometry.add_triangle(idx_prev[Left], idx_a[Left], idx_prev[Bottom]);
                }
            }
            else {
                // The two successive segments are nearly collinear.
                idx_a[Left] = idx_prev[Left];
                idx_a[Right] = idx_prev[Right];
            }

            if (ii == lines.size()) {
                if (!is_sharp) {
                    // Closing a loop with smooth transition. Unify the closing left / right vertices.
                    geometry.set_vertex(idx_initial[Left], geometry.extract_position_3(idx_prev[Left]), geometry.extract_normal_3(idx_prev[Left]));
                    geometry.set_vertex(idx_initial[Right], geometry.extract_position_3(idx_prev[Right]), geometry.extract_normal_3(idx_prev[Right]));
                    geometry.remove_vertex(geometry.vertices_count() - 1);
                    geometry.remove_vertex(geometry.vertices_count() - 1);
                    // Replace the left / right vertex indices to point to the start of the loop.
                    const size_t indices_count = geometry.indices_count();
                    for (size_t u = indices_count - 24; u < indices_count; ++u) {
                        const unsigned int id = geometry.extract_index(u);
                        if (id == (unsigned int)idx_prev[Left])
                            geometry.set_index(u, (unsigned int)idx_initial[Left]);
                        else if (id == (unsigned int)idx_prev[Right])
                            geometry.set_index(u, (unsigned int)idx_initial[Right]);
                    }
                }

                // This is the last iteration, only required to solve the transition.
                break;
            }
        }

        // Only new allocate top / bottom vertices, if not closing a loop.
        if (closed && ii + 1 == lines.size())
            idx_b[Top] = idx_initial[Top];
        else {
            idx_b[Top] = idx_last++;
            geometry.add_vertex((Vec3f)b[Top].cast<float>(), (Vec3f)n_top.cast<float>());
        }

        if (closed && ii + 1 == lines.size() && width == width_initial)
            idx_b[Bottom] = idx_initial[Bottom];
        else {
            idx_b[Bottom] = idx_last++;
            geometry.add_vertex((Vec3f)b[Bottom].cast<float>(), (Vec3f)n_bottom.cast<float>());
        }

        // Generate new vertices for the end of this line segment.
        idx_b[Left] = idx_last++;
        geometry.add_vertex((Vec3f)b[Left].cast<float>(), (Vec3f)n_left.cast<float>());
        idx_b[Right] = idx_last++;
        geometry.add_vertex((Vec3f)b[Right].cast<float>(), (Vec3f)n_right.cast<float>());

        idx_prev = idx_b;
        n_right_prev = n_right;
        n_top_prev = n_top;
        unit_v_prev = unit_v;
        len_prev = len;

        if (!closed) {
            // Terminate open paths with caps.
            if (i == 0) {
                geometry.add_triangle(idx_a[Bottom], idx_a[Right], idx_a[Top]);
                geometry.add_triangle(idx_a[Bottom], idx_a[Top], idx_a[Left]);
            }

            // We don't use 'else' because both cases are true if we have only one line.
            if (i + 1 == lines.size()) {
                geometry.add_triangle(idx_b[Bottom], idx_b[Left], idx_b[Top]);
                geometry.add_triangle(idx_b[Bottom], idx_b[Top], idx_b[Right]);
            }
        }

        // Add quads for a straight hollow tube-like segment.
        // bottom-right face
        geometry.add_triangle(idx_a[Bottom], idx_b[Bottom], idx_b[Right]);
        geometry.add_triangle(idx_a[Bottom], idx_b[Right], idx_a[Right]);
        // top-right face
        geometry.add_triangle(idx_a[Right], idx_b[Right], idx_b[Top]);
        geometry.add_triangle(idx_a[Right], idx_b[Top], idx_a[Top]);
        // top-left face
        geometry.add_triangle(idx_a[Top], idx_b[Top], idx_b[Left]);
        geometry.add_triangle(idx_a[Top], idx_b[Left], idx_a[Left]);
        // bottom-left face
        geometry.add_triangle(idx_a[Left], idx_b[Left], idx_b[Bottom]);
        geometry.add_triangle(idx_a[Left], idx_b[Bottom], idx_a[Bottom]);
    }
}

void _3DScene::thick_lines_to_verts(
    const Lines&               lines,
    const std::vector<double>& widths,
    const std::vector<double>& heights,
    bool                       closed,
    double                     top_z,
    GUI::GLModel::Geometry&    geometry)
{
    thick_lines_to_geometry(lines, widths, heights, closed, top_z, geometry);
}

void _3DScene::thick_lines_to_verts(
    const Lines3&              lines,
    const std::vector<double>& widths,
    const std::vector<double>& heights,
    bool                       closed,
    GUI::GLModel::Geometry&    geometry)
{
    thick_lines_to_geometry(lines, widths, heights, closed, geometry);
}

// Fill in the qverts and tverts with quads and triangles for the extrusion_path.
void _3DScene::extrusionentity_to_verts(const ExtrusionPath& extrusion_path, float print_z, const Point& copy, GUI::GLModel::Geometry& geometry)
{
    Polyline            polyline = extrusion_path.polyline;
    polyline.remove_duplicate_points();
    polyline.translate(copy);
    const Lines               lines = polyline.lines();
    std::vector<double> widths(lines.size(), extrusion_path.width());
    std::vector<double> heights(lines.size(), extrusion_path.height());
    thick_lines_to_verts(lines, widths, heights, false, print_z, geometry);
}

// Fill in the qverts and tverts with quads and triangles for the extrusion_loop.
void _3DScene::extrusionentity_to_verts(const ExtrusionLoop& extrusion_loop, float print_z, const Point& copy, GUI::GLModel::Geometry& geometry)
{
    Lines               lines;
    std::vector<double> widths;
    std::vector<double> heights;
    for (const ExtrusionPath& extrusion_path : extrusion_loop.paths) {
        Polyline            polyline = extrusion_path.polyline;
        polyline.remove_duplicate_points();
        polyline.translate(copy);
        const Lines lines_this = polyline.lines();
        append(lines, lines_this);
        widths.insert(widths.end(), lines_this.size(), extrusion_path.width());
        heights.insert(heights.end(), lines_this.size(), extrusion_path.height());
    }
    thick_lines_to_verts(lines, widths, heights, true, print_z, geometry);
}

// Fill in the qverts and tverts with quads and triangles for the extrusion_multi_path.
void _3DScene::extrusionentity_to_verts(const ExtrusionMultiPath& extrusion_multi_path, float print_z, const Point& copy, GUI::GLModel::Geometry& geometry)
{
    Lines               lines;
    std::vector<double> widths;
    std::vector<double> heights;
    for (const ExtrusionPath& extrusion_path : extrusion_multi_path.paths) {
        Polyline            polyline = extrusion_path.polyline;
        polyline.remove_duplicate_points();
        polyline.translate(copy);
        const Lines lines_this = polyline.lines();
        append(lines, lines_this);
        widths.insert(widths.end(), lines_this.size(), extrusion_path.width());
        heights.insert(heights.end(), lines_this.size(), extrusion_path.height());
    }
    thick_lines_to_verts(lines, widths, heights, false, print_z, geometry);
}

void _3DScene::extrusionentity_to_verts(const ExtrusionEntityCollection& extrusion_entity_collection, float print_z, const Point& copy, GUI::GLModel::Geometry& geometry)
{
    for (const ExtrusionEntity* extrusion_entity : extrusion_entity_collection.entities)
        extrusionentity_to_verts(extrusion_entity, print_z, copy, geometry);
}

void _3DScene::extrusionentity_to_verts(const ExtrusionEntity* extrusion_entity, float print_z, const Point& copy, GUI::GLModel::Geometry& geometry)
{
    if (extrusion_entity != nullptr) {
        auto* extrusion_path = dynamic_cast<const ExtrusionPath*>(extrusion_entity);
        if (extrusion_path != nullptr)
            extrusionentity_to_verts(*extrusion_path, print_z, copy, geometry);
        else {
            auto* extrusion_loop = dynamic_cast<const ExtrusionLoop*>(extrusion_entity);
            if (extrusion_loop != nullptr)
                extrusionentity_to_verts(*extrusion_loop, print_z, copy, geometry);
            else {
                auto* extrusion_multi_path = dynamic_cast<const ExtrusionMultiPath*>(extrusion_entity);
                if (extrusion_multi_path != nullptr)
                    extrusionentity_to_verts(*extrusion_multi_path, print_z, copy, geometry);
                else {
                    auto* extrusion_entity_collection = dynamic_cast<const ExtrusionEntityCollection*>(extrusion_entity);
                    if (extrusion_entity_collection != nullptr)
                        extrusionentity_to_verts(*extrusion_entity_collection, print_z, copy, geometry);
                    else
                        throw Slic3r::RuntimeError("Unexpected extrusion_entity type in to_verts()");
                }
            }
        }
    }
}

} // namespace Slic3r
