#include "Fs3mfJson.hpp"

#include "Fs3mfConstants.hpp"

#include <charconv>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string_view>

namespace Slic3r::FullSpectrum3mf {

namespace {

template<class T> void get_if_present(const nlohmann::json &j, const char *key, T &out)
{
    auto it = j.find(key);
    if (it != j.end() && !it->is_null())
        out = it->get<T>();
}

} // namespace

void to_json(nlohmann::json &j, const Checksum &v)
{
    j = nlohmann::json{{"algorithm", v.algorithm}, {"value", v.value}};
}

void from_json(const nlohmann::json &j, Checksum &v)
{
    get_if_present(j, "algorithm", v.algorithm);
    get_if_present(j, "value", v.value);
}

void to_json(nlohmann::json &j, const ManifestPart &v)
{
    j = nlohmann::json{
        {"role", v.role},
        {"path", v.path},
        {"content_type", v.content_type},
        {"required", v.required},
        {"checksum", v.checksum}
    };
}

void from_json(const nlohmann::json &j, ManifestPart &v)
{
    get_if_present(j, "role", v.role);
    get_if_present(j, "path", v.path);
    get_if_present(j, "content_type", v.content_type);
    get_if_present(j, "required", v.required);
    get_if_present(j, "checksum", v.checksum);
}

void to_json(nlohmann::json &j, const LegacyProjection &v)
{
    j = nlohmann::json{
        {"present", v.present},
        {"non_fullspectrum_policy", v.non_fullspectrum_policy},
        {"derived_from", v.derived_from},
        {"paths", v.paths}
    };
}

void from_json(const nlohmann::json &j, LegacyProjection &v)
{
    get_if_present(j, "present", v.present);
    get_if_present(j, "non_fullspectrum_policy", v.non_fullspectrum_policy);
    get_if_present(j, "derived_from", v.derived_from);
    get_if_present(j, "paths", v.paths);
}

void to_json(nlohmann::json &j, const Manifest &v)
{
    j = nlohmann::json{
        {"kind", v.kind},
        {"schema_version", v.schema_version},
        {"document_class", v.document_class},
        {"package_id", v.package_id},
        {"features", {
            {"required", v.required_features},
            {"optional", v.optional_features}
        }},
        {"parts", v.parts},
        {"authoritative_sources", v.authoritative_sources},
        {"legacy_projection", v.legacy_projection}
    };
}

void from_json(const nlohmann::json &j, Manifest &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "schema_version", v.schema_version);
    get_if_present(j, "document_class", v.document_class);
    get_if_present(j, "package_id", v.package_id);
    if (auto it = j.find("features"); it != j.end()) {
        get_if_present(*it, "required", v.required_features);
        get_if_present(*it, "optional", v.optional_features);
    }
    get_if_present(j, "parts", v.parts);
    get_if_present(j, "authoritative_sources", v.authoritative_sources);
    get_if_present(j, "legacy_projection", v.legacy_projection);
}

void to_json(nlohmann::json &j, const Project &v)
{
    j = nlohmann::json{
        {"kind", v.kind},
        {"schema_version", v.schema_version},
        {"project", {
            {"id", v.project_id},
            {"display_name", v.display_name}
        }},
        {"compatibility", {
            {"legacy_projection_written", v.legacy_projection_written},
            {"non_fullspectrum_policy", "physical-filaments-only"}
        }},
        {"feature_policy", {
            {"unknown_required_features", "fail_closed"}
        }}
    };
}

void from_json(const nlohmann::json &j, Project &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "schema_version", v.schema_version);
    if (auto it = j.find("project"); it != j.end()) {
        get_if_present(*it, "id", v.project_id);
        get_if_present(*it, "display_name", v.display_name);
    }
    if (auto it = j.find("compatibility"); it != j.end())
        get_if_present(*it, "legacy_projection_written", v.legacy_projection_written);
}

void to_json(nlohmann::json &j, const PhysicalFilament &v)
{
    j = nlohmann::json{
        {"id", v.id},
        {"display_name", v.display_name},
        {"material_family", v.material_family},
        {"diameter_mm", v.diameter_mm},
        {"color", v.color},
        {"source_index", v.source_index}
    };
}

void from_json(const nlohmann::json &j, PhysicalFilament &v)
{
    get_if_present(j, "id", v.id);
    get_if_present(j, "display_name", v.display_name);
    get_if_present(j, "material_family", v.material_family);
    get_if_present(j, "diameter_mm", v.diameter_mm);
    get_if_present(j, "color", v.color);
    get_if_present(j, "source_index", v.source_index);
}

void to_json(nlohmann::json &j, const Materials &v)
{
    j = nlohmann::json{
        {"kind", v.kind},
        {"schema_version", v.schema_version},
        {"physical_filaments", v.physical_filaments}
    };
}

void from_json(const nlohmann::json &j, Materials &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "schema_version", v.schema_version);
    get_if_present(j, "physical_filaments", v.physical_filaments);
}

void to_json(nlohmann::json &j, const ModelObjectBinding &v)
{
    j = nlohmann::json{{"model_object_id", v.model_object_id}, {"stable_object_id", v.stable_object_id}};
}

void from_json(const nlohmann::json &j, ModelObjectBinding &v)
{
    get_if_present(j, "model_object_id", v.model_object_id);
    get_if_present(j, "stable_object_id", v.stable_object_id);
}

void to_json(nlohmann::json &j, const VolumeBinding &v)
{
    j = nlohmann::json{
        {"model_object_id", v.model_object_id},
        {"model_volume_id", v.model_volume_id},
        {"stable_object_id", v.stable_object_id},
        {"stable_volume_id", v.stable_volume_id}
    };
}

void from_json(const nlohmann::json &j, VolumeBinding &v)
{
    get_if_present(j, "model_object_id", v.model_object_id);
    get_if_present(j, "model_volume_id", v.model_volume_id);
    get_if_present(j, "stable_object_id", v.stable_object_id);
    get_if_present(j, "stable_volume_id", v.stable_volume_id);
}

void to_json(nlohmann::json &j, const MaterialBinding &v)
{
    j = nlohmann::json{{"runtime_filament_id", v.runtime_filament_id}, {"material_ref", v.material_ref}};
}

void from_json(const nlohmann::json &j, MaterialBinding &v)
{
    get_if_present(j, "runtime_filament_id", v.runtime_filament_id);
    get_if_present(j, "material_ref", v.material_ref);
}

void to_json(nlohmann::json &j, const MixedFilamentBinding &v)
{
    j = nlohmann::json{
        {"runtime_filament_id", v.runtime_filament_id},
        {"legacy_stable_id", v.legacy_stable_id},
        {"material_ref", v.material_ref}
    };
}

void from_json(const nlohmann::json &j, MixedFilamentBinding &v)
{
    get_if_present(j, "runtime_filament_id", v.runtime_filament_id);
    get_if_present(j, "legacy_stable_id", v.legacy_stable_id);
    get_if_present(j, "material_ref", v.material_ref);
}

void to_json(nlohmann::json &j, const IdentityMap &v)
{
    j = nlohmann::json{
        {"kind", v.kind},
        {"schema_version", v.schema_version},
        {"model_object_bindings", v.model_object_bindings},
        {"volume_bindings", v.volume_bindings},
        {"material_bindings", v.material_bindings},
        {"mixed_filament_bindings", v.mixed_filament_bindings}
    };
}

void from_json(const nlohmann::json &j, IdentityMap &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "schema_version", v.schema_version);
    get_if_present(j, "model_object_bindings", v.model_object_bindings);
    get_if_present(j, "volume_bindings", v.volume_bindings);
    get_if_present(j, "material_bindings", v.material_bindings);
    get_if_present(j, "mixed_filament_bindings", v.mixed_filament_bindings);
}

void to_json(nlohmann::json &j, const AssignmentTarget &v)
{
    j = nlohmann::json{{"kind", v.kind}};
    if (!v.stable_object_id.empty())
        j["stable_object_id"] = v.stable_object_id;
    if (!v.stable_volume_id.empty())
        j["stable_volume_id"] = v.stable_volume_id;
    if (!v.stable_region_id.empty())
        j["stable_region_id"] = v.stable_region_id;
}

void from_json(const nlohmann::json &j, AssignmentTarget &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "stable_object_id", v.stable_object_id);
    get_if_present(j, "stable_volume_id", v.stable_volume_id);
    get_if_present(j, "stable_region_id", v.stable_region_id);
}

void to_json(nlohmann::json &j, const Assignment &v)
{
    j = nlohmann::json{{"id", v.id}, {"target", v.target}, {"material_ref", v.material_ref}};
}

void from_json(const nlohmann::json &j, Assignment &v)
{
    get_if_present(j, "id", v.id);
    get_if_present(j, "target", v.target);
    get_if_present(j, "material_ref", v.material_ref);
}

void to_json(nlohmann::json &j, const PaintStateBinding &v)
{
    j = nlohmann::json{
        {"scope", {{"stable_volume_id", v.stable_volume_id}}},
        {"paint_state", v.paint_state},
        {"material_ref", v.material_ref}
    };
}

void from_json(const nlohmann::json &j, PaintStateBinding &v)
{
    if (auto it = j.find("scope"); it != j.end())
        get_if_present(*it, "stable_volume_id", v.stable_volume_id);
    get_if_present(j, "paint_state", v.paint_state);
    get_if_present(j, "material_ref", v.material_ref);
}

void to_json(nlohmann::json &j, const Assignments &v)
{
    j = nlohmann::json{
        {"kind", v.kind},
        {"schema_version", v.schema_version},
        {"assignments", v.assignments},
        {"paint_state_bindings", v.paint_state_bindings}
    };
}

void from_json(const nlohmann::json &j, Assignments &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "schema_version", v.schema_version);
    get_if_present(j, "assignments", v.assignments);
    get_if_present(j, "paint_state_bindings", v.paint_state_bindings);
}

void to_json(nlohmann::json &j, const MixedFilamentOrigin &v)
{
    j = nlohmann::json{
        {"kind", v.kind},
        {"component_refs", v.component_refs},
        {"origin_auto_generated", v.origin_auto_generated}
    };
}

void from_json(const nlohmann::json &j, MixedFilamentOrigin &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "component_refs", v.component_refs);
    get_if_present(j, "origin_auto_generated", v.origin_auto_generated);
}

void to_json(nlohmann::json &j, const MixedFilamentBlend &v)
{
    j = nlohmann::json{{"type", v.type}, {"component_b_percent", v.component_b_percent}};
}

void from_json(const nlohmann::json &j, MixedFilamentBlend &v)
{
    get_if_present(j, "type", v.type);
    get_if_present(j, "component_b_percent", v.component_b_percent);
}

void to_json(nlohmann::json &j, const MixedFilamentDistribution &v)
{
    j = nlohmann::json{{"mode", v.mode}};
}

void from_json(const nlohmann::json &j, MixedFilamentDistribution &v)
{
    get_if_present(j, "mode", v.mode);
}

void to_json(nlohmann::json &j, const ManualPattern &v)
{
    j = nlohmann::json{{"groups", v.groups}};
}

void from_json(const nlohmann::json &j, ManualPattern &v)
{
    get_if_present(j, "groups", v.groups);
}

void to_json(nlohmann::json &j, const Gradient &v)
{
    j = nlohmann::json{{"component_refs", v.component_refs}, {"weights", v.weights}};
    if (v.enabled) {
        j["enabled"] = true;
        j["component_a_start"] = v.component_a_start;
        j["component_a_end"] = v.component_a_end;
        j["stop_positions"] = v.stop_positions;
        if (!v.solid_widths.empty())
            j["solid_widths"] = v.solid_widths;
    }
}

void from_json(const nlohmann::json &j, Gradient &v)
{
    get_if_present(j, "component_refs", v.component_refs);
    get_if_present(j, "weights", v.weights);
    get_if_present(j, "enabled", v.enabled);
    get_if_present(j, "component_a_start", v.component_a_start);
    get_if_present(j, "component_a_end", v.component_a_end);
    get_if_present(j, "stop_positions", v.stop_positions);
    get_if_present(j, "solid_widths", v.solid_widths);
}

void to_json(nlohmann::json &j, const SurfaceBias &v)
{
    j = nlohmann::json{
        {"component_a_offset_mm", v.component_a_offset_mm},
        {"component_b_offset_mm", v.component_b_offset_mm},
        {"application", v.application}
    };
    if (!v.component_refs.empty())
        j["component_refs"] = v.component_refs;
    if (!v.component_offsets_mm.empty())
        j["component_offsets_mm"] = v.component_offsets_mm;
}

void from_json(const nlohmann::json &j, SurfaceBias &v)
{
    get_if_present(j, "component_a_offset_mm", v.component_a_offset_mm);
    get_if_present(j, "component_b_offset_mm", v.component_b_offset_mm);
    get_if_present(j, "component_refs", v.component_refs);
    get_if_present(j, "component_offsets_mm", v.component_offsets_mm);
    get_if_present(j, "application", v.application);
}

void to_json(nlohmann::json &j, const LocalZ &v)
{
    j = nlohmann::json{{"max_sublayers", v.max_sublayers}, {"strategy", v.strategy}};
}

void from_json(const nlohmann::json &j, LocalZ &v)
{
    get_if_present(j, "max_sublayers", v.max_sublayers);
    get_if_present(j, "strategy", v.strategy);
}

void to_json(nlohmann::json &j, const VirtualFilament &v)
{
    j = nlohmann::json{
        {"id", v.id},
        {"legacy_stable_id", v.legacy_stable_id},
        {"visibility_state", v.visibility_state},
        {"source_kind", v.source_kind},
        {"origin", v.origin},
        {"blend", v.blend},
        {"distribution", v.distribution},
        {"manual_pattern", v.manual_pattern ? nlohmann::json(*v.manual_pattern) : nlohmann::json(nullptr)},
        {"gradient", v.gradient ? nlohmann::json(*v.gradient) : nlohmann::json(nullptr)},
        {"surface_bias", v.surface_bias}
    };
    if (v.local_z)
        j["local_z"] = *v.local_z;
}

void from_json(const nlohmann::json &j, VirtualFilament &v)
{
    get_if_present(j, "id", v.id);
    get_if_present(j, "legacy_stable_id", v.legacy_stable_id);
    get_if_present(j, "visibility_state", v.visibility_state);
    get_if_present(j, "source_kind", v.source_kind);
    get_if_present(j, "origin", v.origin);
    get_if_present(j, "blend", v.blend);
    get_if_present(j, "distribution", v.distribution);
    if (auto it = j.find("manual_pattern"); it != j.end() && !it->is_null())
        v.manual_pattern = it->get<ManualPattern>();
    if (auto it = j.find("gradient"); it != j.end() && !it->is_null())
        v.gradient = it->get<Gradient>();
    get_if_present(j, "surface_bias", v.surface_bias);
    if (auto it = j.find("local_z"); it != j.end() && !it->is_null())
        v.local_z = it->get<LocalZ>();
}

void to_json(nlohmann::json &j, const MixedFilaments &v)
{
    j = nlohmann::json{
        {"kind", v.kind},
        {"schema_version", v.schema_version},
        {"virtual_filaments", v.virtual_filaments}
    };
}

void from_json(const nlohmann::json &j, MixedFilaments &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "schema_version", v.schema_version);
    get_if_present(j, "virtual_filaments", v.virtual_filaments);
}

void to_json(nlohmann::json &j, const ImageMapTextureAsset &v)
{
    j = nlohmann::json{{"id", v.id}, {"display_name", v.display_name}, {"path", v.path}, {"width", v.width}, {"height", v.height}};
}

void from_json(const nlohmann::json &j, ImageMapTextureAsset &v)
{
    get_if_present(j, "id", v.id);
    get_if_present(j, "display_name", v.display_name);
    get_if_present(j, "path", v.path);
    get_if_present(j, "width", v.width);
    get_if_present(j, "height", v.height);
}

void to_json(nlohmann::json &j, const ImageMapSurfaceSource &v)
{
    j = nlohmann::json{{"kind", v.kind}, {"texture_asset_ref", v.texture_asset_ref}, {"wrap_u", v.wrap_u}, {"wrap_v", v.wrap_v},
                       {"uvs", v.uvs}, {"corner_rgba", v.corner_rgba}};
}

void from_json(const nlohmann::json &j, ImageMapSurfaceSource &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "texture_asset_ref", v.texture_asset_ref);
    get_if_present(j, "wrap_u", v.wrap_u);
    get_if_present(j, "wrap_v", v.wrap_v);
    get_if_present(j, "uvs", v.uvs);
    get_if_present(j, "corner_rgba", v.corner_rgba);
}

void to_json(nlohmann::json &j, const ImageMapTriangleBinding &v)
{
    j = nlohmann::json{{"triangle_index", v.triangle_index}, {"zone_index", v.zone_index}, {"source", v.source}};
}

void from_json(const nlohmann::json &j, ImageMapTriangleBinding &v)
{
    get_if_present(j, "triangle_index", v.triangle_index);
    get_if_present(j, "zone_index", v.zone_index);
    get_if_present(j, "source", v.source);
}

void to_json(nlohmann::json &j, const ImageMapPaletteEntry &v)
{
    j = nlohmann::json{{"target_rgba", v.target_rgba}, {"material_ref", v.material_ref},
                       {"fallback_runtime_filament_id", v.fallback_runtime_filament_id}};
}

void from_json(const nlohmann::json &j, ImageMapPaletteEntry &v)
{
    get_if_present(j, "target_rgba", v.target_rgba);
    get_if_present(j, "material_ref", v.material_ref);
    get_if_present(j, "fallback_runtime_filament_id", v.fallback_runtime_filament_id);
}

void to_json(nlohmann::json &j, const ImageMapZone &v)
{
    j = nlohmann::json{{"id", v.id}, {"display_name", v.display_name}, {"enabled", v.enabled}, {"priority", v.priority},
                       {"render_mode", v.render_mode}, {"adaptive_modulation_mode", v.adaptive_modulation_mode},
                       {"color_mix_model", v.color_mix_model},
                       {"synchronize_whole_object_cadence", v.synchronize_whole_object_cadence},
                       {"minimum_component_percent", v.minimum_component_percent},
                       {"target_sample_size_mm", v.target_sample_size_mm}, {"max_facet_samples", v.max_facet_samples},
                       {"modulation_sample_spacing_mm", v.modulation_sample_spacing_mm},
                       {"corner_smoothing_radius_mm", v.corner_smoothing_radius_mm},
                       {"disable_broad_path_smoothing", v.disable_broad_path_smoothing},
                       {"gaussian_smoothing_strength", v.gaussian_smoothing_strength},
                       {"first_path_smoothing_strength", v.first_path_smoothing_strength},
                       {"second_path_smoothing_strength", v.second_path_smoothing_strength},
                       {"tone_gamma", v.tone_gamma},
                       {"overhang_contrast_percent", v.overhang_contrast_percent},
                       {"image_exposure_ev", v.image_exposure_ev},
                       {"image_contrast_percent", v.image_contrast_percent},
                       {"image_saturation_percent", v.image_saturation_percent},
                       {"image_edge_boost_percent", v.image_edge_boost_percent},
                       {"palette", v.palette}};
}

void from_json(const nlohmann::json &j, ImageMapZone &v)
{
    get_if_present(j, "id", v.id);
    get_if_present(j, "display_name", v.display_name);
    get_if_present(j, "enabled", v.enabled);
    get_if_present(j, "priority", v.priority);
    get_if_present(j, "render_mode", v.render_mode);
    get_if_present(j, "adaptive_modulation_mode", v.adaptive_modulation_mode);
    get_if_present(j, "color_mix_model", v.color_mix_model);
    get_if_present(j, "synchronize_whole_object_cadence", v.synchronize_whole_object_cadence);
    get_if_present(j, "minimum_component_percent", v.minimum_component_percent);
    get_if_present(j, "target_sample_size_mm", v.target_sample_size_mm);
    get_if_present(j, "max_facet_samples", v.max_facet_samples);
    get_if_present(j, "modulation_sample_spacing_mm", v.modulation_sample_spacing_mm);
    get_if_present(j, "corner_smoothing_radius_mm", v.corner_smoothing_radius_mm);
    get_if_present(j, "disable_broad_path_smoothing", v.disable_broad_path_smoothing);
    get_if_present(j, "gaussian_smoothing_strength", v.gaussian_smoothing_strength);
    get_if_present(j, "first_path_smoothing_strength", v.first_path_smoothing_strength);
    get_if_present(j, "second_path_smoothing_strength", v.second_path_smoothing_strength);
    get_if_present(j, "tone_gamma", v.tone_gamma);
    get_if_present(j, "overhang_contrast_percent", v.overhang_contrast_percent);
    get_if_present(j, "image_exposure_ev", v.image_exposure_ev);
    get_if_present(j, "image_contrast_percent", v.image_contrast_percent);
    get_if_present(j, "image_saturation_percent", v.image_saturation_percent);
    get_if_present(j, "image_edge_boost_percent", v.image_edge_boost_percent);
    get_if_present(j, "palette", v.palette);
}

void to_json(nlohmann::json &j, const ImageMapVolume &v)
{
    j = nlohmann::json{{"stable_volume_id", v.stable_volume_id}, {"topology_fingerprint", v.topology_fingerprint},
                       {"zones", v.zones}, {"triangle_bindings", v.triangle_bindings}};
}

void from_json(const nlohmann::json &j, ImageMapVolume &v)
{
    get_if_present(j, "stable_volume_id", v.stable_volume_id);
    get_if_present(j, "topology_fingerprint", v.topology_fingerprint);
    get_if_present(j, "zones", v.zones);
    get_if_present(j, "triangle_bindings", v.triangle_bindings);
}

void to_json(nlohmann::json &j, const ImageMaps &v)
{
    j = nlohmann::json{{"kind", v.kind}, {"schema_version", v.schema_version}, {"texture_assets", v.texture_assets}, {"volumes", v.volumes}};
}

void from_json(const nlohmann::json &j, ImageMaps &v)
{
    get_if_present(j, "kind", v.kind);
    get_if_present(j, "schema_version", v.schema_version);
    get_if_present(j, "texture_assets", v.texture_assets);
    get_if_present(j, "volumes", v.volumes);
}

void to_json(nlohmann::json &j, const PreservedPart &v)
{
    j = nlohmann::json{
        {"path", v.path},
        {"content_type", v.content_type},
        {"role", v.role},
        {"bytes", v.bytes},
        {"required", v.required},
        {"must_preserve", v.must_preserve}
    };
}

void from_json(const nlohmann::json &j, PreservedPart &v)
{
    get_if_present(j, "path", v.path);
    get_if_present(j, "content_type", v.content_type);
    get_if_present(j, "role", v.role);
    get_if_present(j, "bytes", v.bytes);
    get_if_present(j, "required", v.required);
    get_if_present(j, "must_preserve", v.must_preserve);
}

// ---------------------------------------------------------------------------
// Fast image-map JSON (writer + reader)
// ---------------------------------------------------------------------------
namespace {

struct EscapedStringCache
{
    std::string last_in;
    std::string last_out;
    bool        valid = false;
    const std::string &get(const std::string &in)
    {
        if (!valid || in != last_in) {
            last_in  = in;
            last_out = nlohmann::json(in).dump();
            valid    = true;
        }
        return last_out;
    }
};

void append_double(std::string &out, double v)
{
    if (!std::isfinite(v)) {
        out += "null";
        return;
    }
    char  buf[64];
    char *end = nlohmann::detail::to_chars(buf, buf + sizeof(buf), v);
    out.append(buf, end);
}

void append_double_array(std::string &out, const std::vector<double> &values)
{
    out += '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i)
            out += ',';
        append_double(out, values[i]);
    }
    out += ']';
}

struct FastReader
{
    const char *p;
    const char *e;

    [[noreturn]] void fail() const { throw std::runtime_error("image-maps fast parse"); }
    void ws()
    {
        while (p < e && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t'))
            ++p;
    }
    void expect(char c)
    {
        ws();
        if (p >= e || *p != c)
            fail();
        ++p;
    }
    bool consume(char c)
    {
        ws();
        if (p < e && *p == c) {
            ++p;
            return true;
        }
        return false;
    }
    // Returns the raw string token including quotes.
    std::string_view raw_string()
    {
        ws();
        if (p >= e || *p != '"')
            fail();
        const char *s = p++;
        while (p < e && *p != '"') {
            if (*p == '\\')
                ++p;
            ++p;
        }
        if (p >= e)
            fail();
        ++p;
        return std::string_view(s, size_t(p - s));
    }
    std::string string_value()
    {
        std::string_view r = raw_string();
        if (r.find('\\') == std::string_view::npos)
            return std::string(r.substr(1, r.size() - 2));
        return nlohmann::json::parse(r.begin(), r.end()).get<std::string>();
    }
    // Keys used here never contain escapes; an escaped key just won't match.
    std::string_view key()
    {
        std::string_view r = raw_string();
        expect(':');
        return r.substr(1, r.size() - 2);
    }
    void skip_value()
    {
        ws();
        if (p >= e)
            fail();
        if (*p == '"') {
            raw_string();
            return;
        }
        if (*p == '{' || *p == '[') {
            int depth = 0;
            while (p < e) {
                char c = *p;
                if (c == '"') {
                    raw_string();
                    continue;
                }
                if (c == '{' || c == '[')
                    ++depth;
                else if (c == '}' || c == ']') {
                    if (--depth == 0) {
                        ++p;
                        return;
                    }
                }
                ++p;
            }
            fail();
        }
        while (p < e && *p != ',' && *p != '}' && *p != ']' && *p != ' ' && *p != '\n' && *p != '\r' && *p != '\t')
            ++p;
    }
    template<class T> T delegate()
    {
        ws();
        const char *s = p;
        skip_value();
        return nlohmann::json::parse(s, p).get<T>();
    }
    double number()
    {
        ws();
        double v = 0.;
#if defined(_MSC_VER)
        auto r = std::from_chars(p, e, v);
        if (r.ec != std::errc())
            fail();
        p = r.ptr;
#else
        const char *s = p;
        while (p < e && (std::strchr("+-0123456789.eE", *p) != nullptr) && *p != '\0')
            ++p;
        if (p == s)
            fail();
        v = nlohmann::json::parse(s, p).get<double>();
#endif
        return v;
    }
    uint64_t unsigned_number()
    {
        ws();
        uint64_t v = 0;
        auto     r = std::from_chars(p, e, v);
        if (r.ec != std::errc())
            fail();
        p = r.ptr;
        // tolerate a fractional/exponent part written by other tools
        if (p < e && (*p == '.' || *p == 'e' || *p == 'E'))
            fail();
        return v;
    }
    void double_array(std::vector<double> &out)
    {
        out.clear();
        expect('[');
        if (consume(']'))
            return;
        do {
            out.push_back(number());
        } while (consume(','));
        expect(']');
    }
};

struct CachedString
{
    std::string_view raw;
    std::string      value;
};

void read_string_cached(FastReader &r, CachedString &cache, std::string &out)
{
    const char      *before = r.p;
    std::string_view raw    = r.raw_string();
    (void) before;
    if (raw != cache.raw) {
        cache.raw = raw;
        if (raw.find('\\') == std::string_view::npos)
            cache.value.assign(raw.data() + 1, raw.size() - 2);
        else
            cache.value = nlohmann::json::parse(raw.begin(), raw.end()).get<std::string>();
    }
    out = cache.value;
}

struct SourceStringCaches
{
    CachedString kind, texture_asset_ref, wrap_u, wrap_v;
};

void read_source(FastReader &r, ImageMapSurfaceSource &src, SourceStringCaches &caches)
{
    r.expect('{');
    if (r.consume('}'))
        return;
    do {
        std::string_view k = r.key();
        if (k == "kind")
            read_string_cached(r, caches.kind, src.kind);
        else if (k == "texture_asset_ref")
            read_string_cached(r, caches.texture_asset_ref, src.texture_asset_ref);
        else if (k == "wrap_u")
            read_string_cached(r, caches.wrap_u, src.wrap_u);
        else if (k == "wrap_v")
            read_string_cached(r, caches.wrap_v, src.wrap_v);
        else if (k == "uvs")
            r.double_array(src.uvs);
        else if (k == "corner_rgba")
            r.double_array(src.corner_rgba);
        else
            r.skip_value();
    } while (r.consume(','));
    r.expect('}');
}

void read_bindings(FastReader &r, std::vector<ImageMapTriangleBinding> &out)
{
    out.clear();
    SourceStringCaches caches;
    r.expect('[');
    if (r.consume(']'))
        return;
    do {
        ImageMapTriangleBinding b;
        r.expect('{');
        if (!r.consume('}')) {
            do {
                std::string_view k = r.key();
                if (k == "triangle_index")
                    b.triangle_index = uint32_t(r.unsigned_number());
                else if (k == "zone_index")
                    b.zone_index = uint32_t(r.unsigned_number());
                else if (k == "source")
                    read_source(r, b.source, caches);
                else
                    r.skip_value();
            } while (r.consume(','));
            r.expect('}');
        }
        out.emplace_back(std::move(b));
    } while (r.consume(','));
    r.expect(']');
}

void read_volume(FastReader &r, ImageMapVolume &v)
{
    r.expect('{');
    if (r.consume('}'))
        return;
    do {
        std::string_view k = r.key();
        if (k == "stable_volume_id")
            v.stable_volume_id = r.string_value();
        else if (k == "topology_fingerprint")
            v.topology_fingerprint = r.unsigned_number();
        else if (k == "zones")
            v.zones = r.delegate<std::vector<ImageMapZone>>();
        else if (k == "triangle_bindings")
            read_bindings(r, v.triangle_bindings);
        else
            r.skip_value();
    } while (r.consume(','));
    r.expect('}');
}

ImageMaps parse_image_maps_custom(const std::string &bytes)
{
    FastReader r{bytes.data(), bytes.data() + bytes.size()};
    ImageMaps  out;
    r.expect('{');
    if (!r.consume('}')) {
        do {
            std::string_view k = r.key();
            if (k == "kind")
                out.kind = r.string_value();
            else if (k == "schema_version")
                out.schema_version = r.string_value();
            else if (k == "texture_assets")
                out.texture_assets = r.delegate<std::vector<ImageMapTextureAsset>>();
            else if (k == "volumes") {
                r.expect('[');
                if (!r.consume(']')) {
                    do {
                        ImageMapVolume v;
                        read_volume(r, v);
                        out.volumes.emplace_back(std::move(v));
                    } while (r.consume(','));
                    r.expect(']');
                }
            } else
                r.skip_value();
        } while (r.consume(','));
        r.expect('}');
    }
    r.ws();
    if (r.p != r.e)
        r.fail();
    return out;
}

} // namespace

std::string serialize_image_maps_fast(const ImageMaps &v)
{
    size_t bindings = 0;
    for (const ImageMapVolume &vol : v.volumes)
        bindings += vol.triangle_bindings.size();

    std::string out;
    out.reserve(4096 + bindings * 320);
    out += "{\"kind\":";
    out += nlohmann::json(v.kind).dump();
    out += ",\"schema_version\":";
    out += nlohmann::json(v.schema_version).dump();
    out += ",\"texture_assets\":";
    out += nlohmann::json(v.texture_assets).dump();
    out += ",\"volumes\":[";
    for (size_t vi = 0; vi < v.volumes.size(); ++vi) {
        const ImageMapVolume &vol = v.volumes[vi];
        if (vi)
            out += ',';
        out += "{\"stable_volume_id\":";
        out += nlohmann::json(vol.stable_volume_id).dump();
        out += ",\"topology_fingerprint\":";
        out += std::to_string(vol.topology_fingerprint);
        out += ",\"zones\":";
        out += nlohmann::json(vol.zones).dump();
        out += ",\"triangle_bindings\":[";
        EscapedStringCache kind, asset, wrap_u, wrap_v;
        for (size_t bi = 0; bi < vol.triangle_bindings.size(); ++bi) {
            const ImageMapTriangleBinding &b = vol.triangle_bindings[bi];
            if (bi)
                out += ',';
            out += "{\"triangle_index\":";
            out += std::to_string(b.triangle_index);
            out += ",\"zone_index\":";
            out += std::to_string(b.zone_index);
            out += ",\"source\":{\"kind\":";
            out += kind.get(b.source.kind);
            out += ",\"texture_asset_ref\":";
            out += asset.get(b.source.texture_asset_ref);
            out += ",\"wrap_u\":";
            out += wrap_u.get(b.source.wrap_u);
            out += ",\"wrap_v\":";
            out += wrap_v.get(b.source.wrap_v);
            out += ",\"uvs\":";
            append_double_array(out, b.source.uvs);
            out += ",\"corner_rgba\":";
            append_double_array(out, b.source.corner_rgba);
            out += "}}";
        }
        out += "]}";
    }
    out += "]}";
    return out;
}

ImageMaps parse_image_maps_fast(const std::string &bytes)
{
    try {
        return parse_image_maps_custom(bytes);
    } catch (...) {
        // Unexpected layout: use the original (slow but complete) parser.
        return parse_json<ImageMaps>(bytes);
    }
}

} // namespace Slic3r::FullSpectrum3mf
