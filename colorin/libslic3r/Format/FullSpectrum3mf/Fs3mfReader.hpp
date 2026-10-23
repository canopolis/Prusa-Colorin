#ifndef slic3r_FullSpectrum3mf_Reader_hpp_
#define slic3r_FullSpectrum3mf_Reader_hpp_

#include <map>
#include <string>

namespace Slic3r {
class DynamicPrintConfig;
class Model;
class ModelObject;
class ModelVolume;
}

namespace Slic3r::FullSpectrum3mf {

struct CanonicalBindingContext
{
    std::map<int, ModelObject *> model_objects_by_3mf_id;
    std::map<int, ModelVolume *> model_volumes_by_3mf_id;
};

class ArchiveImportState
{
public:
    bool accepts_part(const std::string &zip_path) const;
    void add_part(std::string zip_path, std::string bytes);
    bool empty() const;
    bool apply_to_config(DynamicPrintConfig &config, std::string *warning = nullptr) const;
    bool apply_to_model_and_config(Model                         &model,
                                   DynamicPrintConfig            &config,
                                   const CanonicalBindingContext  &context,
                                   std::string                   *warning = nullptr) const;

private:
    std::map<std::string, std::string> m_json_parts;
};

bool has_canonical_manifest(const std::map<std::string, std::string> &parts);
bool apply_canonical_mixed_filaments_to_config(const std::map<std::string, std::string> &parts,
                                               DynamicPrintConfig                       &config,
                                               std::string                              *warning = nullptr);

} // namespace Slic3r::FullSpectrum3mf

#endif
