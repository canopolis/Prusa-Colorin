#ifndef slic3r_FullSpectrum3mf_LegacyBridge_hpp_
#define slic3r_FullSpectrum3mf_LegacyBridge_hpp_

#include "Fs3mfTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r {
class DynamicPrintConfig;
class MixedFilamentManager;
}

namespace Slic3r::FullSpectrum3mf {

Materials materials_from_project_config(const DynamicPrintConfig &config);
std::vector<std::string> physical_filament_refs(const Materials &materials);

MixedFilaments mixed_filaments_from_manager(const MixedFilamentManager    &manager,
                                            const std::vector<std::string> &physical_refs);
std::string legacy_rows_from_mixed_filaments(const MixedFilaments          &mixed_filaments,
                                             const std::vector<std::string> &physical_refs);
MixedFilamentManager manager_from_mixed_filaments(const MixedFilaments          &mixed_filaments,
                                                  const std::vector<std::string> &filament_colours,
                                                  const std::vector<std::string> &physical_refs);

} // namespace Slic3r::FullSpectrum3mf

#endif
