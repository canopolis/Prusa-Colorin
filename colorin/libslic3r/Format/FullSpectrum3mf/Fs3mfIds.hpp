#ifndef slic3r_FullSpectrum3mf_Ids_hpp_
#define slic3r_FullSpectrum3mf_Ids_hpp_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace Slic3r::FullSpectrum3mf {

std::string sanitize_id_fragment(const std::string &value);
std::string short_stable_hash(const std::string &value);
std::string make_stable_id(const std::string &prefix, const std::string &seed);
std::string make_indexed_id(const std::string &prefix, size_t one_based_index);
std::string physical_filament_id_from_source(size_t one_based_index,
                                             const std::string &filament_id,
                                             const std::string &preset_name,
                                             const std::string &color);
std::string mixed_filament_id_from_legacy_stable_id(uint64_t stable_id,
                                                    const std::string &fallback_seed);

} // namespace Slic3r::FullSpectrum3mf

#endif
