#ifndef slic3r_FullSpectrum3mf_Validation_hpp_
#define slic3r_FullSpectrum3mf_Validation_hpp_

#include "Fs3mfTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::FullSpectrum3mf {

struct ValidationResult
{
    bool                     valid = true;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    void fail(std::string message);
    void warn(std::string message);
};

ValidationResult validate_package_model(const PackageModel &model);

} // namespace Slic3r::FullSpectrum3mf

#endif
