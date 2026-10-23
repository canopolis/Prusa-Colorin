// Prusa Colorin (FullSpectrum): self-contained SHA-256 (FIPS 180-4), replacing OpenSSL SHA256()
// used by the OrcaSlicer FullSpectrum 3MF module. Output is byte-identical to OpenSSL.
#ifndef slic3r_ColorinSha256_hpp_
#define slic3r_ColorinSha256_hpp_

#include <cstddef>
#include <cstdint>

namespace Slic3r { namespace Colorin {

constexpr std::size_t SHA256_DIGEST_SIZE = 32;
void sha256(const unsigned char *data, std::size_t size, unsigned char digest[SHA256_DIGEST_SIZE]);

}} // namespace Slic3r::Colorin

#endif
