// Prusa Colorin (FullSpectrum): colour PNG decoding used by imported textures.
// Copied from OrcaSlicer FullSpectrum PNGReadWrite (decode_colored_png); PrusaSlicer only decodes greyscale.
#ifndef slic3r_ColorinPNG_hpp_
#define slic3r_ColorinPNG_hpp_

#include "PNGReadWrite.hpp"

namespace Slic3r { namespace Colorin { namespace png {

struct ImageColorscale : Slic3r::png::Image<unsigned char>
{
    int bytes_per_pixel;
};

bool decode_colored_png(Slic3r::png::IStream &in_buf, ImageColorscale &out_img);
bool decode_colored_png(const Slic3r::png::ReadBuf &in_buf, ImageColorscale &out_img);

}}} // namespace Slic3r::Colorin::png

#endif
