#include "ColorinPNG.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>
#include <png.h>

#include <boost/format.hpp>
#include <boost/log/trivial.hpp>

namespace Slic3r { namespace Colorin { namespace png {

using Slic3r::png::IStream;
using Slic3r::png::ReadBuf;

namespace {

struct PNGDescr {
    png_struct *png = nullptr; png_info *info = nullptr;

    PNGDescr() = default;
    PNGDescr(const PNGDescr&) = delete;
    PNGDescr(PNGDescr&&) = delete;
    PNGDescr& operator=(const PNGDescr&) = delete;
    PNGDescr& operator=(PNGDescr&&) = delete;

    ~PNGDescr()
    {
        if (png && info) png_destroy_info_struct(png, &info);
        if (png) png_destroy_read_struct( &png, nullptr, nullptr);
    }
};

static void png_read_callback(png_struct *png_ptr,
                              png_bytep   outBytes,
                              png_size_t  byteCountToRead)
{
    // Retrieve our input buffer through the png_ptr
    auto reader = static_cast<IStream *>(png_get_io_ptr(png_ptr));

    if (!reader || !reader->is_ok()) return;

    reader->read(static_cast<std::uint8_t *>(outBytes), byteCountToRead);
}

struct ReadBufStream : public IStream {
    const ReadBuf &rbuf_ref;
    size_t pos = 0;
    explicit ReadBufStream(const ReadBuf &buf) : rbuf_ref{buf} {}
    size_t read(std::uint8_t *outp, size_t amount) override
    {
        if (amount > rbuf_ref.sz - pos) return 0;
        auto buf = static_cast<const std::uint8_t *>(rbuf_ref.buf);
        std::copy(buf + pos, buf + (pos + amount), outp);
        pos += amount;
        return amount;
    }
    bool is_ok() const override { return pos < rbuf_ref.sz; }
};

} // namespace

bool decode_colored_png(IStream &in_buf, ImageColorscale &out_img)
{
    static const constexpr int PNG_SIG_BYTES = 8;

    std::vector<png_byte> sig(PNG_SIG_BYTES, 0);
    in_buf.read(sig.data(), PNG_SIG_BYTES);
    if (!png_check_sig(sig.data(), PNG_SIG_BYTES)) {
        BOOST_LOG_TRIVIAL(error) << boost::format("decode_colored_png: png_check_sig failed");
        return false;
    }

    PNGDescr dsc;
    dsc.png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr,
                                     nullptr);

    if(!dsc.png) {
        BOOST_LOG_TRIVIAL(error) << boost::format("decode_colored_png: png_create_read_struct failed");
        return false;
    }

    dsc.info = png_create_info_struct(dsc.png);
    if(!dsc.info) {
        BOOST_LOG_TRIVIAL(error) << boost::format("decode_colored_png: png_create_info_struct failed");
        png_destroy_read_struct(&dsc.png, &dsc.info, NULL);
        return false;
    }

    png_set_read_fn(dsc.png, static_cast<void *>(&in_buf), png_read_callback);

    // Tell that we have already read the first bytes to check the signature
    png_set_sig_bytes(dsc.png, PNG_SIG_BYTES);

    png_read_info(dsc.png, dsc.info);

    out_img.cols = png_get_image_width(dsc.png, dsc.info);
    out_img.rows = png_get_image_height(dsc.png, dsc.info);
    size_t color_type = png_get_color_type(dsc.png, dsc.info);
    size_t bit_depth  = png_get_bit_depth(dsc.png, dsc.info);
    unsigned long rowbytes = png_get_rowbytes(dsc.png, dsc.info);

    switch(color_type)
    {
        case PNG_COLOR_TYPE_RGB:
            out_img.bytes_per_pixel = 3;
            break;
        case PNG_COLOR_TYPE_RGB_ALPHA:
            out_img.bytes_per_pixel = 4;
            break;
        default: //not supported currently
            png_destroy_read_struct(&dsc.png, &dsc.info, NULL);
            return false;
    }

    BOOST_LOG_TRIVIAL(info) << boost::format("png's cols %1%, rows %2%, color_type %3%, bit_depth %4%, bytes_per_pixel %5%, rowbytes %6%")%out_img.cols %out_img.rows %color_type %bit_depth %out_img.bytes_per_pixel %rowbytes;
    out_img.buf.resize(out_img.rows * rowbytes);

    int filter_type = png_get_filter_type(dsc.png, dsc.info);
    int compression_type = png_get_compression_type(dsc.png, dsc.info);
    int interlace_type = png_get_interlace_type(dsc.png, dsc.info);
    BOOST_LOG_TRIVIAL(info) << boost::format("filter_type %1%, compression_type %2%, interlace_type %3%, rowbytes %4%")%filter_type %compression_type %interlace_type %rowbytes;

    auto readbuf = static_cast<png_bytep>(out_img.buf.data());
    for (size_t r = out_img.rows; r > 0; r--)
    {
        png_read_row(dsc.png, readbuf + (r - 1) * rowbytes, nullptr);
    }

    png_read_end(dsc.png, dsc.info);
    png_destroy_read_struct(&dsc.png, &dsc.info, NULL);

    return true;
}

bool decode_colored_png(const ReadBuf &in_buf, ImageColorscale &out_img)
{
    ReadBufStream stream{in_buf};
    return decode_colored_png(stream, out_img);
}

}}} // namespace Slic3r::Colorin::png
