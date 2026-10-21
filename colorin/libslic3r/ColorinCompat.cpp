#include "ColorinCompat.hpp"

namespace Slic3r {
namespace Colorin {

std::string encode_paint_state_as_facet_string(unsigned int state)
{
    int value = int(state);
    if (value < 0 || value > int(PaintStateMax))
        return {};
    if (value <= 2)
        return std::string(1, char('0' + (value << 2)));

    auto hex_digit = [](int nibble) {
        return char(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    };

    value -= 3;
    std::string encoded("C");
    while (value >= 15) {
        encoded.insert(encoded.begin(), 'F');
        value -= 15;
    }
    encoded.insert(encoded.begin(), hex_digit(value));
    return encoded;
}

} // namespace Colorin
} // namespace Slic3r
