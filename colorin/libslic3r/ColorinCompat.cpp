#include "ColorinCompat.hpp"

namespace Slic3r {
namespace Colorin {

std::string encode_paint_state_as_facet_string(unsigned int state)
{
    // Prusa Colorin (FullSpectrum): written in the PrusaSlicer 2.9.6 TriangleSelector leaf encoding
    // (decode_leaf_state / FacetsAnnotation::set_triangle_from_string), NOT the OrcaSlicer base-15 one.
    // Orca: 0xC followed by 0xF repeats (+15 each) and a final nibble. Prusa: states 3-16 -> 0xC, (state-3);
    // states 17-255 -> 0xC, 0xE, low nibble, high nibble of (state-17). The string is read right to left.
    if (state > PaintStateMax)
        return {};
    auto hex_digit = [](unsigned int nibble) {
        return char(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    };
    if (state <= 2)
        return std::string(1, hex_digit(state << 2));
    if (state <= 16)
        return std::string{hex_digit(state - 3), 'C'};
    const unsigned int value = state - 17;
    return std::string{hex_digit(value >> 4), hex_digit(value & 0xF), 'E', 'C'};
}

} // namespace Colorin
} // namespace Slic3r
