#include "PerimeterModulation.hpp"

#include "../libslic3r.h"
#include "../ExPolygon.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Slic3r {

namespace {

// Prusa Colorin: PrusaSlicer 2.9.6 no tiene Point::projection_onto ni
// projection_onto(ExPolygons, Point). Copia fiel de la logica de OrcaSlicer.
Point colorin_projection_onto_line(const Point& pt, const Point& a, const Point& b)
{
    if (a == b)
        return a;
    const double lx    = double(b.x() - a.x());
    const double ly    = double(b.y() - a.y());
    const double theta = (double(b.x() - pt.x()) * lx + double(b.y() - pt.y()) * ly) / (lx * lx + ly * ly);
    if (0.0 <= theta && theta <= 1.0)
        return Point((theta * a.cast<double>() + (1.0 - theta) * b.cast<double>()).cast<coord_t>());
    return ((a - pt).cast<double>().squaredNorm() < (b - pt).cast<double>().squaredNorm()) ? a : b;
}

Point colorin_projection_onto_polygon(const Point& pt, const Polygon& poly)
{
    if (poly.points.empty())
        return pt;
    Point        running_projection = poly.points.front();
    double       running_min        = (running_projection - pt).cast<double>().norm();
    const size_t n                  = poly.points.size();
    for (size_t i = 0; i < n; ++i) {
        const Point  candidate = colorin_projection_onto_line(pt, poly.points[i], poly.points[(i + 1) % n]);
        const double d         = (candidate - pt).cast<double>().norm();
        if (d < running_min) {
            running_projection = candidate;
            running_min        = d;
        }
    }
    return running_projection;
}

Point projection_onto(const ExPolygons& polygons, const Point& from)
{
    Point  projected_pt(0, 0);
    double min_dist = std::numeric_limits<double>::max();
    for (const ExPolygon& poly : polygons) {
        for (size_t i = 0; i < poly.num_contours(); ++i) {
            const Point  p    = colorin_projection_onto_polygon(from, poly.contour_or_hole(i));
            const double dist = (from - p).cast<double>().squaredNorm();
            if (dist < min_dist) {
                projected_pt = p;
                min_dist     = dist;
            }
        }
    }
    return projected_pt;
}

bool layer_slices_contain(const ExPolygons& layer_slices, const Point& point)
{
    return std::any_of(layer_slices.begin(), layer_slices.end(), [&point](const ExPolygon& slice) { return slice.contains(point, false); });
}

Vec2d fallback_outward_direction(const Points& points, size_t point_index, const ExPolygons& layer_slices, double probe_scaled)
{
    if (points.size() < 2)
        return Vec2d::Zero();

    size_t previous = point_index;
    while (previous > 0 && points[previous - 1] == points[point_index])
        --previous;
    size_t next = point_index;
    while (next + 1 < points.size() && points[next + 1] == points[point_index])
        ++next;

    const Point& before         = previous > 0 ? points[previous - 1] : points[point_index];
    const Point& after          = next + 1 < points.size() ? points[next + 1] : points[point_index];
    Vec2d        tangent        = (after - before).cast<double>();
    const double tangent_length = tangent.norm();
    if (!std::isfinite(tangent_length) || tangent_length <= EPSILON)
        return Vec2d::Zero();
    tangent /= tangent_length;

    const Vec2d normal(-tangent.y(), tangent.x());
    auto        probe = [&points, point_index, probe_scaled](const Vec2d& direction) {
        const Vec2d shifted = points[point_index].cast<double>() + direction * probe_scaled;
        return Point(coord_t(std::llround(shifted.x())), coord_t(std::llround(shifted.y())));
    };
    const bool positive_inside = layer_slices_contain(layer_slices, probe(normal));
    const bool negative_inside = layer_slices_contain(layer_slices, probe(-normal));
    if (positive_inside != negative_inside)
        return positive_inside ? -normal : normal;
    return normal;
}

} // namespace

bool apply_mixed_filament_perimeter_modulation(
    Polyline& polyline, const ExPolygons& layer_slices, float surface_inset_mm, float max_abs_shift_mm, float max_boundary_distance_mm)
{
    if (polyline.points.size() < 2 || layer_slices.empty() || !std::isfinite(surface_inset_mm) || !std::isfinite(max_abs_shift_mm) ||
        max_abs_shift_mm <= 0.f || !std::isfinite(max_boundary_distance_mm) || max_boundary_distance_mm <= 0.f)
        return false;

    const double clamped_inset_mm     = std::clamp(double(surface_inset_mm), -double(max_abs_shift_mm), double(max_abs_shift_mm));
    const double outward_shift_scaled = scale_(-clamped_inset_mm);
    if (std::abs(outward_shift_scaled) <= EPSILON)
        return false;

    const double probe_scaled                 = scale_(std::max(0.05, std::abs(clamped_inset_mm) + 0.05));
    const double max_boundary_distance_scaled = scale_(max_boundary_distance_mm);
    Points       shifted_points;
    shifted_points.reserve(polyline.points.size());
    bool changed = false;
    for (size_t point_index = 0; point_index < polyline.points.size(); ++point_index) {
        const Point& source   = polyline.points[point_index];
        const Point  boundary = projection_onto(layer_slices, source);
        Vec2d        outward  = (boundary - source).cast<double>();
        const double distance = outward.norm();
        if (!std::isfinite(distance) || distance > max_boundary_distance_scaled) {
            shifted_points.emplace_back(source);
            continue;
        }
        if (distance <= EPSILON)
            outward = fallback_outward_direction(polyline.points, point_index, layer_slices, probe_scaled);
        else
            outward /= distance;

        if (!outward.allFinite() || outward.squaredNorm() <= EPSILON * EPSILON) {
            shifted_points.emplace_back(source);
            continue;
        }

        const Vec2d shifted = source.cast<double>() + outward * outward_shift_scaled;
        if (!shifted.allFinite()) {
            shifted_points.emplace_back(source);
            continue;
        }
        const Point quantized(coord_t(std::llround(shifted.x())), coord_t(std::llround(shifted.y())));
        shifted_points.emplace_back(quantized);
        changed |= quantized != source;
    }

    if (changed)
        polyline.points = std::move(shifted_points);
    return changed;
}

} // namespace Slic3r
