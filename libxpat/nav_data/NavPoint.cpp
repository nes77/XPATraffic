/*
    XPATraffic: FOSS ATC for X-Plane
    Copyright(C) 2019 Nicholas Samson

    This program is free software : you can redistribute itand /or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.If not, see < https://www.gnu.org/licenses/>.

    Additional permission under GNU GPL version 3 section 7

    If you modify this Program, or any covered work, by linking or combining
    it with the X-Plane SDK by Laminar Research (or a modified version of that
    library), containing parts covered by the terms of the MIT License, the
    licensors of this Program grant you additional permission to convey the
    resulting work.
    {Corresponding Source for a non-source form of such a combination shall
    include the source code for the parts of the X-Plane SDK by Laminar Research
    used as well as that of the covered work.}
*/
#include <libxpat/nav_data/NavPoint.hpp>
#include <glm/geometric.hpp>
#include <cmath>
#include <boost/geometry/algorithms/detail/azimuth.hpp>
#include <boost/geometry/formulas/andoyer_inverse.hpp>

using namespace xpat::nav;
using namespace xpat::phys;
using namespace units;

namespace bg = boost::geometry;

const spheroid_type WGS84::spheroid_meters{ WGS84::equatorial_radius.to<unit_numeric_t>(), WGS84::polar_radius.to<unit_numeric_t>() };


xpat::nav::NavPoint::NavPoint(const unit_numeric_t& latitude, const unit_numeric_t& longitude, const unit_numeric_t& elev) noexcept : NavPoint(degrees(latitude), degrees(longitude), feet(elev))
{
}

xpat::nav::NavPoint::NavPoint(const phys::degrees& latitude, const phys::degrees& longitude, const phys::feet& elevation) noexcept : latitude(latitude), longitude(longitude), elevation_amsl(elevation) {}

xpat::nav::NavPoint::NavPoint(const NavPoint& nav_point, const phys::feet& elevation) noexcept : NavPoint(nav_point.latitude, nav_point.longitude, elevation) {}

glm::dvec3 xpat::nav::NavPoint::as_euclidean_coord() const {
    glm::dvec2 base{ this->latitude.convert<units::angle::radians>().to<unit_numeric_t>(),
        this->longitude.convert<units::angle::radians>().to<unit_numeric_t>() };

    auto out = glm::euclidean(base);
    out *= earth_mean_radius.convert<units::length::meters>().to<unit_numeric_t>();
    return out;
}

nautical_miles xpat::nav::NavPoint::euclidean_distance(const NavPoint& that) const
{
    const auto coord_a = this->as_euclidean_coord();
    const auto coord_b = that.as_euclidean_coord();

    return meters(glm::distance(coord_a, coord_b));
}

feet xpat::nav::NavPoint::vertical_distance(const NavPoint& that) const noexcept
{
    return math::fabs(this->elevation_amsl - that.elevation_amsl);
}

nautical_miles xpat::nav::NavPoint::slant_range(const NavPoint& that) const noexcept
{
    return math::sqrt(math::cpow<2>(this->haversine_distance(that)) + math::cpow<2>(this->vertical_distance(that)));
}

NavPoint xpat::nav::NavPoint::move_towards(const NavPoint& that, const meters& distance) const noexcept
{

    degrees bearing = this->bearing_to(that);
    return this->lateral_translate(bearing, distance);

}

NavPoint xpat::nav::NavPoint::translate(const degrees& bearing, const meters& distance, const feet& altitude_change) const noexcept
{
    return NavPoint(this->lateral_translate(bearing, distance), this->elevation_amsl + altitude_change);
}

NavPoint xpat::nav::NavPoint::normalize() const noexcept
{
    return polar_math::normalize_nav_point(*this);
}

NavPoint xpat::nav::NavPoint::lateral_translate(const degrees& bearing, const meters& distance) const noexcept
{
    const auto result = bg::strategy::vincenty::direct<phys::unit_numeric_t>::apply(
        bg::get_as_radian<0>(*this), bg::get_as_radian<1>(*this),
        distance.to<unit_numeric_t>(),
        polar_math::normalize_longitude(bearing).convert<radians::unit_type>().to<unit_numeric_t>(),
        WGS84::spheroid_meters
    );

    return NavPoint(radians(result.lat2), radians(result.lon2), this->elevation_amsl);

}


nautical_miles NavPoint::haversine_distance(const NavPoint& other) const noexcept
{
    const bg::strategy::distance::haversine<unit_numeric_t> unit_haversine;
    const scalar_t unit_dist{ bg::distance(*this, other, unit_haversine) };

    return earth_mean_radius * unit_dist;

}

std::optional<nautical_miles> xpat::nav::NavPoint::vincenty_distance(const NavPoint& that, unsigned iteration_limit, const radians& precision) const noexcept
{
    const bg::strategy::distance::vincenty<> strat{ WGS84::spheroid_meters };
    const meters dist_m{ bg::distance(*this, that, strat) };

    return std::make_optional(nautical_miles(dist_m));
    
}

nautical_miles xpat::nav::NavPoint::andoyer_distance(const NavPoint& that) const noexcept
{
    const bg::strategy::distance::andoyer<> strat{ WGS84::spheroid_meters };
    const meters dist_m{ bg::distance(*this, that, strat) };

    return dist_m;
}

degrees xpat::nav::NavPoint::bearing_to(const NavPoint& that) const noexcept
{
    const radians out{ bg::formula::andoyer_inverse<phys::unit_numeric_t, false, true>::apply(
        bg::get_as_radian<0>(*this), bg::get_as_radian<1>(*this),
        bg::get_as_radian<0>(that), bg::get_as_radian<1>(that),
        WGS84::spheroid_meters
    ).azimuth };

    return polar_math::normalize_heading(out + polar_math::full_circle);
}

degrees xpat::nav::polar_math::normalize_longitude(const phys::degrees& longitude) noexcept {
    auto new_lon = normalize_heading(longitude);
    if (new_lon > semi_circle) {
        new_lon -= full_circle;
    }
    else if (new_lon <= -semi_circle) {
        new_lon += full_circle;
    }
    return new_lon;
}

NavPoint xpat::nav::polar_math::normalize_nav_point(const NavPoint& nav) noexcept {
    NavPoint out;
    out.longitude = normalize_longitude(nav.longitude);
    auto new_lat = normalize_longitude(nav.latitude);
    bool flip = false;
    if (new_lat > right_angle) {
        new_lat = semi_circle - new_lat;
        flip = true;
    }
    else if (new_lat <= -right_angle) {
        new_lat = -semi_circle - new_lat;
        flip = true;
    }

    if (flip) {
        out.longitude *= -1.0f;
    }

    out.latitude = new_lat;

    return out;
}

meters xpat::nav::WGS84::radius_at_latitude(const phys::radians& latitude) noexcept {
    auto cos_lat = units::math::cos(latitude);
    auto sin_lat = units::math::cos(latitude);

    return units::math::sqrt((units::math::cpow<2>(units::math::cpow<2>(WGS84::equatorial_radius) * cos_lat) + units::math::cpow<2>(units::math::cpow<2>(WGS84::polar_radius) * sin_lat)) /
        (units::math::cpow<2>(WGS84::equatorial_radius * cos_lat) + units::math::cpow<2>(WGS84::polar_radius * sin_lat)));
}
