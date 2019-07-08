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
*/
#pragma once
#include <units.h>
#include <utility>
#include <optional>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtx/polar_coordinates.hpp>
#include <libxpat/physics/Units.hpp>
#include <libxpat/macros.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/srs/spheroid.hpp>

namespace xpat {
    namespace nav {

        using spheroid_type = boost::geometry::srs::spheroid<phys::unit_numeric_t>;

        namespace WGS84 {
            constexpr const phys::meters equatorial_radius(6378137.0); // a.k.a. a
            constexpr const phys::meters polar_radius(6356752.314245); // a.k.a. b
            constexpr const phys::unit_numeric_t flattening = phys::unit_numeric_t(1.0 / 298.257223563); // a.k.a. f

            extern const spheroid_type spheroid_meters;

            phys::meters radius_at_latitude(const phys::radians& latitude) noexcept;
        }

        constexpr phys::kilometers earth_mean_radius(6371);
        constexpr phys::kilometers R_e(earth_mean_radius);

        struct NavPoint final {
            phys::degrees latitude;
            phys::degrees longitude;
            phys::feet elevation_amsl;

            explicit NavPoint(const phys::unit_numeric_t& latitude, const phys::unit_numeric_t& longitude, const phys::unit_numeric_t& feet = 0.0) noexcept;
            NavPoint(const phys::degrees& latitude, const phys::degrees& longitude, const phys::feet& = phys::feet(0.0)) noexcept;
            NavPoint() noexcept = default;
            NavPoint(const NavPoint&, const phys::feet&) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(NavPoint);

            phys::nautical_miles haversine_distance(const NavPoint&) const noexcept;
            std::optional<phys::nautical_miles> vincenty_distance(const NavPoint& that, unsigned iteration_limit = 100, const phys::radians& precision = phys::radians(1E-12)) const noexcept;
            phys::nautical_miles andoyer_distance(const NavPoint&) const noexcept;
            // Returns true bearing
            phys::degrees bearing_to(const NavPoint&) const noexcept;
            // Returns true bearing
            inline phys::degrees bearing_from(const NavPoint& that) const noexcept {
                return that.bearing_to(*this);
            }

            NavPoint lateral_translate(const phys::degrees& bearing, const phys::meters& distance) const noexcept;
            inline NavPoint& lateral_translate_in_place(const phys::degrees& bearing, const phys::meters& distance) noexcept {
                *this = this->lateral_translate(bearing, distance);
                return *this;
            }

            NavPoint translate(const phys::degrees& bearing, const phys::meters& distance, const phys::feet& altitude_change) const noexcept;
            inline NavPoint& translate_in_place(const phys::degrees& bearing, const phys::meters& distance, const phys::feet& altitude_change) noexcept {
                *this = this->translate(bearing, distance, altitude_change);
                return *this;
            }

            NavPoint normalize() const noexcept;
            inline NavPoint& normalize_in_place() noexcept {
                *this = this->normalize();
                return *this;
            }

            // Units in meters on WGS84 geoid
            glm::dvec3 as_euclidean_coord() const;

            phys::nautical_miles euclidean_distance(const NavPoint&) const;

            phys::feet vertical_distance(const NavPoint&) const noexcept;
            phys::nautical_miles slant_range(const NavPoint&) const noexcept;
            inline phys::nautical_miles slant_range_fast(const NavPoint& that) const {
                return this->euclidean_distance(that);
            }

            constexpr bool operator==(const NavPoint& that) const noexcept {
                return this->latitude == that.latitude &&
                    this->longitude == that.longitude &&
                    this->elevation_amsl == that.elevation_amsl;
            }

            constexpr bool operator!=(const NavPoint& that) const noexcept {
                return !(*this == that);
            }

            constexpr phys::unit_numeric_t get_lat_raw() const noexcept {
                return this->latitude.to<phys::unit_numeric_t>();
            }

            constexpr phys::unit_numeric_t get_lon_raw() const noexcept {
                return this->longitude.to<phys::unit_numeric_t>();
            }

            inline void set_lat_raw(phys::unit_numeric_t val) noexcept {
                this->latitude = phys::degrees(val);
            }

            inline void set_lon_raw(phys::unit_numeric_t val) noexcept {
                this->longitude = phys::degrees(val);
            }

            NavPoint move_towards(const NavPoint&, const phys::meters&) const noexcept;
        };

        namespace polar_math {
            constexpr phys::degrees full_circle{ 360.0 };
            constexpr phys::degrees semi_circle{ 180.0 };
            constexpr phys::degrees right_angle{ 90.0 };

            inline phys::degrees normalize_heading(const phys::degrees& heading) noexcept {
                return units::math::fmod(heading, full_circle);
            }

            phys::degrees normalize_longitude(const phys::degrees& longitude) noexcept;

            NavPoint normalize_nav_point(const NavPoint& nav) noexcept;
            
        }

    }
}

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(
    xpat::nav::NavPoint,
    xpat::phys::unit_numeric_t,
    boost::geometry::cs::geographic<boost::geometry::degree>,
    get_lon_raw, get_lat_raw,
    set_lon_raw,
    set_lat_raw
);