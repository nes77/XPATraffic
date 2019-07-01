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
#pragma once

#include <units.h>
#include <type_traits>

namespace xpat {
    namespace phys {
        // We define some commonly used units so we don't pollute the namespace by using "using namespace units::length, etc"
        // Also, with all the unit tomfoolery in aviation data, having all this stuff defined and handled is nice.
        using degrees = units::angle::degree_t;
        using feet = units::length::foot_t;
        using miles = units::length::mile_t;
        using meters = units::length::meter_t;
        using kilometers = units::length::kilometer_t;
        using radians = units::angle::radian_t;
        using nautical_miles = units::length::nauticalMile_t;
        using knots = units::velocity::knot_t;

        using seconds = units::time::second_t;

        using deg_per_s = units::angular_velocity::degrees_per_second_t;
        using rad_per_s = units::angular_velocity::radians_per_second_t;

        using deg_per_s2 = units::unit_t<units::detail::unit_divide<units::angular_velocity::degrees_per_second, units::time::second> >;

        using mps = units::velocity::meters_per_second_t; // Aviation is a bit of a mess re: US units mixed with Metric, and so is our code!
        using kmh = units::velocity::kilometers_per_hour_t;
        using fpm_base = units::detail::unit_divide<units::length::foot, units::time::minute>;
        using fpm = units::unit_t<fpm_base>;

        using mps2 = units::acceleration::meters_per_second_squared_t; // Most common unit I've found for acceleration values in aviation

        template <typename UnitType>
        inline bool units_within(const UnitType& lhs, const UnitType& rhs, const UnitType& epsilon) noexcept {
            static_assert(units::traits::is_unit_t<UnitType>::value, "All arguments to units_within must be the same unit_t type");
            return units::math::fabs(lhs - rhs) < epsilon;
        }

        template <typename UnitLHS, typename UnitRHS, typename EpsilonType, typename std::enable_if<!std::is_same<UnitLHS, UnitRHS>::value>::type* = nullptr >
        inline bool units_within(const UnitLHS& lhs, const UnitRHS& rhs, const EpsilonType& epsilon) noexcept {
            static_assert(units::traits::is_convertible_unit_t<UnitLHS, UnitRHS>::value && units::traits::is_convertible_unit_t<UnitLHS, EpsilonType>::value);
            return units_within(lhs, rhs.template convert<UnitLHS::unit_type>(), epsilon.template convert<UnitLHS::unit_type>());
        }        

        template <typename AngleUnit, typename AU2, typename AU3>
        inline bool headings_within(const AngleUnit& lhs, const AU2& rhs, const AU3& epsilon) {
            static_assert(units::traits::is_convertible_unit_t<degrees, AngleUnit>::value &&
                units::traits::is_convertible_unit_t<AU2, AU3>::value &&
                units::traits::is_convertible_unit_t<AU2, AngleUnit>::value);
            return units_within(lhs, rhs, epsilon);
        }
    }
}