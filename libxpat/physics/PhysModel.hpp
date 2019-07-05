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

#include <libxpat/nav_data/NavPoint.hpp>
#include <units.h>
#include <utility>
#include <variant>
#include <atomic>
#include <tuple>
#include <type_traits>
#include <libxpat/macros.hpp>
#include <libxpat/physics/Units.hpp>

/*
    PhysModel.hpp
    We think of our physics from a polar perspective; this has advantages and disadvantages
    The primary disadvantage is that some of the math we have to use is pretty confusing for non-mathematicians (myself included)
    The primary advantage is that certain things make more intuitive sense using polar math.

    When we're in the sim, we think generally in terms of heading, VSI, and airspeed, which can all be easily mapped in a polar space
    in a fairly direct manner.
*/

namespace xpat {
    namespace phys {

        constexpr fpm touchdown_speed{ 150 };

        enum class FlightPhase {
            PARKED,
            TAXI,
            TAKEOFF,
            INIT_CLIMB,
            CLIMB,
            CRUISE,
            INIT_DESCENT,
            DESCENT,
            APPROACH,
            FINAL,
            FLARE,
            ROLLOUT
        };

        struct FlightIntent {
            FlightPhase cur_phase;
            feet target_alt_amsl;
            knots target_kias;
            std::variant<degrees, deg_per_s> target_dir_change;

            FlightIntent(FlightPhase, feet, knots, const std::variant<degrees, deg_per_s>&) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(FlightIntent);
            constexpr bool target_is_heading() const noexcept {
                return std::holds_alternative<degrees>(target_dir_change);
            }

            const degrees& target_heading() const noexcept;
            const deg_per_s& target_turn_rate() const noexcept;
            

        };

        struct VerticalSpeedModel {

            static constexpr fpm vsi_landing{ 150 };

            fpm vsi_init_climb;
            fpm vsi_climb;
            fpm vsi_cruise;
            fpm vsi_descent;
            fpm vsi_approach;

            VerticalSpeedModel(fpm vsi_init_climb, fpm vsi_climb, fpm vsi_cruise, fpm vsi_descent, fpm vsi_approach) noexcept;
            VerticalSpeedModel() noexcept = default;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(VerticalSpeedModel);

        };

        struct AirspeedModel {
            knots v_rot;
            knots v_init_climb;
            knots v_climb;
            knots v_cruise;
            knots v_init_descent;
            knots v_descent;
            knots v_approach_min;
            knots v_ref;
            knots max_taxi_speed;

            AirspeedModel(knots v_rot, knots v_init_climb, knots v_climb, knots v_cruise, knots v_init_descent, knots v_descent, knots v_approach_min, knots v_ref, knots max_taxi_speed) noexcept;
            AirspeedModel() noexcept = default;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(AirspeedModel);
        };

        struct AngularMovementModel {

            static constexpr deg_per_s2 taxi_turn_accel{ 3.0 };

            degrees max_bank;
            deg_per_s bank_rate;
            deg_per_s max_taxi_turn_rate;
            deg_per_s max_air_turn_rate;

            AngularMovementModel(degrees max_bank, deg_per_s bank_rate, deg_per_s max_taxi_turn_rate, deg_per_s max_air_turn_rate) noexcept;
            AngularMovementModel() noexcept = default;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(AngularMovementModel);

            inline void set_taxi_turn_rate(const seconds& rotation_period = seconds(45)) noexcept {
                this->max_taxi_turn_rate = nav::polar_math::full_circle / rotation_period;
            }
            inline void set_air_turn_rate(const seconds& turn_period = seconds(120)) noexcept {
                this->max_air_turn_rate = nav::polar_math::full_circle / turn_period;
            }

            // These units are incredibly obnoxious
            static constexpr auto radial_conversion_constant1 = decltype(scalar_t() / degrees()){ 20 * units::constants::detail::PI_VAL }; // deg^-1
            static constexpr auto radial_conversion_constant2 = phys::constants::g.convert<decltype(knots() * knots() / feet())::unit_type>(); // kt^2 / ft

            static degrees bank_required(const knots& tas, const deg_per_s& turn_rate) noexcept;
            static degrees bank_required(const knots& tas, const nautical_miles& turn_radius) noexcept;
            static constexpr nautical_miles turn_radius(const knots& tas, const deg_per_s& turn_rate) noexcept {
                return tas / (turn_rate * AngularMovementModel::radial_conversion_constant1);
            }
            knots max_tas_for_turn(const deg_per_s& turn_rate) const noexcept;
            knots max_tas_for_turn(const nautical_miles& turn_radius) const noexcept;
        };

        struct AccelerationModel {
            mps2 vsi_accel{ 1.9 };
            mps2 max_accel{ 2.0 };
            mps2 max_decel{ 2.0 };

            AccelerationModel(mps2 vsi_accel, mps2 max_accel, mps2 max_decel) noexcept;
            AccelerationModel() noexcept = default;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(AccelerationModel);
        };

        struct FlightModel {

            VerticalSpeedModel vsi_mod;
            AirspeedModel ias_mod;
            AngularMovementModel ang_mod;
            AccelerationModel acc_mod;

            FlightModel(VerticalSpeedModel vsi_mod, AirspeedModel ias_mod, AngularMovementModel ang_mod, AccelerationModel acc_mod) noexcept;
            FlightModel() noexcept = default;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(FlightModel);

            constexpr std::tuple<knots, fpm> phase_limit(const FlightPhase phase) const noexcept {
                switch (phase) {
                case FlightPhase::APPROACH:
                    return std::make_tuple(ias_mod.v_approach_min, vsi_mod.vsi_approach);
                case FlightPhase::CLIMB:
                    return std::make_tuple(ias_mod.v_climb, vsi_mod.vsi_climb);
                case FlightPhase::CRUISE:
                    return std::make_tuple(ias_mod.v_cruise, vsi_mod.vsi_cruise);
                case FlightPhase::DESCENT:
                    return std::make_tuple(ias_mod.v_descent, vsi_mod.vsi_descent);
                case FlightPhase::FINAL:
                    return std::make_tuple(ias_mod.v_ref, vsi_mod.vsi_approach);
                case FlightPhase::FLARE:
                    return std::make_tuple(ias_mod.v_ref, vsi_mod.vsi_landing);
                case FlightPhase::INIT_CLIMB:
                    return std::make_tuple(ias_mod.v_init_climb, vsi_mod.vsi_init_climb);
                case FlightPhase::INIT_DESCENT:
                    return std::make_tuple(ias_mod.v_init_descent, vsi_mod.vsi_descent);
                case FlightPhase::PARKED:
                    return std::make_tuple(knots(0), fpm(0));
                case FlightPhase::ROLLOUT:
                    return std::make_tuple(ias_mod.v_rot, fpm(0));
                case FlightPhase::TAKEOFF:
                    return std::make_tuple(ias_mod.v_rot, fpm(0));
                case FlightPhase::TAXI:
                    return std::make_tuple(ias_mod.max_taxi_speed, fpm(0));
                }
            }

        };
        
        struct Windspeed {
            degrees bearing;
            knots magnitude;

            constexpr Windspeed(const degrees& bearing = degrees(0.0), const knots& mag = knots(0.0)) noexcept : bearing(bearing), magnitude(mag)  {};
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(Windspeed);
            inline degrees origin() const noexcept {
                return nav::polar_math::normalize_heading(bearing + nav::polar_math::semi_circle);
            }

            // Positive for headwind, negative for tailwind
            knots headwind_component(const degrees& heading) const noexcept;

            // Positive for rightward wind, negative for leftward
            knots crosswind_component(const degrees& heading) const noexcept;
        };

        struct AngularPosition {

            AngularPosition(degrees pitch = degrees(0), degrees roll = degrees(0), degrees yaw = degrees(0)) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(AngularPosition)

            degrees pitch;
            degrees roll;
            degrees yaw;

            inline degrees bank_angle() const noexcept {
                return units::math::fabs(roll);
            }
        };

        struct AngularVelocity {

            AngularVelocity(deg_per_s delta_pitch = deg_per_s(0), deg_per_s delta_roll = deg_per_s(0), deg_per_s delta_yaw = deg_per_s(0)) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(AngularVelocity)

            deg_per_s delta_pitch;
            deg_per_s delta_roll;
            deg_per_s delta_yaw;
        };

        struct Acceleration {

            Acceleration(mps2 airspeed_accel = mps2(0), mps2 vertical_accel = mps2(0)) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(Acceleration)

            mps2 airspeed_accel;
            mps2 vertical_accel;
        };

        struct Velocity {

            Velocity(knots airspeed = knots(0), degrees bearing = degrees(0), deg_per_s turn_rate = deg_per_s(0), fpm vertical_speed = fpm(0), AngularVelocity ang_vel = AngularVelocity()) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(Velocity)

            knots airspeed;
            degrees heading;
            deg_per_s turn_rate;
            fpm vertical_speed;

            AngularVelocity angular_velocity;

            knots true_airspeed(const Windspeed&) const noexcept;
            knots horizontal_airspeed() const noexcept;
            knots true_horizontal_airspeed(const Windspeed&) const noexcept;

        };

        struct Position {

            Position(nav::NavPoint loc = nav::NavPoint(), AngularPosition ang_pos = AngularPosition()) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(Position)

            nav::NavPoint pos;
            AngularPosition angular_pos;

            static_assert(traits::atomic_well_formed_v<nav::NavPoint>);
            static_assert(traits::atomic_well_formed_v<AngularPosition>);

        };

        struct AircraftPhysics {

            AircraftPhysics(Position pos = Position(), Velocity vel = Velocity(), Acceleration accel = Acceleration()) noexcept;
            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(AircraftPhysics)

            Position pos;
            Velocity vel;
            Acceleration accel;

            static_assert(traits::atomic_well_formed_v<Position>);
            static_assert(traits::atomic_well_formed_v<Velocity>);
            static_assert(traits::atomic_well_formed_v<Acceleration>);

            degrees angle_of_attack() const noexcept;

            AircraftPhysics update(const FlightIntent&, const FlightModel&, const Windspeed&, const seconds&) const noexcept;
            inline AircraftPhysics& update_in_place(const FlightIntent& intent, const FlightModel& model, const Windspeed& wind_speed, const seconds& elapsed) noexcept {
                *this = this->update(intent, model, wind_speed, elapsed);
                return *this;
            }

        private:
            void handle_taxi(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept;
            void handle_takeoff(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const Windspeed& wind, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept;
            void handle_parked(xpat::phys::Velocity& v1) const noexcept;
            void handle_rollout(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept;
            void handle_flare(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const Windspeed& wind, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept;
            void handle_airborne(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const Windspeed& wind, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept;
        };

        namespace {
            static_assert(traits::atomic_well_formed_v<AircraftPhysics>);
        }
    }
}