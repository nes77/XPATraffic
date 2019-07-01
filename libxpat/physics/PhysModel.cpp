
#include <libxpat/physics/PhysModel.hpp>
#include <utility>
#include <spdlog/spdlog.h>

using namespace xpat::phys;
using namespace xpat::nav;
using namespace units;

xpat::phys::AngularPosition::AngularPosition(degrees pitch, degrees roll, degrees yaw) noexcept
    : pitch(std::move(pitch)),
    roll(std::move(roll)),
    yaw(std::move(yaw))
{
}

knots xpat::phys::Windspeed::headwind_component(const degrees& heading) const noexcept
{
    degrees diff(math::fabs(polar_math::normalize_longitude(heading - this->bearing)));

    return this->magnitude * (math::cos(diff) * -1.0);
}

knots xpat::phys::Windspeed::crosswind_component(const degrees& heading) const noexcept
{
    degrees diff(polar_math::normalize_heading(heading - this->bearing));

    return (this->magnitude * (math::sin(diff) * -1.0));
}

xpat::phys::AngularVelocity::AngularVelocity(deg_per_s delta_pitch, deg_per_s delta_roll, deg_per_s delta_yaw) noexcept
    : delta_pitch(std::move(delta_pitch)), delta_roll(std::move(delta_pitch)), delta_yaw(std::move(delta_yaw))
{
}

xpat::phys::Acceleration::Acceleration(mps2 airspeed_accel, mps2 vertical_accel) noexcept
    : airspeed_accel(std::move(airspeed_accel)), vertical_accel(std::move(vertical_accel))
{
}

xpat::phys::Velocity::Velocity(knots airspeed, degrees bearing, deg_per_s turn_rate, fpm vertical_speed, AngularVelocity ang_vel) noexcept
    : airspeed(std::move(airspeed)), heading(std::move(bearing)), turn_rate(std::move(turn_rate)), vertical_speed(std::move(vertical_speed)), angular_velocity(std::move(ang_vel))
{
}

// Ignoring crosswind for now
knots xpat::phys::Velocity::true_airspeed(const Windspeed&) const noexcept
{
    // The horizontal component is horizontal_airspeed - headwind_component
    // The vertical component is our vertical speed
    return knots();
}

// The component of our indicated airspeed that is horizontal
knots xpat::phys::Velocity::horizontal_airspeed() const noexcept
{
    const radians angle_from_horizon = math::asin(this->vertical_speed / this->airspeed);
    return this->airspeed * math::cos(angle_from_horizon);
}

knots xpat::phys::Velocity::true_horizontal_airspeed(const Windspeed& wind) const noexcept
{
    return this->horizontal_airspeed() - wind.headwind_component(this->heading);
}

xpat::phys::Position::Position(nav::NavPoint loc, AngularPosition ang_pos) noexcept
    : pos(std::move(loc)), angular_pos(std::move(ang_pos))
{
}

xpat::phys::AircraftPhysics::AircraftPhysics(Position pos, Velocity vel, Acceleration accel) noexcept
    : pos(std::move(pos)), vel(std::move(vel)), accel(std::move(accel))
{
}

degrees xpat::phys::AircraftPhysics::angle_of_attack() const noexcept
{
    return degrees();
}

// http://physicsforgames.blogspot.com/2010/02/kinematic-integration.html
// If an error happens, we're taking everything down with us.
// ... maybe we should change that later.
AircraftPhysics xpat::phys::AircraftPhysics::update(const FlightIntent& intent, const FlightModel& model, const Windspeed& wind, const seconds& elapsed) const noexcept
{
    // We start by knowing our acceleration.
    // The first thing we need to calculate is our new velocity.
    Velocity v1;
    

    switch (intent.cur_phase) {
    case FlightPhase::TAXI: this->handle_taxi(model, intent, elapsed, v1); break;
    case FlightPhase::TAKEOFF: this->handle_takeoff(model, intent, wind, elapsed, v1); break;
    case FlightPhase::ROLLOUT: this->handle_rollout(model, intent, elapsed, v1); break;
    case FlightPhase::PARKED: this->handle_parked(v1); break;
    case FlightPhase::FLARE: this->handle_flare(model, intent, wind, elapsed, v1); break;
    case FlightPhase::APPROACH:
    case FlightPhase::CLIMB:
    case FlightPhase::CRUISE:
    case FlightPhase::DESCENT:
    case FlightPhase::FINAL:
    case FlightPhase::INIT_CLIMB:
    case FlightPhase::INIT_DESCENT:
        this->handle_airborne(model, intent, wind, elapsed, v1); break;
    // No default, we want the compiler warning here.
    }

    return AircraftPhysics();
}

void xpat::phys::AircraftPhysics::handle_taxi(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept
{
    {
        const deg_per_s& max_turn_speed = model.ang_mod.max_taxi_turn_rate;
        const Velocity& v0 = this->vel;
        if (intent.target_is_heading()) {
            const degrees& target_heading = intent.target_heading();
            // If we have a heading, let's assume we just want the max turn rate... assuming we're not already on that heading.
            // This is the turn-in-place scenario, i.e. for lining up on a runway
            const degrees& v0_delta_heading = math::fabs(v0.turn_rate * elapsed);
            if (headings_within(target_heading, v0.heading, v0_delta_heading)) {
                if (target_heading == v0.heading) { // No more turning!
                    v1.turn_rate = deg_per_s(0);
                }
                else { // We're still turning, but almost there!
                    v1.turn_rate = polar_math::normalize_longitude(target_heading - v0.heading) / elapsed;
                }
            }
            else {
                // Let's make sure we're turning toward our target.
                // Is it faster to turn right or left?
                const degrees diff_to_target = polar_math::normalize_longitude(v0.heading - target_heading);
                if (diff_to_target > degrees(0)) { // Turn left
                    const deg_per_s target_turn_speed = max_turn_speed * -1.0;
                    const deg_per_s2 target_turn_acceleration = AngularMovementModel::taxi_turn_accel * -1.0;
                    const deg_per_s turn_speed_delta = target_turn_acceleration * elapsed;
                    v1.turn_rate = math::fmax(v0.turn_rate + turn_speed_delta, target_turn_speed);
                }
                else { // Turn right
                    const deg_per_s turn_speed_delta = AngularMovementModel::taxi_turn_accel * elapsed;
                    v1.turn_rate = math::fmin(v0.turn_rate + turn_speed_delta, max_turn_speed);
                }
            }
        }
        else {
            const deg_per_s& target_turn_speed = intent.target_turn_rate();
            if (target_turn_speed < v0.turn_rate) {
                const deg_per_s2 target_turn_acceleration = AngularMovementModel::taxi_turn_accel * -1.0;
                const deg_per_s turn_speed_delta = target_turn_acceleration * elapsed;
                v1.turn_rate = math::fmax(v0.turn_rate + turn_speed_delta, target_turn_speed);
            }
            else if (target_turn_speed > v0.turn_rate) {
                const deg_per_s turn_speed_delta = AngularMovementModel::taxi_turn_accel * elapsed;
                v1.turn_rate = math::fmin(v0.turn_rate + turn_speed_delta, target_turn_speed);
            }
            else {
                v1.turn_rate = v0.turn_rate;
            }
        }
        // That's the taxi turn rate. What about the taxi speed?
        const knots& target_speed = intent.target_kias;
        if (v0.airspeed < target_speed) {
            v1.airspeed = math::fmin(v0.airspeed + model.acc_mod.max_accel * elapsed, model.ias_mod.max_taxi_speed);
        }
        else if (v0.airspeed > target_speed) {
            v1.airspeed = math::fmax(v0.airspeed + model.acc_mod.max_decel * elapsed, -model.ias_mod.max_taxi_speed);
        }
        else {
            v1.airspeed = v0.airspeed;
        }

    }
}

/*
Precondition: We are on the runway, and lined up with the runway heading.
Our goal is to accelerate. Anything more complex is handled elsewhere.
*/
void xpat::phys::AircraftPhysics::handle_takeoff(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const Windspeed& wind, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept
{
    const Velocity& v0 = this->vel;
    v1.turn_rate = deg_per_s(0);
    const knots cur_kias = v0.airspeed + wind.headwind_component(v0.heading);
    v1.airspeed = v0.airspeed;
    if (cur_kias <= intent.target_kias) {
        v1.airspeed += model.acc_mod.max_accel * elapsed;
    }
}

// Stop everything!
void xpat::phys::AircraftPhysics::handle_parked(xpat::phys::Velocity& v1) const noexcept
{
    v1 = this->vel;
    v1.airspeed = knots(0);
    v1.angular_velocity = AngularVelocity();
    v1.turn_rate = deg_per_s(0);
    v1.vertical_speed = fpm(0);
}

void xpat::phys::AircraftPhysics::handle_rollout(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept
{
}

void xpat::phys::AircraftPhysics::handle_flare(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const Windspeed& wind, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept
{
    // We want to reduce our vertical speed to about 150 fpm.
    v1 = this->vel;
    v1.angular_velocity = AngularVelocity();
    v1.turn_rate = deg_per_s(0);
    v1.vertical_speed = math::fmax(v1.vertical_speed - model.acc_mod.vsi_accel * elapsed, VerticalSpeedModel::vsi_landing);
}

void xpat::phys::AircraftPhysics::handle_airborne(const xpat::phys::FlightModel& model, const xpat::phys::FlightIntent& intent, const Windspeed& wind, const xpat::phys::seconds& elapsed, xpat::phys::Velocity& v1) const noexcept
{
    // Airborne handling is pretty similar for most phases of flight. The only real difference is our performance limits.
}

xpat::phys::FlightIntent::FlightIntent(FlightPhase phase, feet intend_alt, knots intend_speed, const std::variant<degrees, deg_per_s>& heading_variant) noexcept
    : cur_phase(phase), target_alt_amsl(std::move(intend_alt)), target_kias(std::move(intend_speed)), target_dir_change(heading_variant)
{}

const degrees& xpat::phys::FlightIntent::target_heading() const noexcept
{
    try {
        return std::get<degrees>(this->target_dir_change);
    }
    catch (const std::bad_variant_access& e) {
        spdlog::critical(e.what());
        spdlog::default_logger()->flush();
        std::terminate();
    }
}

const deg_per_s& xpat::phys::FlightIntent::target_turn_rate() const noexcept
{
    try {
        return std::get<deg_per_s>(this->target_dir_change);
    }
    catch (const std::bad_variant_access& e) {
        spdlog::critical(e.what());
        spdlog::default_logger()->flush();
        std::terminate();
    }
}

xpat::phys::VerticalSpeedModel::VerticalSpeedModel(fpm vsi_init_climb, fpm vsi_climb, fpm vsi_cruise_climb, fpm vsi_cruise_descent, fpm vsi_descent, fpm vsi_approach) noexcept
    : vsi_init_climb(vsi_init_climb), vsi_climb(vsi_climb), vsi_cruise_climb(vsi_cruise_climb), vsi_cruise_descent(vsi_cruise_descent), vsi_descent(vsi_descent), vsi_approach(vsi_approach)
{
}

xpat::phys::AirspeedModel::AirspeedModel(knots v_rot, knots v_init_climb, knots v_climb, knots v_cruise, knots v_init_descent, knots v_descent, knots v_approach_min, knots v_ref, knots max_taxi_speed) noexcept
    : v_rot(v_rot), v_init_climb(v_init_climb), v_climb(v_climb), v_cruise(v_cruise), v_init_descent(v_init_descent), v_descent(v_descent), v_approach_min(v_approach_min), v_ref(v_ref), max_taxi_speed(max_taxi_speed)
{
}

xpat::phys::AngularMovementModel::AngularMovementModel(degrees max_bank, deg_per_s bank_rate, deg_per_s max_taxi_turn_rate, deg_per_s max_air_turn_rate) noexcept
    : max_bank(max_bank), bank_rate(bank_rate), max_taxi_turn_rate(max_taxi_turn_rate), max_air_turn_rate(max_air_turn_rate)
{
}

xpat::phys::AccelerationModel::AccelerationModel(mps2 vsi_accel, mps2 max_accel, mps2 max_decel) noexcept
    : vsi_accel(vsi_accel), max_accel(max_accel), max_decel(max_decel)
{
}

xpat::phys::FlightModel::FlightModel(VerticalSpeedModel vsi_mod, AirspeedModel ias_mod, AngularMovementModel ang_mod, AccelerationModel acc_mod) noexcept
    : vsi_mod(vsi_mod), ias_mod(ias_mod), ang_mod(ang_mod), acc_mod(acc_mod)
{
}
