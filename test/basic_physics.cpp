#include <libxpat/physics/PhysModel.hpp>
#include <libxpat/physics/Units.hpp>
#include <units.h>
#include <gtest/gtest.h>

using namespace units;
using namespace xpat::phys;
using namespace xpat::nav;

namespace {

    static const FlightModel med_jet = []() -> auto {
        FlightModel f;
        f.acc_mod = AccelerationModel(mps2(2.0), mps2(2.0), mps2(-2.0));
        f.ang_mod = AngularMovementModel(degrees(30), deg_per_s(5), deg_per_s(12), deg_per_s(3));
        f.ias_mod = AirspeedModel(knots(150), knots(152), knots(250), knots(270), knots(270), knots(270),
            knots(180), knots(140), knots(5));
        f.vsi_mod = VerticalSpeedModel(fpm(3000), fpm(2400), fpm(1000), fpm(3500), fpm(1500));
        return f;
    }();

    static const NavPoint KPDX_RWY_28L(degrees(45) + moa(34.8309), degrees(-122) + moa(-35.0341), feet(22.7));

    TEST(Physics, WindBasics) {
        Windspeed strong_ne_wind(degrees(32), knots(15));
        Velocity poor_plane(knots(60), degrees(34));

        ASSERT_NEAR(-14.99, strong_ne_wind.headwind_component(poor_plane.heading).to<double>(), 0.005);
        ASSERT_NEAR(-0.52, strong_ne_wind.crosswind_component(poor_plane.heading).to<double>(), 0.005);

        strong_ne_wind.bearing += degrees(4);

        ASSERT_NEAR(-14.99, strong_ne_wind.headwind_component(poor_plane.heading).to<double>(), 0.005);
        ASSERT_NEAR(0.52, strong_ne_wind.crosswind_component(poor_plane.heading).to<double>(), 0.005);

        Windspeed perfect_headwind(poor_plane.heading + polar_math::semi_circle, strong_ne_wind.magnitude);

        ASSERT_NEAR(15.0, perfect_headwind.headwind_component(poor_plane.heading).to<double>(), 0.005);
        ASSERT_NEAR(0.0, perfect_headwind.crosswind_component(poor_plane.heading).to<double>(), 0.005);

        Windspeed perfect_crosswind(poor_plane.heading + polar_math::right_angle, knots(15));
        // Perfect right wind

        ASSERT_NEAR(0.0, perfect_crosswind.headwind_component(poor_plane.heading).to<double>(), 0.005);
        ASSERT_NEAR(15.0, perfect_crosswind.crosswind_component(poor_plane.heading).to<double>(), 0.005);

        perfect_crosswind.bearing += polar_math::semi_circle;

        //Perfect left wind
        ASSERT_NEAR(-15.0, perfect_crosswind.crosswind_component(poor_plane.heading).to<double>(), 0.005);
    }

    TEST(Physics, TakeoffRoll) {
        auto a = AircraftPhysics(Position(KPDX_RWY_28L), Velocity(knots(0), degrees(283)));
    }
}