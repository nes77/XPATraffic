#include <libxpat/physics/PhysModel.hpp>
#include <libxpat/physics/Units.hpp>
#include <units.h>
#include <gtest/gtest.h>

using namespace units;
using namespace xpat::phys;
using namespace xpat::nav;

namespace {
    TEST(SimplePhysics, WindBasics) {
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
}