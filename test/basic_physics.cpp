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

    TEST(Physics, BankAngle) {
        ASSERT_NEAR(
            AngularMovementModel::bank_required(knots(120.0), feet(2215.0)).to<double>(),
            30.0,
            0.1
        );

        ASSERT_NEAR(
            AngularMovementModel::turn_radius(knots(120.0), deg_per_s(5.25)).convert<feet::unit_type>().to<double>(),
            2215.0,
            10.0
        );

        ASSERT_NEAR(
            AngularMovementModel::turn_rate(knots(120.0), feet(2215)).to<double>(),
            5.25,
            0.05
        );
        auto a = med_jet.ang_mod.max_turn_rate_for_tas(knots(250));
        std::cout << "Max rate for 250 kts turn: " << a << '\n';
        std::cout << "Bank required: " << AngularMovementModel::bank_required(knots(250), a) << '\n';

        ASSERT_NEAR(AngularMovementModel::bank_required(knots(250), a).to<double>(), 30.0, 0.05);
        a = med_jet.ang_mod.max_turn_rate_for_tas(knots(140));
        std::cout << "Max rate for 140 kts turn: " << a << '\n';
        std::cout << "Bank required: " << AngularMovementModel::bank_required(knots(140), a) << '\n';
        ASSERT_LT(AngularMovementModel::bank_required(knots(140), a).to<double>(), 25.0);

        ASSERT_NEAR(
            AngularMovementModel::turn_radius(knots(180.0), deg_per_s(3)).to<double>(),
            0.95,
            0.01
        );
    }
}