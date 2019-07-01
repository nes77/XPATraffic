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

#include "libxpat/nav_data/NavPoint.hpp"
#include <gtest/gtest.h>
#include <atomic>

using namespace xpat::nav;
using namespace xpat::phys;

namespace {

    const NavPoint KHIO{ 45.5331, -122.9475, 0.0 };
    const NavPoint KPDX(45.5898, -122.5951, 0.0);

    const NavPoint BAL(39.1710633, -76.6612564, 149.5);
    const NavPoint SLOAF(39.36674, -76.85136);
    const NavPoint SLOAF_VORDME_RWY15_MTN(SLOAF, feet(2600.0));

    TEST(NavPointTest, KnownDistance) {
        auto opt_dist = KPDX.vincenty_distance(KHIO);
        ASSERT_TRUE(opt_dist.has_value()); // We know this converges
        ASSERT_NEAR(28.15f, KPDX.haversine_distance(KHIO).convert<units::length::kilometer>().to<double>(), 0.001f); // When we're pretty close, we want fairly accurate resolution. This is within a meter.
        ASSERT_NEAR(28.225f, opt_dist->convert<units::length::kilometer>().to<double>(), 0.001f); // When we're pretty close, we want fairly accurate resolution. This is within a meter.
        ASSERT_NEAR(28.15f, KPDX.euclidean_distance(KHIO).convert<units::length::kilometer>().to<double>(), 0.05f); // Euclidean distance is really cheap to calculate, but it won't be all that accurate by comparison to the others.
    }

    TEST(NavPointTest, Bearing) {
        // KHIO -> KPDX heading 076.93194
        // KPDX -> KHIO heading 257.1836
        ASSERT_NEAR(76.93194, KHIO.bearing_to(KPDX).to<double>(), 0.01);
        ASSERT_NEAR(257.1836, KPDX.bearing_to(KHIO).to<double>(), 0.01);
        ASSERT_DOUBLE_EQ(KPDX.bearing_to(KHIO).to<double>(), KHIO.bearing_from(KPDX).to<double>());
        ASSERT_DOUBLE_EQ(KHIO.bearing_to(KPDX).to<double>(), KPDX.bearing_from(KHIO).to<double>());

    }

    TEST(NavPointTest, Elevation) {
        ASSERT_DOUBLE_EQ(2600.0 - 149.5, BAL.vertical_distance(SLOAF_VORDME_RWY15_MTN).to<double>());
    }

    TEST(NavPointTest, DMEDistance) {
        // 14.7 nautical miles, real performance of DME is +-0.1 nm, so 0.01 is fine
        // BTW, this is the first NAV point as part of the VORDME approach to some airport near Baltimore
        // which is apparently a real humdinger of an approach
        ASSERT_NEAR(14.7, BAL.slant_range(SLOAF_VORDME_RWY15_MTN).to<double>(), 0.01);
    }

    TEST(NavPointTest, Translation) {
        meters distance{ 102.889 / 20 }; // We want to be preeetty accurate when we're going at approach speeds. This is 200 knots at 20 fps
        // Good news is, we seem to get more accurate as we slow down!
        NavPoint start{ 86.32056, -1.72972 };
        degrees bearing{ 96.0216667 };
        NavPoint finish = start.lateral_translate(bearing, distance);

        ASSERT_NEAR(distance.to<double>(), start.haversine_distance(finish).convert<units::length::meter>().to<double>(), 0.01); // Correct within a centimeter
    }

    TEST(NavPointTest, Atomicity) {
        std::atomic<NavPoint> test;
        test.store(KPDX);
        auto test_KPDX = test.load();
        ASSERT_EQ(test_KPDX, KPDX);
    }

}