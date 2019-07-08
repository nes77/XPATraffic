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

#include <libxpat/Config.hpp>
#include <gtest/gtest.h>

using namespace xpat;

namespace {
    TEST(VersionString, BasicFunc) {
        const Version test_obj(0, 10, 0);
        const Version parse_obj("0.10.0-alpha");

        ASSERT_EQ(test_obj, parse_obj);
        ASSERT_STREQ(parse_obj.str().c_str(), "0.10.0");

        const Version release(1, 0, 0);

        ASSERT_GT(release, parse_obj);
        ASSERT_LT(release, Version(1, 0, 1));
        ASSERT_GT(release, Version(0, 100, 2));

    }
}
