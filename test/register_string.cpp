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

#include <libxpat/data/StackString.hpp>
#include <libxpat/macros.hpp>
#include <gtest/gtest.h>
#include <atomic>
#include <iostream>

using namespace xpat::data;
namespace {

    TEST(StackString, SimpleMechanics) {
        StackString8 a;
        ASSERT_STREQ(std::string().c_str(), std::string(a.str_view()).c_str());
        ASSERT_EQ(0, a.as_integer());

        a = StackString8("Hello!");
        ASSERT_STREQ(std::string("Hello!").c_str(), std::string(a.str_view()).c_str());

        a = StackString8("Hello, world!", 6);
        ASSERT_STREQ(std::string("Hello,").c_str(), std::string(a.str_view()).c_str());
        ASSERT_TRUE(xpat::traits::atomic_well_formed_v<StackString8>);
        ASSERT_EQ(sizeof(StackString8), 8);
        ASSERT_TRUE(std::atomic<StackString8>::is_always_lock_free);

        std::cerr << "Testing ostream operator: " << a << '\n';

        ASSERT_EQ(a, StackString8("Hello,"));
        
        StackString8::atomic_type atom{ a };
        
        auto b = atom.exchange("Bye!");
        ASSERT_EQ(b, a);
        ASSERT_EQ(atom.load(), StackString8("Bye!"));
    }

}