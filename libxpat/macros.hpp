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
#include <type_traits>
#define XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(clazz) clazz(const clazz&) noexcept = default; \
    clazz(clazz&&) noexcept = default; \
    clazz& operator=(const clazz&) noexcept = default; \
    clazz& operator=(clazz&&) noexcept = default;

namespace xpat {
    namespace traits {
        template <typename T>
        struct atomic_well_formed {
            static constexpr bool value = std::is_trivially_copyable<T>::value &&
                std::is_move_constructible<T>::value &&
                std::is_move_assignable<T>::value &&
                std::is_copy_constructible<T>::value &&
                std::is_copy_assignable<T>::value;
        };

        template <typename T>
        constexpr bool atomic_well_formed_v = atomic_well_formed<T>::value;
    }
}