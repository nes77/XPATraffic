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
*/

#pragma once

#include <cstring>
#include <cstdint>
#include <type_traits>
#include <string_view>
#include <functional>
#include <array>
#include <atomic>
#include <libxpat/macros.hpp>
#include <ostream>

namespace xpat {
    namespace data {
        // Provides a string type that is both constexpr and fits in an x86_64 register
        // Probably works on big-endian platforms as well, but obviously is not portable between the two
        // TODO: Replace char with char8_t when C++20 is done
        template<typename IntType>
        class alignas(sizeof(IntType)) StackString {
            static_assert(std::is_integral_v<IntType>);
            static_assert(sizeof(IntType) > 1);
            static_assert(sizeof(char) == 1); // TODO: replace me in C++20
            std::array<char, sizeof(IntType)> _data;

            constexpr IntType* const as_int_ptr() noexcept {
                return reinterpret_cast<IntType* const>(_data.data());
            }

        public:
            static constexpr std::size_t size = sizeof(IntType);

            constexpr IntType as_integer() const noexcept {
                return *const_cast<StackString*>(this)->as_int_ptr();
            }

            constexpr StackString() noexcept {
                *this->as_int_ptr() = IntType(0);
            }

            template<std::size_t csize>
            constexpr StackString(const char (&inp)[csize]) noexcept : StackString() {
                auto lim = std::min(StackString::size, csize);
                for (std::size_t i = 0; i < lim; i++) {
                    _data[i] = inp[i];
                }
                _data.back() = '\0'; // Ensures the string is null terminated
            }

            inline StackString(const char* inp, std::size_t count) noexcept : StackString() {
                std::copy_n(inp, std::min(count, StackString::size - 1), _data.data());
            }

            inline StackString(const std::string& view) noexcept : StackString(view.data(), view.size()) {}

            XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(StackString);
            

            // Yes, this returns a reference to a member
            // I'm assuming (though it may bite me later) that users of this function
            // will read this comment and realize that the lifetime of this string_view
            // is bound to this object, or at least read the docs for string_view and realize
            // that.
            constexpr std::string_view str_view() const noexcept {
                return std::string_view(_data.data(), StackString::size);
            }

            constexpr const char* c_str() const noexcept {
                return _data.data();
            }

            constexpr bool operator==(const StackString& that) const noexcept {
                return this->as_integer() == that.as_integer();
            }

            constexpr bool operator!=(const StackString& that) const noexcept {
                return this->as_integer() == that.as_integer();
            }

            using atomic_type = std::atomic<StackString>;

            


        };

        // Can store 3 characters + null terminator
        using StackString4 = StackString<uint32_t>;

        // Can store 7 characters + null terminator
        using StackString8 = StackString<uint64_t>;
    }
}

namespace std {
    template <typename IntType> struct hash< xpat::data::StackString<IntType> > {
        inline std::size_t operator()(const xpat::data::StackString<IntType>& ss) {
            if constexpr (sizeof(IntType) <= sizeof(std::size_t)) {
                return ss.as_integer();
            }
            else {
                // Guaranteed defined according to C++17
                return std::hash<IntType>()(ss.as_integer());
            }
        }
    };



    template <typename IntType>
    inline ostream& operator<< (ostream& out, const xpat::data::StackString<IntType>& ss) {
        return out << ss.str_view();
    }
}