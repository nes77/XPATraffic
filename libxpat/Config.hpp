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
#include <filesystem>
#include <libxpat/macros.hpp>
#include <string>
#include <cstdint>
#include <string_view>
#include <stdexcept>
#include <ostream>
#include <boost/property_tree/json_parser.hpp>


namespace xpat {
    using path_t = std::filesystem::path;

    struct BadVersionString final : public std::invalid_argument {
        explicit BadVersionString(std::string bad_ver) noexcept;
    };

    struct Version {

        uint32_t major{ 0 };
        uint32_t minor{ 0 };
        uint32_t patch{ 0 };

        Version(const std::string&); // throws on bad version string
        constexpr Version(uint32_t major, uint32_t minor, uint32_t patch) noexcept : major(major), minor(minor), patch(patch) {}
        constexpr Version() noexcept = default;

        XPAT_DEFINE_CTORS_DEFAULT_NOEXCEPT(Version);

        std::string str() const noexcept;

        constexpr bool operator==(const Version& that) const noexcept {
            return major == that.major && minor == that.minor && patch == that.patch;
        }

        constexpr bool operator!=(const Version& that) const noexcept {
            return !(*this == that);
        }

        constexpr bool operator<(const Version& that) const noexcept {
            if (this->major < that.major) {
                return true;
            }
            else if(this->major == that.major) {
                if (this->minor < that.minor) {
                    return true;
                }
                else if (this->minor == that.minor) {
                    return this->patch < that.patch;
                }
            }

            return false;
        }

        constexpr bool operator<=(const Version& that) const noexcept {
            return *this == that || *this < that;
        }

        constexpr bool operator>(const Version& that) const noexcept {
            return !(*this <= that);
        }

        constexpr bool operator>=(const Version& that) const noexcept {
            return !(*this < that);
        }

    };

    struct InvalidConfig : public std::invalid_argument {
        explicit InvalidConfig(const std::string& reason) noexcept;
    };

    class Config {
        
        Version _version;
        path_t _xplane_base; // Must be a directory

        void set_version(Version ver);
        void set_xplane_base(path_t path);

    public:
        
        // Expects a path to a file containing a JSON config following our schema (TBA)
        Config(const path_t& ifile);

        Config() noexcept;

        constexpr const Version& version() const noexcept {
            return _version;
        }

        constexpr const path_t& xplane_base_dir() const noexcept {
            return _xplane_base;
        }

    };

    namespace config {
#ifdef WIN32
        inline const path_t xplane_executable("X-Plane.exe");
#else
#error "xplane_executable is not defined on this platform!"
#endif

        extern const Version xpat_lib_version;

     
    }
}

namespace std {
    inline ostream& operator<<(ostream& os, const xpat::Version& ver) {
        return os << ver.str();
    }
}