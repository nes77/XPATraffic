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
#include <boost/property_tree/json_parser.hpp>
#include <regex>
#include <string>
#include <sstream>
#include <stdexcept>
#include <fstream>
#include <locale>
#include <codecvt> // TODO: Replace with non-deprecated header when available

using namespace xpat;
namespace bpt = boost::property_tree;
namespace stdfs = std::filesystem;

static const std::regex version_regex(R"((\d+)\.(\d+)\.(\d+)(\-.+)?)", std::regex_constants::ECMAScript | std::regex_constants::optimize);
static constexpr std::size_t MAJOR_INDEX = 1;
static constexpr std::size_t MINOR_INDEX = 2;
static constexpr std::size_t PATCH_INDEX = 3;

const Version config::xpat_lib_version(XPAT_VERSION);

BadVersionString::BadVersionString(std::string s) noexcept : std::invalid_argument(s.append(" is not a valid version string")) {}

Version::Version(const std::string& s) {
    std::smatch match;

    if (!std::regex_match(s, match, version_regex) || match.size() < 3) {
        throw BadVersionString(s);
    }

    try {
        auto maj_match = match[MAJOR_INDEX];
        this->major = decltype(this->major)(std::stoull(maj_match.str()));

        auto min_match = match[MINOR_INDEX];
        this->minor = decltype(this->minor)(std::stoull(min_match.str()));

        auto patch_match = match[PATCH_INDEX];
        this->patch = decltype(this->patch)(std::stoull(patch_match.str()));
    }
    catch (const std::out_of_range&) {
        throw BadVersionString(s);
    }
}

std::string Version::str() const noexcept {
    std::ostringstream oss;

    oss << major << "." << minor << "." << patch;

    return oss.str();
}

Config::Config() noexcept : _version(XPAT_VERSION) {}

Config::Config(const path_t& config_loc) {
    bpt::wptree config_tree;

    std::wifstream config_file(config_loc.wstring());

    bpt::read_json(config_file, config_tree);

    std::locale loc("");

    std::wstring_convert<std::codecvt_utf8<wchar_t> > converter;

    this->set_version(converter.to_bytes(config_tree.get<std::wstring>(L"version")));
    this->set_xplane_base(config_tree.get<std::wstring>(L"xplane_base_dir"));
}

void Config::set_version(Version ver) {
    if (ver > config::xpat_lib_version) {
        std::ostringstream oss;
        oss << "Config version " << ver << " newer than library version " << config::xpat_lib_version << "; are you missing an update?";
        throw InvalidConfig(oss.str());
    }
    this->_version = ver;
}

void Config::set_xplane_base(path_t path) {
    // Make sure the path exists
    auto path_status = stdfs::status(path);

    if (!stdfs::is_directory(path_status)) {
        throw InvalidConfig("xplane_base_dir doesn't point to a directory that exists");
    }

    auto xplane_exe = path / config::xplane_executable;

    auto exe_status = stdfs::status(xplane_exe);

    if (!stdfs::is_regular_file(exe_status)) {
        throw InvalidConfig("xplane_base_dir exists but does not contain the expected X-Plane executable for this platform.");
    }

    this->_xplane_base = path;
}

InvalidConfig::InvalidConfig(const std::string& why) noexcept : std::invalid_argument(std::string("Bad config: ") + why) {}