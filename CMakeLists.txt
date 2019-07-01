
#   XPATraffic: FOSS ATC for X-Plane
#   Copyright(C) 2019 Nicholas Samson

#   This program is free software : you can redistribute itand /or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.

#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
#   GNU General Public License for more details.

#   You should have received a copy of the GNU General Public License
#   along with this program.If not, see < https://www.gnu.org/licenses/>.

#   Additional permission under GNU GPL version 3 section 7

#   If you modify this Program, or any covered work, by linking or combining
#   it with the X-Plane SDK by Laminar Research (or a modified version of that
#   library), containing parts covered by the terms of the MIT License, the
#   licensors of this Program grant you additional permission to convey the
#   resulting work.
#   {Corresponding Source for a non-source form of such a combination shall
#   include the source code for the parts of the X-Plane SDK by Laminar Research
#   used as well as that of the covered work.}

cmake_minimum_required (VERSION 3.10)

project (XPATraffic VERSION 0.1.0 LANGUAGES CXX)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory("thirdparty/googletest-1.8.1" EXCLUDE_FROM_ALL)
include(GoogleTest)
add_subdirectory("thirdparty/benchmark-master")

enable_testing()

set(XPSDK_DIR "${PROJECT_SOURCE_DIR}/XPSDK301")
set(XPSDK_XPLM_LIBRARIES_DIR "${XPSDK_DIR}/Libraries/Win")
add_library(XPLM_64 STATIC IMPORTED)
add_library(XPWidgets_64 STATIC IMPORTED)
set_property(TARGET XPLM_64 PROPERTY IMPORTED_LOCATION "${XPSDK_XPLM_LIBRARIES_DIR}/XPLM_64.lib")
set_property(TARGET XPWidgets_64 PROPERTY IMPORTED_LOCATION "${XPSDK_XPLM_LIBRARIES_DIR}/XPWidgets_64.lib")
set(XPSDK_XPLM_LIBRARIES XPLM_64 XPWidgets_64)
set(XPSDK_INCLUDE_DIRS "${XPSDK_DIR}/CHeaders/Widgets" "${XPSDK_DIR}/CHeaders/Wrappers" "${XPSDK_DIR}/CHeaders/XPLM")

add_library(xplanemp STATIC IMPORTED)
set_property(TARGET xplanemp PROPERTY IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/lib/xplanemp.lib")



add_library(xpatraffic_core STATIC 
	libxpat/LibMain.cpp
	libxpat/nav_data/NavPoint.cpp
    libxpat/physics/PhysModel.cpp
)

add_library(spdlog STATIC thirdparty/spdlog/spdlog.cpp)
target_include_directories(spdlog PUBLIC thirdparty/spdlog/include)

target_compile_definitions(spdlog PUBLIC SPDLOG_COMPILED_LIB=1)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(SYSTEM "${PROJECT_SOURCE_DIR}/includes")

add_executable(xpatraffic XPATraffic/XPATraffic.cpp)
add_executable(xpat_test 
    test/nav_data.cpp
    test/basic_physics.cpp
)

add_executable(xpat_bench
    bench/nav_bench.cpp
)

if (CMAKE_BUILD_TYPE MATCHES DEBUG)
    set(DEBUG_CXX_FLAGS "-fsanitize=address")
    target_compile_options(xpatraffic_core PUBLIC ${DEBUG_CXX_FLAGS})
endif()

target_include_directories(xpatraffic_core SYSTEM PRIVATE "${spdlog_INCLUDE_DIRECTORIES}")
target_include_directories(xpat_test SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/googletest-1.8.1/include")
target_include_directories(xpat_bench SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/benchmark-master/include")

target_compile_definitions(xpat_bench PUBLIC _CRT_SECURE_NO_WARNINGS=1)

set_property(TARGET xpatraffic xpat_test xpatraffic_core xpat_bench PROPERTY CXX_STANDARD 17)

target_link_libraries(xpatraffic_core PUBLIC ${XPSDK_XPLM_LIBRARIES} xplanemp)
target_link_libraries(xpatraffic_core PRIVATE spdlog)
target_link_libraries(xpatraffic xpatraffic_core)
target_link_libraries(xpat_test gtest_main xpatraffic_core)
target_link_libraries(xpat_bench benchmark_main xpatraffic_core)
gtest_add_tests(TARGET xpat_test)


