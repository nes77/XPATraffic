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

#include <libxpat/nav_data/NavPoint.hpp>
#include <benchmark/benchmark.h>
#include <units.h>
#include <random>

using namespace xpat::nav;
using namespace xpat::phys;
static std::random_device r;
static std::default_random_engine rand_base(r());

static NavPoint generate_random_np() {
    
    std::uniform_real_distribution<double> rand(0.0, 359.9999);

    return NavPoint(rand(rand_base), rand(rand_base)).normalize_in_place();
}

static void BM_HaversineDistance(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_np();
        auto b = generate_random_np();
        state.ResumeTiming();

        benchmark::DoNotOptimize(a.haversine_distance(b));
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_FloatingAdd(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_np();
        state.ResumeTiming();

        benchmark::DoNotOptimize(a.latitude() + a.longitude());
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_VincentyDistance(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_np();
        auto b = generate_random_np();
        state.ResumeTiming();

        benchmark::DoNotOptimize(a.vincenty_distance(b));
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_EuclideanDistance(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_np();
        auto b = generate_random_np();
        state.ResumeTiming();

        benchmark::DoNotOptimize(a.euclidean_distance(b));
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_Translate(benchmark::State& state) {
    std::uniform_real_distribution<double> rand(0.001, 360.0);
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_np();
        auto dist = nautical_miles(rand(rand_base));
        auto bearing = degrees(rand(rand_base));
        state.ResumeTiming();

        benchmark::DoNotOptimize(a.lateral_translate_in_place(bearing, dist));

    }
    state.SetItemsProcessed(state.iterations());
}


BENCHMARK(BM_HaversineDistance);
BENCHMARK(BM_VincentyDistance);
BENCHMARK(BM_EuclideanDistance);
BENCHMARK(BM_Translate);
BENCHMARK(BM_FloatingAdd);