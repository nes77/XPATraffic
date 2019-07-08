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

#include <libxpat/data/StackString.hpp>
#include <benchmark/benchmark.h>
#include <units.h>
#include <algorithm>
#include <random>
#include <atomic>
#include <string>

using namespace xpat::data;
static std::random_device r;
static std::default_random_engine rand_base(r());

static StackString8 generate_random_string() {
    const std::uniform_int_distribution<short> rand('a', 'z');
    char out_base[StackString8::size] = { 0 };

    std::generate_n(out_base, StackString8::size - 1, [&rand]() -> char {return char(rand(rand_base) & 0xFF); });
    return StackString8(out_base);
}

static void BM_StackStringConstruction(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_string();
        state.ResumeTiming();

        benchmark::DoNotOptimize(StackString8(a.c_str()));
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_StdStringConstruction(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_string();
        state.ResumeTiming();

        benchmark::DoNotOptimize(std::string(a.c_str()));
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_StackStringHash(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_string();
        auto hash = std::hash<StackString8>();
        state.ResumeTiming();

        benchmark::DoNotOptimize(hash(a));
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_StdStringHash(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_string();
        auto s = std::string(a.c_str());
        auto hash = std::hash<std::string>();
        state.ResumeTiming();

        benchmark::DoNotOptimize(hash(s));
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_StackStringEq(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_string();
        state.ResumeTiming();

        benchmark::DoNotOptimize(a == a);
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_StdStringEq(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = generate_random_string();
        auto s = std::string(a.c_str());
        state.ResumeTiming();

        benchmark::DoNotOptimize(s == s);
    }
    state.SetItemsProcessed(state.iterations());
}

static void BM_StackStringAtomic(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto a = std::atomic<StackString8>(generate_random_string());
        auto b = generate_random_string();
        state.ResumeTiming();

        benchmark::DoNotOptimize(a.exchange(b));
    }
    state.SetItemsProcessed(state.iterations());
}

BENCHMARK(BM_StackStringConstruction);
BENCHMARK(BM_StdStringConstruction);
BENCHMARK(BM_StackStringHash);
BENCHMARK(BM_StdStringHash);
BENCHMARK(BM_StackStringEq);
BENCHMARK(BM_StdStringEq);
BENCHMARK(BM_StackStringAtomic);