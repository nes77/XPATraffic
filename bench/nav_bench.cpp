

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

        benchmark::DoNotOptimize(a.lateral_translate(bearing, dist));

    }
    state.SetItemsProcessed(state.iterations());
}


BENCHMARK(BM_HaversineDistance);
BENCHMARK(BM_VincentyDistance);
BENCHMARK(BM_EuclideanDistance);
BENCHMARK(BM_Translate);