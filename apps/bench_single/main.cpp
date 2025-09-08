#include <iostream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <cstdint>

#include "pathlab/core/grid_map.hpp"
#include "pathlab/io/scen_loader.hpp"
#include "pathlab/algorithms/dijkstra.hpp"
#include "pathlab/algorithms/astar.hpp"
#include "pathlab/util/heuristic_factory.hpp"

static inline bool eq(const std::string& a, const char* b) {
    return a == b;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: bench_single <map_file> <scen_file> [--astar] [--heuristic H] [--no-diag] [--print N] [--limit N]\n";
        std::cerr << "  H: auto|manhattan|octile|euclidean|zero (default: auto)\n";
        return 1;
    }
    std::string map_path  = argv[1];
    std::string scen_path = argv[2];

    // ---- 옵션 파싱 (간단) ----
    bool use_astar   = false;
    bool allow_diag  = true;
    std::string hname = "auto";
    size_t print_first = 5;         // 앞 N개 케이스 로그
    size_t limit_cases = 0;         // 0이면 전부

    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (eq(a, "--astar")) use_astar = true;
        else if (eq(a, "--no-diag")) allow_diag = false;
        else if (eq(a, "--heuristic") && i+1 < argc) { hname = argv[++i]; }
        else if (eq(a, "--print") && i+1 < argc) { print_first = std::stoul(argv[++i]); }
        else if (eq(a, "--limit") && i+1 < argc) { limit_cases = std::stoul(argv[++i]); }
    }

    // ---- 로드 ----
    pathlab::GridMap map;
    if (!map.load_from_file(map_path)) {
        std::cerr << "Failed to load map: " << map_path << "\n";
        return 1;
    }
    std::cout << "Map: " << map.width() << "x" << map.height() << "\n";

    pathlab::ScenarioLoader sl;
    if (!sl.load_from_file(scen_path)) {
        std::cerr << "Failed to load scen: " << scen_path << "\n";
        return 1;
    }
    std::cout << "Scenarios: " << sl.scenarios().size() << "\n";

    // ---- 누적지표 ----
    size_t solved = 0;
    double   sum_cost = 0.0, sum_ms = 0.0;
    uint64_t sum_expanded = 0, sum_pushes = 0, sum_pops = 0;

    // 휴리스틱 (A*일 때만 사용)
    auto H = pathlab::make_heuristic(hname, allow_diag);

    // ---- 실행 ----
    const size_t n_total = sl.scenarios().size();
    const size_t n_run   = (limit_cases == 0 ? n_total : std::min(limit_cases, n_total));

    for (size_t i = 0; i < n_run; ++i) {
        const auto& s = sl.scenarios()[i];

        pathlab::PathResult res;
        if (use_astar) {
            pathlab::AStar ast;
            res = ast.solve(map, s.start.x, s.start.y, s.goal.x, s.goal.y, allow_diag, H);
        } else {
            pathlab::Dijkstra dj;
            res = dj.solve(map, s.start.x, s.start.y, s.goal.x, s.goal.y, allow_diag);
        }

        if (res.found) { ++solved; sum_cost += res.cost; }
        sum_ms       += res.stats.millis;
        sum_expanded += res.stats.expanded;
        sum_pushes   += res.stats.pushes;
        sum_pops     += res.stats.pops;

        if (i < print_first) {
            std::cout << "Case[" << i << "] "
                      << (res.found ? "FOUND" : "FAIL")
                      << " cost="     << std::fixed << std::setprecision(3) << res.cost
                      << " expanded=" << res.stats.expanded
                      << " pushes="   << res.stats.pushes
                      << " pops="     << res.stats.pops
                      << " time_ms="  << std::fixed << std::setprecision(3) << res.stats.millis
                      << "\n";
        }
    }

    // ---- 요약 ----
    const size_t n = n_run;
    std::cout << "\nSummary (" << solved << "/" << n << " solved)"
              << " algo=" << (use_astar ? "astar" : "dijkstra")
              << " heuristic=" << (use_astar ? H.name : std::string("n/a"))
              << " diag=" << (allow_diag ? "on" : "off")
              << " avg_cost="     << (solved ? sum_cost/solved : 0.0)
              << " avg_expanded=" << (n ? (double)sum_expanded/n : 0.0)
              << " avg_pushes="   << (n ? (double)sum_pushes/n   : 0.0)
              << " avg_pops="     << (n ? (double)sum_pops/n     : 0.0)
              << " avg_time_ms="  << (n ? sum_ms/n : 0.0)
              << "\n";

    return 0;
}
