#include <iostream>
#include <string>
#include <iomanip>
#include "pathlab/core/grid_map.hpp"
#include "pathlab/io/scen_loader.hpp"
#include "pathlab/algorithms/dijkstra.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: bench_single <map_file> <scen_file>\n";
        return 1;
    }
    std::string map_path = argv[1];
    std::string scen_path = argv[2];

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

    pathlab::Dijkstra dj;

    size_t solved = 0;
    double sum_cost = 0.0, sum_ms = 0.0;
    uint64_t sum_expanded = 0;

    for (size_t i=0; i<sl.scenarios().size(); ++i) {
        const auto& s = sl.scenarios()[i];
        auto res = dj.solve(map, s.start.x, s.start.y, s.goal.x, s.goal.y, /*allow_diagonal=*/true);
        if (res.found) {
            ++solved;
            sum_cost += res.cost;
        }
        sum_ms += res.stats.millis;
        sum_expanded += res.stats.expanded;

        if (i < 5) { // 앞 몇 케이스만 로그
            std::cout << "Case[" << i << "] "
                      << (res.found ? "FOUND" : "FAIL")
                      << " cost=" << std::fixed << std::setprecision(3) << res.cost
                      << " expanded=" << res.stats.expanded
                      << " time_ms=" << std::fixed << std::setprecision(3) << res.stats.millis
                      << "\n";
        }
    }

    size_t n = sl.scenarios().size();
    std::cout << "\nSummary (" << solved << "/" << n << " solved)"
              << " avg_cost=" << (solved ? sum_cost/solved : 0.0)
              << " avg_expanded=" << (n ? (double)sum_expanded/n : 0.0)
              << " avg_time_ms=" << (n ? sum_ms/n : 0.0)
              << "\n";

    return 0;
}
