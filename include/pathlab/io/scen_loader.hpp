// include/pathlab/io/scen_loader.hpp
#pragma once
#include <string>
#include <vector>
#include "pathlab/core/grid_map.hpp"

namespace pathlab {

struct Scenario {
    Coord start;
    Coord goal;
    double optimal_length;
    std::string map_name;
};

class ScenarioLoader {
public:
    bool load_from_file(const std::string& filepath);
    const std::vector<Scenario>& scenarios() const { return scenarios_; }

private:
    std::vector<Scenario> scenarios_;
};

} // namespace pathlab
