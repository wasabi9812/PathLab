// src/io/scen_loader.cpp
#include "pathlab/io/scen_loader.hpp"
#include <fstream>
#include <sstream>

namespace pathlab {

bool ScenarioLoader::load_from_file(const std::string& filepath) {
    std::ifstream in(filepath);
    if (!in.is_open()) return false;

    scenarios_.clear();
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == 'v' || line[0] == 't') continue; // 헤더 스킵

        std::istringstream iss(line);
        int bucket, map_w, map_h, sx, sy, gx, gy;
        double opt;
        std::string mapfile;
        if (iss >> bucket >> mapfile >> map_w >> map_h >> sx >> sy >> gx >> gy >> opt) {
            Scenario s;
            s.start = {sx, sy};
            s.goal  = {gx, gy};
            s.optimal_length = opt;
            s.map_name = mapfile;
            scenarios_.push_back(s);
        }
    }
    return true;
}

} // namespace pathlab
