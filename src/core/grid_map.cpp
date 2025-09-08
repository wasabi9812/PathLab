// src/core/grid_map.cpp
#include "pathlab/core/grid_map.hpp"
#include <fstream>
#include <stdexcept>

namespace pathlab {

    bool GridMap::load_from_file(const std::string& filepath) {
        std::ifstream in(filepath);
        if (!in.is_open()) return false;
    
        grid_.clear();
        std::string line;
        bool map_section = false;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back(); // <-- add this
            if (line == "map") { map_section = true; continue; }
            if (!map_section) continue;
            if (!line.empty()) grid_.push_back(line);
        }
        height_ = (int)grid_.size();
        width_  = height_ ? (int)grid_[0].size() : 0;
        return height_ > 0 && width_ > 0;
    }

bool GridMap::is_free(int x, int y) const {
    if (y < 0 || y >= height_ || x < 0 || x >= width_) return false;
    char c = grid_[y][x];
    return c == '.'; // '.'만 free, '@'나 'T'는 obstacle
}

} // namespace pathlab
