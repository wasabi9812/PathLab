// include/pathlab/core/grid_map.hpp
#pragma once
#include <string>
#include <vector>

namespace pathlab {

struct Coord {
    int x, y;
};

class GridMap {
public:
    GridMap() = default;
    bool load_from_file(const std::string& filepath);

    bool is_free(int x, int y) const;
    int width() const { return width_; }
    int height() const { return height_; }

private:
    int width_{0}, height_{0};
    std::vector<std::string> grid_; // 원본 라인 저장 ('.', '@', 'T' 등)
};

} // namespace pathlab
