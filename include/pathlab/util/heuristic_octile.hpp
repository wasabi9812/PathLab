#pragma once
#include <algorithm>
#include <cmath>
#include "pathlab/util/heuristic_base.hpp"

namespace pathlab {

// 8방 격자: 직교=1, 대각=√2
inline double h_octile(int x1,int y1,int x2,int y2){
    int dx = std::abs(x1-x2), dy = std::abs(y1-y2);
    const double SQRT2 = std::sqrt(2.0);
    int m = std::min(dx, dy);
    return (dx + dy) + (SQRT2 - 2.0) * m;
}

} // namespace pathlab
