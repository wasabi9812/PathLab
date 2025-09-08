#pragma once
#include <cmath>
#include "pathlab/util/heuristic_base.hpp"

namespace pathlab {

inline double h_euclidean(int x1,int y1,int x2,int y2){
    int dx = x1 - x2, dy = y1 - y2;
    return std::sqrt(double(dx*dx + dy*dy));
}

} // namespace pathlab
