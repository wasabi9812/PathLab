#pragma once
#include <cstdlib>
#include "pathlab/util/heuristic_base.hpp"

namespace pathlab {

inline double h_manhattan(int x1,int y1,int x2,int y2){
    return std::abs(x1-x2) + std::abs(y1-y2);
}

} // namespace pathlab
