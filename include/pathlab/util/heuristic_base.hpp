#pragma once
#include <cstdint>
#include <functional>
#include <string>

namespace pathlab {

enum class HeuType : uint8_t {
    Zero,
    Manhattan,
    Euclidean,
    Octile
};

struct Heuristic {
    std::function<double(int,int,int,int)> h; // h(x1,y1,x2,y2)
    std::string name;
};

// base에는 zero만 둡니다 (중복 방지)
inline double h_zero(int,int,int,int){ return 0.0; }

} // namespace pathlab
