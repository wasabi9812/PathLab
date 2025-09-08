#pragma once
#include <algorithm>
#include <string>
#include "pathlab/util/heuristic_base.hpp"
#include "pathlab/util/heuristic_manhattan.hpp"
#include "pathlab/util/heuristic_euclidean.hpp"
#include "pathlab/util/heuristic_octile.hpp"

namespace pathlab {

// 열거→Heuristic
inline Heuristic make_heuristic(HeuType t){
    switch(t){
        case HeuType::Zero:      return { h_zero,      "zero"      };
        case HeuType::Manhattan: return { h_manhattan, "manhattan" };
        case HeuType::Euclidean: return { h_euclidean, "euclidean" };
        case HeuType::Octile:    return { h_octile,    "octile"    };
        default:                 return { h_zero,      "zero"      };
    }
}

// 문자열→Heuristic (allow_diagonal에 따라 auto 선택)
inline Heuristic make_heuristic(const std::string& name, bool allow_diagonal){
    std::string s = name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

    if (s=="zero" || s=="none")        return make_heuristic(HeuType::Zero);
    if (s=="manhattan" || s=="l1")     return make_heuristic(HeuType::Manhattan);
    if (s=="euclid" || s=="euclidean" 
        || s=="l2")                    return make_heuristic(HeuType::Euclidean);
    if (s=="octile" || s=="diag")      return make_heuristic(HeuType::Octile);

    // auto / empty → 기본값
    return allow_diagonal ? make_heuristic(HeuType::Octile)
                          : make_heuristic(HeuType::Manhattan);
}

} // namespace pathlab
