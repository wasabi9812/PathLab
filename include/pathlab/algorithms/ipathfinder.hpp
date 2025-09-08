#pragma once
#include <vector>
#include <cstdint>

namespace pathlab {

struct SearchStats {
  uint64_t expanded{0};
  uint64_t pushes{0};  
  uint64_t pops{0};    
  double millis{0.0};
};

struct PathResult {
  bool found{false};
  std::vector<int> path;  // 노드ID 나열 (y*W + x)
  double cost{0.0};
  SearchStats stats;
};

} // namespace pathlab
