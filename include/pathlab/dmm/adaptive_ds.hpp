#pragma once
#include <queue>
#include <vector>
#include <utility>
#include <limits>

namespace pathlab::dmm {

struct VertexDistance {
  size_t v;
  double d;
  bool operator>(const VertexDistance& o) const { return d > o.d; }
};

struct AdaptiveDataStructure {
  // min-heap by distance
  std::priority_queue<VertexDistance, std::vector<VertexDistance>, std::greater<VertexDistance>> pq;
  size_t capacity{0};
  double bound{std::numeric_limits<double>::infinity()};

  AdaptiveDataStructure() = default;
  AdaptiveDataStructure(size_t cap, double bnd) : capacity(cap), bound(bnd) {}

  void reset(size_t cap, double bnd) {
    capacity = cap; bound = bnd;
    pq = decltype(pq)();
  }

  void insert(size_t v, double d) {
    if (d < bound && std::isfinite(d)) pq.push({v,d});
  }

  void batch_prepend(std::vector<std::pair<size_t,double>> items) {
    for (auto& it : items) insert(it.first, it.second);
  }

  // pull(): (min_remaining, vertices up to 'capacity')
  std::pair<double, std::vector<size_t>> pull() {
    std::vector<size_t> res; res.reserve(capacity);
    for (size_t i=0; i<capacity && !pq.empty(); ++i) {
      res.push_back(pq.top().v);
      pq.pop();
    }
    double min_remaining = bound;
    if (!pq.empty()) min_remaining = std::min(min_remaining, pq.top().d);
    return { min_remaining, std::move(res) };
  }

  bool is_empty() const { return pq.empty(); }
};

} // namespace pathlab::dmm
