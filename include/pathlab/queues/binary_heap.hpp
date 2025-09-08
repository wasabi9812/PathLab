#pragma once
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <optional>
#include "pathlab/queues/ipriority_queue.hpp"

namespace pathlab {

// Key: 정수 노드ID, Prio: double/float 등
template <class KeyT=int, class PrioT=double>
class BinaryHeap final : public IPriorityQueue<KeyT,PrioT> {
public:
  void push(const KeyT& k, PrioT p) override {
    pq_.emplace(p, k);
    ++pushes_;
  }
  std::optional<KeyT> pop() override {
    if (pq_.empty()) return std::nullopt;
    auto [p,k] = pq_.top(); pq_.pop();
    ++pops_;
    return k;
  }
  bool empty() const override { return pq_.empty(); }
  size_t size() const override { return pq_.size(); }

  uint64_t push_count() const override { return pushes_; }
  uint64_t pop_count() const override { return pops_; }
  void reset_stats() override { pushes_=0; pops_=0; }

private:
  using Item = std::pair<PrioT,KeyT>;
  struct Cmp { bool operator()(const Item&a,const Item&b) const { return a.first > b.first; } };
  std::priority_queue<Item,std::vector<Item>,Cmp> pq_;

  uint64_t pushes_{0}, pops_{0};
};


} // namespace pathlab
