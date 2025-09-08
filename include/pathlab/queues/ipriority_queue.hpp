#pragma once
#include <optional>

namespace pathlab {

template <class KeyT, class PrioT>
struct IPriorityQueue {
  virtual ~IPriorityQueue() = default;
  virtual void push(const KeyT& k, PrioT p) = 0;
  virtual std::optional<KeyT> pop() = 0;   // 최소 우선순위 key 반환
  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
};

} // namespace pathlab
