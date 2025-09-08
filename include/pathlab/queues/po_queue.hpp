#pragma once
#include <vector>
#include <cstdint>
#include <optional>
#include <limits>
#include <cmath>
#include <algorithm>
#include "pathlab/queues/ipriority_queue.hpp"

namespace pathlab {

// Partial-Order Queue (windowed bucket queue)
// - 키(거리) 단조 비감소 pop 가정(Dijkstra 계열).
// - 전역 정렬 대신 [base, base+WINDOW) 구간만 K개의 버킷으로 부분정렬.
// - 그 밖의 키는 future에 모아두었다가, active가 비면 새로운 base로 슬라이드하며 한번에 재분배.
// - 실수 prio는 SCALE로 정수화. GRAIN은 버킷 폭(정수 키 단위).
template <class KeyT=int, uint64_t SCALE=1000000ULL, uint32_t K=256, uint64_t GRAIN=256ULL>
class POQueue final : public IPriorityQueue<KeyT,double> {
  static_assert(K >= 2, "K must be >= 2");
public:
  POQueue(){ clear(); }

  void clear() {
    for (auto &b: buckets_) b.clear();
    future_.clear();
    base_ = 0;
    window_ = K * GRAIN; // active window width
    sz_ = 0;
    pushes_ = pops_ = 0;
    peak_ = 0;
    min_future_ = UINT64_MAX;
    cursor_ = 0;
  }

  // --- IPriorityQueue ---
  void push(const KeyT& k, double prio) override {
    uint64_t key = to_int_key(prio);
    if (key < base_) key = base_; // 단조 위반 안전망
    if (key < base_ + window_) {
      const uint64_t offset = key - base_;
      const uint32_t idx = static_cast<uint32_t>(offset / GRAIN);
      buckets_[idx].emplace_back(key, k);
    } else {
      future_.emplace_back(key, k);
      if (key < min_future_) min_future_ = key;
    }
    ++sz_; ++pushes_;
    if (sz_ > peak_) peak_ = sz_;
  }

  std::optional<KeyT> pop() override {
    if (sz_ == 0) return std::nullopt;

    // active window에서 다음 non-empty 버킷 찾기
    while (cursor_ < K && buckets_[cursor_].empty()) ++cursor_;

    if (cursor_ == K) {
      // active가 비었음 → future로부터 윈도우 슬라이드
      if (!refill_from_future()) return std::nullopt; // 정말 비었음
      while (cursor_ < K && buckets_[cursor_].empty()) ++cursor_;
      if (cursor_ == K) return std::nullopt; // 방어
    }

    // LIFO pop (FIFO도 무방)
    auto &b = buckets_[cursor_];
    auto kv = b.back(); b.pop_back();
    --sz_; ++pops_;
    // 현재 버킷이 비면 다음 버킷으로 이동
    if (b.empty()) ++cursor_;
    return kv.second;
  }

  bool empty() const override { return sz_ == 0; }
  size_t size()  const override { return sz_; }

  // --- stats ---
  uint64_t push_count() const override { return pushes_; }
  uint64_t pop_count()  const override { return pops_;  }
  void reset_stats()    override { pushes_ = pops_ = 0; peak_ = sz_; }
  size_t peak_size()    const { return peak_; }
  size_t current_size() const { return sz_; }

private:
  using Pair = std::pair<uint64_t, KeyT>; // (int_key, id)

  std::vector<Pair> buckets_[K];
  std::vector<Pair> future_;
  uint64_t base_{0};
  uint64_t window_{K * GRAIN};
  size_t   sz_{0};
  uint64_t pushes_{0}, pops_{0};
  size_t   peak_{0};
  uint64_t min_future_{UINT64_MAX};
  uint32_t cursor_{0}; // active window 내 버킷 스캔 위치

  static inline uint64_t to_int_key(double prio) {
    if (prio <= 0) return 0;
    long double s = static_cast<long double>(prio) * static_cast<long double>(SCALE);
    if (s > (long double)std::numeric_limits<uint64_t>::max())
      s = (long double)std::numeric_limits<uint64_t>::max();
    return static_cast<uint64_t>(s + 0.5L);
  }

  bool refill_from_future() {
    if (future_.empty()) return false;

    // 새 base = min_future를 GRAIN 경계로 내림
    base_ = (min_future_ / GRAIN) * GRAIN;
    // future 전체를 새 윈도우로 재분배 (넘치면 다시 future로 남김)
    std::vector<Pair> rest; rest.reserve(future_.size());
    for (auto &kv : future_) {
      const uint64_t key = (kv.first < base_ ? base_ : kv.first);
      if (key < base_ + window_) {
        const uint32_t idx = static_cast<uint32_t>((key - base_) / GRAIN);
        buckets_[idx].emplace_back(kv.first, kv.second);
      } else rest.emplace_back(kv);
    }
    future_.swap(rest);
    // future의 새 최소 갱신
    min_future_ = UINT64_MAX;
    for (auto &kv : future_) if (kv.first < min_future_) min_future_ = kv.first;

    cursor_ = 0;
    return true;
  }
};

} // namespace pathlab
