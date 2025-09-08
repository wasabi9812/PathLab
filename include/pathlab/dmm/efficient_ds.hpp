#pragma once
#include <vector>
#include <deque>
#include <utility>
#include <limits>
#include <algorithm>

namespace pathlab::dmm {

struct EfficientDataStructure {
  using Item = std::pair<size_t, double>; // (vertex, distance)

  std::deque<std::vector<Item>> batch_blocks; // 큐: 미정렬 배치 (먼저 소진)
  std::vector<std::vector<Item>> sorted_blocks; // 스택: 미정렬 배치 (나중 소진)
  size_t block_size{0};
  double bound{std::numeric_limits<double>::infinity()};

  EfficientDataStructure() = default;
  EfficientDataStructure(size_t block, double bnd)
  : block_size(block), bound(bnd) {}

  void reset(size_t block, double bnd) {
    batch_blocks.clear();
    sorted_blocks.clear();
    block_size = block;
    bound = bnd;
  }

  // Rust: insert(vertex, distance)
  void insert(size_t v, double d) {
    if (d >= bound || !std::isfinite(d)) return;
    if (sorted_blocks.empty() || sorted_blocks.back().size() >= block_size) {
      sorted_blocks.emplace_back();
      sorted_blocks.back().reserve(block_size);
    }
    sorted_blocks.back().emplace_back(v, d);
  }

  // Rust: batch_prepend(items)
  void batch_prepend(std::vector<Item> items) {
    if (items.empty()) return;
    batch_blocks.emplace_front(std::move(items));
  }

  // Rust: pull() -> (min_remaining, vertices)
  std::pair<double, std::vector<size_t>> pull() {
    if (!batch_blocks.empty()) {
      auto blk = std::move(batch_blocks.front());
      batch_blocks.pop_front();
      std::sort(blk.begin(), blk.end(),
                [](const Item& a, const Item& b){ return a.second < b.second; });
      std::vector<size_t> vs; vs.reserve(blk.size());
      for (auto& it : blk) vs.push_back(it.first);
      return { peek_min().value_or(bound), std::move(vs) };
    }
    if (!sorted_blocks.empty()) {
      auto blk = std::move(sorted_blocks.back());
      sorted_blocks.pop_back();
      std::sort(blk.begin(), blk.end(),
                [](const Item& a, const Item& b){ return a.second < b.second; });
      std::vector<size_t> vs; vs.reserve(blk.size());
      for (auto& it : blk) vs.push_back(it.first);
      return { peek_min().value_or(bound), std::move(vs) };
    }
    return { bound, {} };
  }

  // Rust: peek_min()
  std::optional<double> peek_min() const {
    double batch_min = std::numeric_limits<double>::infinity();
    for (auto const& blk : batch_blocks) {
      for (auto const& it : blk) batch_min = std::min(batch_min, it.second);
    }
    double sorted_min = std::numeric_limits<double>::infinity();
    for (auto const& blk : sorted_blocks) {
      for (auto const& it : blk) sorted_min = std::min(sorted_min, it.second);
    }
    double m = std::min(batch_min, sorted_min);
    return std::isfinite(m) ? std::optional<double>(m) : std::nullopt;
  }

  bool is_empty() const {
    return batch_blocks.empty() && sorted_blocks.empty();
  }
};

} // namespace pathlab::dmm
