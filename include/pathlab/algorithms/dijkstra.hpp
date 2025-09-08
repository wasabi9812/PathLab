#pragma once
#include <vector>
#include <limits>
#include <chrono>
#include <cmath>
#include "pathlab/algorithms/ipathfinder.hpp"
#include "pathlab/core/grid_map.hpp"
#include "pathlab/queues/binary_heap.hpp"

namespace pathlab {

struct Dijkstra {
  static inline int id(int x, int y, int W) { return y*W + x; }

  PathResult solve(const GridMap& map, int sx, int sy, int gx, int gy, bool allow_diagonal = true) {
    const int W = map.width(), H = map.height();
    PathResult r;

    if (sx<0||sy<0||gx<0||gy<0||sx>=W||gx>=W||sy>=H||gy>=H) return r;
    if (!map.is_free(sx,sy) || !map.is_free(gx,gy)) return r;

    const int N = W*H;
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<int> parent(N, -1);

    BinaryHeap<int,double> open;
    const int sId = id(sx,sy,W), gId = id(gx,gy,W);
    dist[sId] = 0.0;
    open.push(sId, 0.0);

    auto t0 = std::chrono::steady_clock::now();
    uint64_t expanded = 0;

    // 8방향(옥타일). allow_diagonal=false면 앞 4개(직교)만 사용.
    static const int DX[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
    static const int DY[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };
    static const double W_COST[8] = {
      1.0, 1.0, 1.0, 1.0, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)
    };
    const int neighbor_count = allow_diagonal ? 8 : 4;

    while (!open.empty()) {
      auto curOpt = open.pop();
      if (!curOpt) break;
      int u = *curOpt;
      if (u == gId) break;

      int ux = u % W, uy = u / W;
      ++expanded;

      for (int k=0; k<neighbor_count; ++k) {
        int vx = ux + DX[k], vy = uy + DY[k];
        if (vx<0||vy<0||vx>=W||vy>=H) continue;
        if (!map.is_free(vx,vy)) continue;

        // (선택) "corner cutting" 방지: 대각 이동 시 양옆이 벽이면 금지
        if (k >= 4) {
          int ax = ux + DX[k], ay = uy;     // 수평
          int bx = ux,         by = uy+DY[k]; // 수직
          if (!map.is_free(ax, ay) || !map.is_free(bx, by)) continue;
        }

        int v = id(vx,vy,W);
        double nd = dist[u] + W_COST[k];
        if (nd < dist[v]) {
          dist[v] = nd;
          parent[v] = u;
          open.push(v, nd); // decrease_key 없이 중복 허용 (간단구현)
        }
      }
    }

    auto t1 = std::chrono::steady_clock::now();
    r.stats.millis = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.stats.expanded = expanded;

    if (dist[gId] == INF) { r.found=false; return r; }
    r.found = true;
    r.cost = dist[gId];

    std::vector<int> rev;
    for (int v=gId; v!=-1; v=parent[v]) rev.push_back(v);
    r.path.assign(rev.rbegin(), rev.rend());
    return r;
  }
};

} // namespace pathlab
