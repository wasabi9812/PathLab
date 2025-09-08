#pragma once
#include <vector>
#include <limits>
#include <chrono>
#include <cmath>
#include "pathlab/algorithms/ipathfinder.hpp"
#include "pathlab/core/grid_map.hpp"
#include "pathlab/queues/po_queue.hpp"

namespace pathlab {

class DijkstraPO {
public:
  PathResult solve(const GridMap& map, int sx, int sy, int gx, int gy, bool allow_diagonal = true) {
    PathResult r;

    const int W = map.width(), H = map.height();
    if (W<=0 || H<=0) return r;
    if (sx<0||sy<0||gx<0||gy<0||sx>=W||gx>=W||sy>=H||gy>=H) return r;
    if (!map.is_free(sx,sy) || !map.is_free(gx,gy)) return r;

    const int N = W*H;
    auto id = [W](int x,int y){ return y*W + x; };
    auto xy = [W](int v){ return std::pair<int,int>{ v%W, v/W }; };

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<int> parent(N, -1);
    std::vector<char> closed(N, 0);

    // ★ 부분순서 큐 사용: K, GRAIN은 상황 맞춰 조정 가능
    POQueue<int, 1000000ULL, 256, 256ULL> open;

    const int sId = id(sx,sy), gId = id(gx,gy);
    dist[sId] = 0.0;
    open.push(sId, 0.0);

    static const int DX[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
    static const int DY[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };
    static const double WC[8] = {
      1.0, 1.0, 1.0, 1.0, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)
    };
    const int NB = allow_diagonal ? 8 : 4;

    auto t0 = std::chrono::steady_clock::now();
    uint64_t expanded = 0;

    while (!open.empty()) {
      int u = *open.pop();
      if (closed[u]) continue;
      if (u == gId) break;
      closed[u] = 1;

      ++expanded;

      auto [ux,uy] = xy(u);
      for (int k=0;k<NB;++k) {
        int vx = ux + DX[k], vy = uy + DY[k];
        if (vx<0||vy<0||vx>=W||vy>=H) continue;
        if (!map.is_free(vx,vy)) continue;

        if (k>=4) { // corner cutting 방지
          if (!map.is_free(ux+DX[k], uy) || !map.is_free(ux, uy+DY[k])) continue;
        }

        int v = id(vx,vy);
        if (closed[v]) continue;

        double nd = dist[u] + WC[k];
        if (nd < dist[v]) {
          dist[v] = nd;
          parent[v] = u;
          open.push(v, nd);
        }
      }
    }

    auto t1 = std::chrono::steady_clock::now();
    r.stats.millis   = std::chrono::duration<double,std::milli>(t1-t0).count();
    r.stats.expanded = expanded;
    r.stats.pushes   = open.push_count();
    r.stats.pops     = open.pop_count();
    // (선택) peak_open: open.peak_size()

    if (dist[gId] == INF) { r.found=false; return r; }
    r.found = true;
    r.cost  = dist[gId];

    std::vector<int> rev;
    for (int v=gId; v!=-1; v=parent[v]) rev.push_back(v);
    r.path.assign(rev.rbegin(), rev.rend());
    return r;
  }
};

} // namespace pathlab
