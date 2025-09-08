#pragma once
#include <vector>
#include <limits>
#include <chrono>
#include <cmath>
#include "pathlab/algorithms/ipathfinder.hpp"
#include "pathlab/core/grid_map.hpp"
#include "pathlab/util/heuristic_factory.hpp"
#include "pathlab/queues/po_queue.hpp"   // 부분순서 큐

namespace pathlab {

// A* with POQueue (equivalent to reweighted Dijkstra with phi=h, w=1)
class AStarPO {
public:
  PathResult solve(const GridMap& map,
                   int sx, int sy, int gx, int gy,
                   bool allow_diagonal = true,
                   Heuristic H = make_heuristic("auto", /*allow_diagonal=*/true)) {
    PathResult r;

    const int W = map.width(), Ht = map.height();
    if (W<=0 || Ht<=0) return r;
    if (sx<0||sy<0||gx<0||gy<0||sx>=W||gx>=W||sy>=Ht||gy>=Ht) return r;
    if (!map.is_free(sx,sy) || !map.is_free(gx,gy)) return r;

    const int N = W*Ht;
    auto id = [W](int x,int y){ return y*W + x; };
    auto xy = [W](int v){ return std::pair<int,int>{ v%W, v/W }; };

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g(N, INF);
    std::vector<int>    parent(N, -1);
    std::vector<char>   closed(N, 0);

    // 부분순서 큐: (기본 SCALE=1e6, K=256, GRAIN=256)
    POQueue<int, 1000000ULL, 256, 256ULL> open;

    const int sId = id(sx,sy), gId = id(gx,gy);
    g[sId] = 0.0;
    open.push(sId, H.h(sx,sy,gx,gy)); // f(s) = g(s)+h(s) = h(s)

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
      if (closed[u]) continue;      // stale pop
      if (u == gId) break;          // goal pop → 종료 (A*와 동일)
      closed[u] = 1;

      ++expanded;

      auto [ux,uy] = xy(u);
      for (int k=0;k<NB;++k){
        int vx = ux + DX[k], vy = uy + DY[k];
        if (vx<0||vy<0||vx>=W||vy>=Ht) continue;
        if (!map.is_free(vx,vy)) continue;

        // corner-cutting 방지 (대각)
        if (k>=4){
          if (!map.is_free(ux+DX[k], uy) || !map.is_free(ux, uy+DY[k])) continue;
        }

        int v = id(vx,vy);
        if (closed[v]) continue;

        double ng = g[u] + WC[k];
        if (ng < g[v]) {
          g[v] = ng;
          parent[v] = u;
          double f = ng + H.h(vx,vy,gx,gy); // f = g + h  (reweighted Dijkstra key)
          open.push(v, f);
        }
      }
    }

    auto t1 = std::chrono::steady_clock::now();
    r.stats.millis   = std::chrono::duration<double,std::milli>(t1-t0).count();
    r.stats.expanded = expanded;
    r.stats.pushes   = open.push_count();
    r.stats.pops     = open.pop_count();

    if (g[gId] == INF) { r.found=false; return r; }
    r.found = true;
    r.cost  = g[gId];

    // 경로 복원
    std::vector<int> rev;
    for (int v=gId; v!=-1; v=parent[v]) rev.push_back(v);
    r.path.assign(rev.rbegin(), rev.rend());
    return r;
  }
};

} // namespace pathlab
