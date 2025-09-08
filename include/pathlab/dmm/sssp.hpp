#pragma once
#include <vector>
#include <limits>
#include <chrono>
#include <cmath>
#include "pathlab/algorithms/ipathfinder.hpp"
#include "pathlab/core/grid_map.hpp"
#include "pathlab/dmm/efficient_ds.hpp"   // 블록 기반 부분정렬 DS

namespace pathlab::dmm {

// DMM 스타일(전역 힙 X, 블록 단위 부분정렬) SSSP 스켈레톤
// - 정확한 최단경로 유지
// - 큐는 EfficientDataStructure (pull 시 블록만 정렬)
// - allow_diagonal, corner-cutting 처리 동일
class SSSP {
public:
  struct Params {
    size_t block_size = 1024;   // pull할 때 정렬하는 배치(블록) 크기
    double bound = std::numeric_limits<double>::infinity(); // BMSSP 용 상한 (무한이면 전체)
  };

  SSSP() : P() {}                       
  explicit SSSP(const Params& p) : P(p) {}

  pathlab::PathResult solve(const pathlab::GridMap& map,
                            int sx, int sy, int gx, int gy,
                            bool allow_diagonal = true) {
    PathResult r;

    const int W = map.width(), H = map.height();
    if (W<=0 || H<=0) return r;
    if (sx<0||sy<0||gx<0||gy<0||sx>=W||gx>=W||sy>=H||gy>=H) return r;
    if (!map.is_free(sx,sy) || !map.is_free(gx,gy)) return r;

    const int N = W*H;
    auto id = [W](int x,int y){ return y*W + x; };
    auto xy = [W](int v){ return std::pair<int,int>{ v%W, v/W }; };

    const int sId = id(sx,sy), gId = id(gx,gy);
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<double> dist(N, INF);
    std::vector<int>    parent(N, -1);
    std::vector<char>   closed(N, 0);

    // 4/8방 이웃 (MovingAI 표준: 직교=1, 대각=√2)
    static const int DX[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
    static const int DY[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };
    static const double WC[8] = {
      1.0, 1.0, 1.0, 1.0, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)
    };
    const int NB = allow_diagonal ? 8 : 4;

    auto t0 = std::chrono::steady_clock::now();

    // ★ 전역 우선순위큐 대신, 블록 DS 사용
    EfficientDataStructure ds(P.block_size, P.bound);

    dist[sId] = 0.0;
    ds.insert((size_t)sId, 0.0);

    uint64_t expanded = 0;

    while (!ds.is_empty()) {
      // 한 번에 블록 하나만 정렬해서 뽑는다 (부분정렬)
      auto [min_remain, batch] = ds.pull();
      if (batch.empty()) break;

      for (size_t u : batch) {
        if (closed[u]) continue;
        if ((int)u == gId) { closed[u] = 1; goto DONE; }
        closed[u] = 1;
        ++expanded;

        auto [ux,uy] = xy((int)u);
        for (int k=0;k<NB;++k) {
          int vx = ux + DX[k], vy = uy + DY[k];
          if (vx<0||vy<0||vx>=W||vy>=H) continue;
          if (!map.is_free(vx,vy)) continue;

          // 대각선 corner-cutting 방지
          if (k>=4){
            if (!map.is_free(ux+DX[k], uy) || !map.is_free(ux, uy+DY[k])) continue;
          }

          int v = id(vx,vy);
          if (closed[v]) continue;

          double nd = dist[u] + WC[k];
          if (nd < dist[v]) {
            dist[v] = nd;
            parent[v] = (int)u;
            // 전역 힙 대신 배치 컨테이너에 삽입
            if (nd < P.bound) ds.insert((size_t)v, nd);
          }
        }
      }
    }

DONE:
    auto t1 = std::chrono::steady_clock::now();
    r.stats.millis   = std::chrono::duration<double,std::milli>(t1-t0).count();
    r.stats.expanded = expanded;

    if (dist[gId] == INF) { r.found=false; return r; }
    r.found = true;
    r.cost  = dist[gId];

    // 경로 복원
    std::vector<int> rev;
    for (int v=gId; v!=-1; v=parent[v]) rev.push_back(v);
    r.path.assign(rev.rbegin(), rev.rend());
    return r;
  }

private:
  Params P;
};

} // namespace pathlab::dmm
