#include "a_star.hpp"
#include <queue>
#include <limits>
#include <cmath>
#include <array>

struct Node {
    int idx;
    double f;
    bool operator<(const Node &o) const { return f > o.f; } // pre priority_queue
};

AStarPlanner::AStarPlanner(const VoxelMap &vm) : vm_(vm) {}

int to_idx(int x,int y,int z,const VoxelMap &vm) { return z*vm.sx*vm.sy + y*vm.sx + x; }
void from_idx(int idx,int &x,int &y,int &z,const VoxelMap &vm) {
    x = idx % vm.sx;
    int t = idx / vm.sx;
    y = t % vm.sy;
    z = t / vm.sy;
}

bool AStarPlanner::plan(const std::array<double,3>& start_w,
                        const std::array<double,3>& goal_w,
                        std::vector<std::array<double,3>>& out_path) {

    int sx0,sy0,sz0,sx1,sy1,sz1;
    auto world_to_vox = [&](const std::array<double,3>& w,int &ix,int &iy,int &iz){
        ix = std::min(vm_.sx-1,std::max(0,(int)std::floor((w[0]-vm_.minx)/vm_.voxel_size)));
        iy = std::min(vm_.sy-1,std::max(0,(int)std::floor((w[1]-vm_.miny)/vm_.voxel_size)));
        iz = std::min(vm_.sz-1,std::max(0,(int)std::floor((w[2]-vm_.minz)/vm_.voxel_size)));
    };
    world_to_vox(start_w,sx0,sy0,sz0);
    world_to_vox(goal_w,sx1,sy1,sz1);

    if(vm_.get(sx0,sy0,sz0) || vm_.get(sx1,sy1,sz1)) return false;

    int N = vm_.sx*vm_.sy*vm_.sz;
    std::vector<double> g(N,std::numeric_limits<double>::infinity());
    std::vector<int> parent(N,-1);
    std::vector<char> closed(N,0);

    std::priority_queue<Node> open;

    auto heur = [&](int ix,int iy,int iz){
        double dx = (ix-sx1)*vm_.voxel_size;
        double dy = (iy-sy1)*vm_.voxel_size;
        double dz = (iz-sz1)*vm_.voxel_size;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    int start_idx = to_idx(sx0,sy0,sz0,vm_);
    int goal_idx  = to_idx(sx1,sy1,sz1,vm_);

    g[start_idx] = 0.0;
    open.push({start_idx, heur(sx0,sy0,sz0)});

    std::vector<std::array<int,3>> nbrs;
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) for(int dz=-1;dz<=1;++dz)
        if(dx!=0||dy!=0||dz!=0) nbrs.push_back({dx,dy,dz});

    while(!open.empty()){
        Node cur = open.top(); open.pop();
        if(closed[cur.idx]) continue;

        int cx,cy,cz; from_idx(cur.idx,cx,cy,cz,vm_);
        if(cur.idx==goal_idx) break;

        closed[cur.idx]=1;

        for(auto &d: nbrs){
            int nx=cx+d[0], ny=cy+d[1], nz=cz+d[2];
            if(nx<0||ny<0||nz<0||nx>=vm_.sx||ny>=vm_.sy||nz>=vm_.sz) continue;
            if(vm_.get(nx,ny,nz)) continue;

            int nidx = to_idx(nx,ny,nz,vm_);
            double step_cost = std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])*vm_.voxel_size;
            double tg = g[cur.idx] + step_cost;
            if(tg < g[nidx]){
                g[nidx] = tg;
                parent[nidx] = cur.idx;
                double f = tg + heur(nx,ny,nz);
                open.push({nidx,f});
            }
        }
    }

    if(parent[goal_idx]==-1) return false;

    // reconstruct path
    std::vector<int> rev; int cur = goal_idx;
    while(cur!=-1){ rev.push_back(cur); cur=parent[cur]; }
    out_path.clear();
    for(auto it=rev.rbegin(); it!=rev.rend(); ++it){
        int x,y,z; from_idx(*it,x,y,z,vm_);
        double wx = vm_.minx + (x+0.5)*vm_.voxel_size;
        double wy = vm_.miny + (y+0.5)*vm_.voxel_size;
        double wz = vm_.minz + (z+0.5)*vm_.voxel_size;
        out_path.push_back({wx,wy,wz});
    }

    return true;
}

