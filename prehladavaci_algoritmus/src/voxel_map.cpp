#include "voxel_map.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>

VoxelMap voxel_from_pcd(const std::string &path, double voxel_size) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(path, cloud) == -1) throw std::runtime_error("pcd load fail");

    double minx=1e9, miny=1e9, minz=1e9;
    double maxx=-1e9, maxy=-1e9, maxz=-1e9;

    for (auto &p: cloud.points) {
        minx = std::min(minx, (double)p.x);
        miny = std::min(miny, (double)p.y);
        minz = std::min(minz, (double)p.z);
        maxx = std::max(maxx, (double)p.x);
        maxy = std::max(maxy, (double)p.y);
        maxz = std::max(maxz, (double)p.z);
    }

    int sx = (int)std::ceil((maxx-minx)/voxel_size) + 1;
    int sy = (int)std::ceil((maxy-miny)/voxel_size) + 1;
    int sz = (int)std::ceil((maxz-minz)/voxel_size) + 1;

    VoxelMap vm; 
    vm.sx = sx; vm.sy = sy; vm.sz = sz; 
    vm.voxel_size = voxel_size; 
    vm.minx = minx; vm.miny = miny; vm.minz = minz;
    vm.data.assign((size_t)sx*sy*sz, 0);

    for (auto &p: cloud.points) {
        int ix = std::min(sx-1, std::max(0, (int)std::floor((p.x-minx)/voxel_size)));
        int iy = std::min(sy-1, std::max(0, (int)std::floor((p.y-miny)/voxel_size)));
        int iz = std::min(sz-1, std::max(0, (int)std::floor((p.z-minz)/voxel_size)));
        vm.set(ix,iy,iz, true);
    }

    return vm;
}

VoxelMap inflate_voxels(const VoxelMap &src, double inflate_m) {
    int sx=src.sx, sy=src.sy, sz=src.sz;
    double vs = src.voxel_size;
    int r = (int)std::ceil(inflate_m / vs);
    VoxelMap out = src;

    for (int x=0;x<sx;++x) for (int y=0;y<sy;++y) for (int z=0;z<sz;++z) {
        if (src.get(x,y,z)) continue;
        bool occ=false;
        for (int dx=-r; dx<=r && !occ; ++dx) for (int dy=-r; dy<=r && !occ; ++dy) for (int dz=-r; dz<=r && !occ; ++dz) {
            int nx=x+dx, ny=y+dy, nz=z+dz;
            if (nx<0||ny<0||nz<0||nx>=sx||ny>=sy||nz>=sz) continue;
            if (src.get(nx,ny,nz)) {
                double dist = std::sqrt(dx*dx+dy*dy+dz*dz)*vs;
                if (dist <= inflate_m) { occ=true; break; }
            }
        }
        if (occ) out.set(x,y,z, true);
    }

    return out;
}

