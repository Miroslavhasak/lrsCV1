#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <algorithm>

struct VoxelMap {
    int sx, sy, sz;            // rozmery voxel gridu
    double voxel_size;          // rozmer jedneho voxel
    double minx, miny, minz;    // minima sveta
    std::vector<uint8_t> data; // 1 = obsadene, 0 = volne

    inline int idx(int x,int y,int z) const { return (z*sx*sy) + (y*sx) + x; }
    inline bool get(int x,int y,int z) const { return data[idx(x,y,z)] != 0; }
    inline void set(int x,int y,int z, bool v) { data[idx(x,y,z)] = v ? 1 : 0; }
};

// funkcia na prevod PCD cloud -> voxel map
VoxelMap voxel_from_pcd(const std::string &path, double voxel_size);

// funkcia na inflaciu voxel mapy podla polomeru drona
VoxelMap inflate_voxels(const VoxelMap &src, double inflate_m);

#endif

