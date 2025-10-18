#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "voxel_map.hpp"
#include <vector>
#include <array>

class AStarPlanner {
public:
    AStarPlanner(const VoxelMap &vm);

    // planuje cestu zo start_world do goal_world
    // vystup: out_path vo world suradniciach
    bool plan(const std::array<double,3>& start_world,
              const std::array<double,3>& goal_world,
              std::vector<std::array<double,3>>& out_path);

private:
    const VoxelMap &vm_;
};

#endif

