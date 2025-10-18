#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "voxel_map.hpp"
#include "a_star.hpp"
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

struct Record {
    double x, y, z;
    std::string surface;
    std::string action;
};

// funkcia na nacitanie CSV misie
std::vector<Record> load_mission(const std::string &file){
    std::vector<Record> records;
    std::ifstream f(file);
    if(!f.is_open()){
        std::cerr << "Nepodarilo sa otvorit CSV: " << file << std::endl;
        return records;
    }

    std::string line;
    while(std::getline(f,line)){
        std::stringstream ss(line);
        std::string token;
        Record r;
        std::getline(ss, token, ','); r.x = std::stod(token);
        std::getline(ss, token, ','); r.y = std::stod(token);
        std::getline(ss, token, ','); r.z = std::stod(token);
        std::getline(ss, r.surface, ',');
        std::getline(ss, r.action, ',');
        records.push_back(r);
    }
    return records;
}

// funkcia na vyhladenie trajektorie
std::vector<std::array<double,3>> smooth_path(const std::vector<std::array<double,3>>& path, double min_dist){
    std::vector<std::array<double,3>> out;
    if(path.empty()) return out;
    out.push_back(path[0]);
    for(size_t i=1; i<path.size(); i++){
        double dx = path[i][0]-out.back()[0];
        double dy = path[i][1]-out.back()[1];
        double dz = path[i][2]-out.back()[2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if(dist >= min_dist) out.push_back(path[i]);
    }
    return out;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("planner_node");

    // 1. Nacitanie PCD mapy
    //std::string pcd_file = "/home/ubuntu/LRS-FEI/src/map.pcd";  // uprav podla tvojho suboru
    std::string pcd_file = ament_index_cpp::get_package_share_directory("prehladavaci_algoritmus") + "/maps/map.pcd";

    double voxel_size = 0.2; // velkost voxel v metroch
    double inflate_radius = 0.4; // inflacia podla drona

    VoxelMap vm = voxel_from_pcd(pcd_file, voxel_size);
    vm = inflate_voxels(vm, inflate_radius);

    RCLCPP_INFO(node->get_logger(), "Voxel map pripraveny: %d x %d x %d", vm.sx, vm.sy, vm.sz);

    // 2. Inicializacia A* planneru
    AStarPlanner planner(vm);

    // 3. Nacitanie misie z CSV
    std::string mission_file = "mission_1_all.csv";
    auto mission_points = load_mission(mission_file);
    RCLCPP_INFO(node->get_logger(), "Nacitana misia, pocet waypointov: %zu", mission_points.size());

    if(mission_points.size() < 2){
        RCLCPP_WARN(node->get_logger(), "Misia ma menej ako 2 body, nemoze sa naplanovat trajektoria.");
        rclcpp::shutdown();
        return 0;
    }

    // 4. Planovanie trajektorie medzi vsetkymi waypointmi
    std::vector<std::array<double,3>> full_path;
    for(size_t i=0; i<mission_points.size()-1; i++){
        std::array<double,3> start{mission_points[i].x, mission_points[i].y, mission_points[i].z};
        std::array<double,3> goal{mission_points[i+1].x, mission_points[i+1].y, mission_points[i+1].z};
        std::vector<std::array<double,3>> segment;
        if(planner.plan(start, goal, segment)){
            full_path.insert(full_path.end(), segment.begin(), segment.end());
        } else {
            RCLCPP_WARN(node->get_logger(), "Path nenajdeny medzi bodmi %zu a %zu", i, i+1);
        }
    }

    // 5. Vyhladenie trajektorie
    double min_dist = 0.1;
    auto smooth_full_path = smooth_path(full_path, min_dist);

    RCLCPP_INFO(node->get_logger(), "Dlzka trajektorie po vyhladeni: %zu", smooth_full_path.size());
    for(auto &p: smooth_full_path){
        std::cout << "x=" << p[0] << " y=" << p[1] << " z=" << p[2] << std::endl;
    }
    
    // 6. Postupne publikovanie bodov pre dron
    auto point_pub = node->create_publisher<geometry_msgs::msg::Pose>("drone_goal", 10);
    rclcpp::Rate rate(5); // publikovanie kazdych 0.2 sekundy

    for(const auto &p : smooth_full_path){
      geometry_msgs::msg::Pose pose;
      pose.position.x = p[0];
      pose.position.y = p[1];
      pose.position.z = p[2];
      pose.orientation.w = 1.0; // jednoducha orientacia (bez rotacie)
    
      point_pub->publish(pose);
      RCLCPP_INFO(node->get_logger(), "Odoslany bod: x=%.2f y=%.2f z=%.2f", p[0], p[1], p[2]);

      rclcpp::spin_some(node);
      rate.sleep();
}


    // 7. Publikovanie trajektorie do ROS2
    auto path_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("planned_path", 10);
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "map";
    pose_array_msg.header.stamp = node->now();

    for(const auto &p : smooth_full_path){
        geometry_msgs::msg::Pose pose;
        pose.position.x = p[0];
        pose.position.y = p[1];
        pose.position.z = p[2];
        pose.orientation.w = 1.0; // jednoduchá orientácia
        pose_array_msg.poses.push_back(pose);
    }

    path_pub->publish(pose_array_msg);
    RCLCPP_INFO(node->get_logger(), "Trajektoria publikovana na topic 'planned_path'");

    rclcpp::shutdown();
    return 0;
}

