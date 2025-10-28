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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"

using namespace std::chrono_literals;

// ---------------- quaternion <-> yaw helpers ----------------
double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}
// normalize angle to [-pi, pi]
double angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0*M_PI;
    while (d < -M_PI) d += 2.0*M_PI;
    return d;
}

struct Record {
    double x, y, z;
    std::string precision;
    std::string action;
};

// ---------------- Načítanie CSV (opravené) ----------------
std::vector<Record> load_mission(const std::string &file){
    std::vector<Record> records;
    std::ifstream f(file);
    if(!f.is_open()){
        std::cerr << "Nepodarilo sa otvorit CSV: " << file << std::endl;
        return records;
    }
    std::string line;
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        Record r;
        if(!std::getline(ss, token, ',')) continue;
        r.x = std::stod(token);
        if(!std::getline(ss, token, ',')) continue;
        r.y = std::stod(token);
        if(!std::getline(ss, token, ',')) continue;
        r.z = std::stod(token);
        // nasleduju volitelne polia surface a action
        if(!std::getline(ss, r.precision, ',')) r.precision = "";
        if(!std::getline(ss, r.action, ',')) r.action = "";
        records.push_back(r);
    }
    return records;
}

// ---------------- Vyhladenie trajektórie ----------------
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

// ---------------- Hľadanie zodpovedajúceho rekord-u z CSV pre publikovaný bod ----------------
// Hľadáme záznam v mission_points, ktorý má súradnice blízke publikovanému bodu (do tolerance)
int find_matching_mission_index(const std::vector<Record> &mission_points, const std::array<double,3> &pt, double tol=0.5) {
    for(size_t i=0;i<mission_points.size();++i){
        double dx = mission_points[i].x - pt[0];
        double dy = mission_points[i].y - pt[1];
        double dz = mission_points[i].z - pt[2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if(dist <= tol) return static_cast<int>(i);
    }
    return -1;
}

// =====================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("planner_node");

    // 1. Nacitanie PCD mapy
    std::string pcd_file = ament_index_cpp::get_package_share_directory("prehladavaci_algoritmus") + "/maps/map.pcd";
    double voxel_size = 0.2;
    double inflate_radius = 0.4;
    VoxelMap vm = voxel_from_pcd(pcd_file, voxel_size);
    vm = inflate_voxels(vm, inflate_radius);
    RCLCPP_INFO(node->get_logger(), "Voxel map pripraveny: %zu x %zu x %zu",
                static_cast<size_t>(vm.sx), static_cast<size_t>(vm.sy), static_cast<size_t>(vm.sz));

    // 2. Inicializacia A* planneru
    AStarPlanner planner(vm);

    // 3. Nacitanie misie z CSV
    std::string mission_file = "mission_1_all.csv";
    auto mission_points = load_mission(mission_file);
    RCLCPP_INFO(node->get_logger(), "Nacitana misia, pocet waypointov: %zu", mission_points.size());
    if(mission_points.size() < 1){
        RCLCPP_WARN(node->get_logger(), "Misia neobsahuje body.");
        rclcpp::shutdown();
        return 0;
    }

    // 4. Planovanie trajektorie medzi vsetkymi waypointmi
    std::vector<std::array<double,3>> full_path;
    for(size_t i=0; i+1<mission_points.size(); ++i){
        std::array<double,3> start{mission_points[i].x, mission_points[i].y, mission_points[i].z};
        std::array<double,3> goal{mission_points[i+1].x, mission_points[i+1].y, mission_points[i+1].z};
        std::vector<std::array<double,3>> segment;
        if(planner.plan(start, goal, segment)){
            if(!full_path.empty() && !segment.empty() && full_path.back() == segment.front()){
                full_path.insert(full_path.end(), segment.begin()+1, segment.end());
            } else {
                full_path.insert(full_path.end(), segment.begin(), segment.end());
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Path nenajdeny medzi bodmi %zu a %zu", i, i+1);
        }
    }

    // 5. Vyhladenie trajektorie
    double min_dist = 0.1;
    auto smooth_full_path = smooth_path(full_path, min_dist);
    RCLCPP_INFO(node->get_logger(), "Dlzka trajektorie po vyhladeni: %zu", smooth_full_path.size());
    for(const auto &p: smooth_full_path){
        std::cout << "x=" << p[0] << " y=" << p[1] << " z=" << p[2] << std::endl;
    }

    // 6. Publikovanie pre vizualizaciu
    auto path_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("planned_path", 10);
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "map";
    pose_array_msg.header.stamp = node->now();
    for(const auto &p : smooth_full_path){
        geometry_msgs::msg::Pose pose;
        pose.position.x = p[0];
        pose.position.y = p[1];
        pose.position.z = p[2];
        pose.orientation.w = 1.0;
        pose_array_msg.poses.push_back(pose);
    }
    path_pub->publish(pose_array_msg);
    RCLCPP_INFO(node->get_logger(), "Trajektoria publikovana na topic 'planned_path'");

    // --- MAVROS clients a subskripcie ---
    auto mode_client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    auto arm_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto takeoff_client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    auto land_client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    geometry_msgs::msg::PoseStamped current_local_pose;
    bool local_pose_received = false;
    auto local_qos = rclcpp::QoS(10).best_effort();
    auto local_pos_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", local_qos,
        [&current_local_pose, &local_pose_received](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            current_local_pose = *msg;
            local_pose_received = true;
        }
    );

    // cakat na dostupnost služieb MAVROS
    while (!mode_client->wait_for_service(1s) ||
           !arm_client->wait_for_service(1s) ||
           !takeoff_client->wait_for_service(1s))
    {
        RCLCPP_INFO(node->get_logger(), "Čakám na služby MAVROS...");
    }

    // počkaj na prvú lokálnu polohu (dôležité)
    RCLCPP_INFO(node->get_logger(), "Čakám na prvú lokálnu polohu...");
    int wait_pose_tries = 0;
    while (rclcpp::ok() && !local_pose_received && wait_pose_tries < 200) { // ~20s timeout
        rclcpp::spin_some(node);
        rclcpp::sleep_for(100ms);
        ++wait_pose_tries;
    }
    if(!local_pose_received) {
        RCLCPP_WARN(node->get_logger(), "Prvá lokálna poloha neprišla (timeout). Pokračujem, ale misia môže byť nepresná.");
    } else {
        RCLCPP_INFO(node->get_logger(), "Lokálna poloha prijatá.");
    }

    // Nastavenie módu GUIDED
    auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_req->base_mode = 0;
    mode_req->custom_mode = "GUIDED";
    auto mode_future = mode_client->async_send_request(mode_req);
    if (rclcpp::spin_until_future_complete(node, mode_future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Režim GUIDED nastavený (request odoslaný).");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Zlyhalo odoslanie requestu na nastavenie módu.");
    }

    // Armovanie
    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = true;
    auto arm_future = arm_client->async_send_request(arm_req);
    if (rclcpp::spin_until_future_complete(node, arm_future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Arm request odoslaný.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Zlyhalo odoslanie arm requestu.");
    }

    // --- Pošli niekoľko počiatočných setpointov (pomôcka pre offboard/guided) ---
    auto setpoint_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    geometry_msgs::msg::PoseStamped init_sp;
    init_sp.header.frame_id = "map";
    if(local_pose_received) {
        init_sp.pose = current_local_pose.pose;
    } else {
        // fallback: použijeme prvý bod misie
        init_sp.pose.position.x = mission_points[0].x;
        init_sp.pose.position.y = mission_points[0].y;
        init_sp.pose.position.z = mission_points[0].z;
        init_sp.pose.orientation.w = 1.0;
    }
    rclcpp::Rate rate_init(10);
    for(int i=0;i<20 && rclcpp::ok();++i){
        init_sp.header.stamp = node->now();
        setpoint_pub->publish(init_sp);
        rclcpp::spin_some(node);
        rate_init.sleep();
    }

    // Vzlet (prikaz)
    double takeoff_alt = 2.0;
    // pokus najst v misii prvok s action == "takeoff"
    for(const auto &m : mission_points) {
        if(m.action == "takeoff") { takeoff_alt = m.z; break; }
    }
    auto takeoff_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    takeoff_req->min_pitch = 0.0;
    takeoff_req->yaw = 0.0;
    takeoff_req->altitude = takeoff_alt;
    auto takeoff_future = takeoff_client->async_send_request(takeoff_req);
    if (rclcpp::spin_until_future_complete(node, takeoff_future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Takeoff request odoslaný (alt=%.2f).", takeoff_alt);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Chyba pri odosielani takeoff requestu.");
    }

    // pockaj na dosiahnutie vysky s toleranciou
    const double target_alt = takeoff_alt;
    const double alt_tol = 0.15;
    RCLCPP_INFO(node->get_logger(), "Čakám na dosiahnutie výšky %.2f m...", target_alt);
    rclcpp::Rate rate_wait(10);
    int wait_iters = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if(local_pose_received) {
            if(current_local_pose.pose.position.z >= target_alt - alt_tol) {
                RCLCPP_INFO(node->get_logger(), "Dosažená výška: %.2f m", current_local_pose.pose.position.z);
                break;
            }
        }
        rate_wait.sleep();
        ++wait_iters;
        if(wait_iters > 600) { // ~60s timeout
            RCLCPP_WARN(node->get_logger(), "Timeout čakania na výšku, pokračujem do waypointov.");
            break;
        }
    }

    // **********************************************************************************************
    // Po vzlietnutí: postupne posielaj setpointy a čakaj na dosiahnutie každého bodu
    if (!smooth_full_path.empty()) {
        double pos_tol = 0.3;           // celková euklidovská tolerancia
        const double match_tol = 0.5;        // tolerancia pri hľadaní originálneho waypointu v CSV
        int a1 = 0;
        int a2 = 0;
        int a3 = 0;

        rclcpp::Rate sp_rate(20);

        for (size_t idx = 0; idx < smooth_full_path.size() && rclcpp::ok(); ++idx) {
            const auto &p = smooth_full_path[idx];
            geometry_msgs::msg::PoseStamped sp;
            sp.header.frame_id = "map";
            // Tu ponechávam tvoju transformáciu, ak je potrebná uprav ju.
            sp.pose.position.x = -p[1] + 1.5;   // transformácia podľa pôvodného kódu
            sp.pose.position.y = p[0] - 13.6;
            sp.pose.position.z = p[2];
            sp.pose.orientation.w = 1.0;

            RCLCPP_INFO(node->get_logger(), "Waypoint %zu: x=%.2f y=%.2f z=%.2f", idx, sp.pose.position.x, sp.pose.position.y, sp.pose.position.z);

            int stable_count = 0;
            const int stable_needed = 8; // musí byť v tolerancii aspoň 8 iterácií pri 20Hz (~0.4s)
            while (rclcpp::ok()) {
                sp.header.stamp = node->now();
                setpoint_pub->publish(sp);
                rclcpp::spin_some(node);

                if (local_pose_received) {
                    double dx = current_local_pose.pose.position.x - sp.pose.position.x;
                    double dy = current_local_pose.pose.position.y - sp.pose.position.y;
                    double dz = current_local_pose.pose.position.z - sp.pose.position.z;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (dist <= pos_tol) {
                        ++stable_count;
                    } else {
                        stable_count = 0;
                    }

                    if (stable_count >= stable_needed) {
                        RCLCPP_INFO(node->get_logger(), "Dosiahnutý waypoint %zu (dist=%.3f m).", idx, dist);

                        // nájdi, či tento publikovaný bod zodpovedá nejakému originálnemu waypointu z CSV
                        // použijeme pôvodné (x,y,z) bez transformácie - preto hľadáme podľa p (smooth_full_path)
                        int m_i = find_matching_mission_index(mission_points, p, match_tol);
                        if (m_i >= 0) {
                            const Record &wp_raw = mission_points[m_i];
                            const std::string action = wp_raw.action;
                            const std::string precision = wp_raw.precision;
                            if (precision == "soft"){
                                pos_tol = 0.3;
                            }else if(precision == "hard"){
                                pos_tol = 0.1;
                            }

                            if (action.empty() || action == "none") {
                                RCLCPP_INFO(node->get_logger(), "Akcia: none -> pokracujem.");
                            } else if (action == "yaw180" && a1==0) {

                                a1=1;
                                double current_yaw = quaternion_to_yaw(current_local_pose.pose.orientation);
                                double target_yaw = current_yaw + M_PI;
                                // normalizacia
                                while (target_yaw > M_PI) target_yaw -= 2.0*M_PI;
                                while (target_yaw < -M_PI) target_yaw += 2.0*M_PI;
                                geometry_msgs::msg::PoseStamped yaw_sp = sp;
                                yaw_sp.pose.orientation = yaw_to_quaternion(target_yaw);
                                RCLCPP_INFO(node->get_logger(), "Vykonavam yaw180...");
                                rclcpp::Time start = node->now();
                                while (rclcpp::ok()) {
                                    yaw_sp.header.stamp = node->now();
                                    setpoint_pub->publish(yaw_sp);
                                    rclcpp::spin_some(node);
                                    if (local_pose_received) {
                                        double now_yaw = quaternion_to_yaw(current_local_pose.pose.orientation);
                                        double diff = std::fabs(angle_diff(now_yaw, target_yaw));
                                        if (diff < 0.1) {
                                            RCLCPP_INFO(node->get_logger(), "Yaw dokonceny (odchylka %.3f rad).", diff);
                                            break;
                                        }
                                    }
                                    if ((node->now() - start).seconds() > 20.0) {
                                        RCLCPP_WARN(node->get_logger(), "Yaw timeout, pokracujem dalej.");
                                        break;
                                    }
                                    sp_rate.sleep();
                                } 
                            } else if (action == "landtakeoff" && a3==0) {
                                a3=1;
                                // pristat
                                auto land_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                                RCLCPP_INFO(node->get_logger(), "LandTakeoff: odosielam land...");
                                auto land_future = land_client->async_send_request(land_req);
                                if (rclcpp::spin_until_future_complete(node, land_future) == rclcpp::FutureReturnCode::SUCCESS) {
                                    RCLCPP_INFO(node->get_logger(), "Land request odoslany.");
                                }
                                // pockaj na pristatie
                                RCLCPP_INFO(node->get_logger(), "Cakam na pristatie (alt < 0.2 m)...");
                                rclcpp::Time start = node->now();
                                while (rclcpp::ok()) {
                                    rclcpp::spin_some(node);
                                    if (local_pose_received && current_local_pose.pose.position.z < 0.2) break;
                                    if ((node->now() - start).seconds() > 30.0) {
                                        RCLCPP_WARN(node->get_logger(), "Land timeout v landtakeoff.");
                                        break;
                                    }
                                    rclcpp::sleep_for(200ms);
                                }
                                // Nastavenie módu GUIDED
                                auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                                mode_req->base_mode = 0;
                                mode_req->custom_mode = "GUIDED";
                                auto mode_future = mode_client->async_send_request(mode_req);
                                if (rclcpp::spin_until_future_complete(node, mode_future) == rclcpp::FutureReturnCode::SUCCESS) {
                                    RCLCPP_INFO(node->get_logger(), "Režim GUIDED nastavený (request odoslaný).");
                                } else {
                                    RCLCPP_ERROR(node->get_logger(), "Zlyhalo odoslanie requestu na nastavenie módu.");
                                }

                                // Armovanie
                                auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                                arm_req->value = true;
                                auto arm_future = arm_client->async_send_request(arm_req);
                                if (rclcpp::spin_until_future_complete(node, arm_future) == rclcpp::FutureReturnCode::SUCCESS) {
                                    RCLCPP_INFO(node->get_logger(), "Arm request odoslaný.");
                                } else {
                                    RCLCPP_ERROR(node->get_logger(), "Zlyhalo odoslanie arm requestu.");
                                }

                                // --- Pošli niekoľko počiatočných setpointov (pomôcka pre offboard/guided) ---
                                auto setpoint_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
                                geometry_msgs::msg::PoseStamped init_sp;
                                init_sp.header.frame_id = "map";
                                if(local_pose_received) {
                                    init_sp.pose = current_local_pose.pose;
                                } else {
                                    // fallback: použijeme prvý bod misie
                                    init_sp.pose.position.x = mission_points[0].x;
                                    init_sp.pose.position.y = mission_points[0].y;
                                    init_sp.pose.position.z = mission_points[0].z;
                                    init_sp.pose.orientation.w = 1.0;
                                }
                                rclcpp::Rate rate_init(10);
                                for(int i=0;i<20 && rclcpp::ok();++i){
                                    init_sp.header.stamp = node->now();
                                    setpoint_pub->publish(init_sp);
                                    rclcpp::spin_some(node);
                                    rate_init.sleep();
                                }
                                for(const auto &m : mission_points) {
                                    if(m.action == "takeoff") { takeoff_alt = m.z; break; }
                                }
                                auto takeoff_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                                takeoff_req->min_pitch = 0.0;
                                takeoff_req->yaw = 0.0;
                                takeoff_req->altitude = takeoff_alt;
                                auto takeoff_future = takeoff_client->async_send_request(takeoff_req);
                                if (rclcpp::spin_until_future_complete(node, takeoff_future) == rclcpp::FutureReturnCode::SUCCESS) {
                                    RCLCPP_INFO(node->get_logger(), "Takeoff request odoslaný (alt=%.2f).", takeoff_alt);
                                } else {
                                    RCLCPP_ERROR(node->get_logger(), "Chyba pri odosielani takeoff requestu.");
                                }

                                // pockaj na dosiahnutie vysky s toleranciou
                                const double target_alt = takeoff_alt;
                                const double alt_tol = 0.15;
                                RCLCPP_INFO(node->get_logger(), "Čakám na dosiahnutie výšky %.2f m...", target_alt);
                                rclcpp::Rate rate_wait(10);
                                int wait_iters = 0;
                            } 
                        } else {
                            // nenajdeny zodpovedajuci rekord v CSV
                            RCLCPP_DEBUG(node->get_logger(), "Žiadna CSV akcia pre tento bod (idx=%zu).", idx);
                        }
                        break; // prejdeme na dalsi waypoint
                    }
                }
                sp_rate.sleep();
            } // koniec while (cakanie na waypoint)
        } // koniec for vsetky waypointy
        RCLCPP_INFO(node->get_logger(), "Trajektória dokončená – všetky body spracované.");
        auto land_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        RCLCPP_INFO(node->get_logger(), "Odosielam prikaz land...");
        auto land_future = land_client->async_send_request(land_req);
        if (rclcpp::spin_until_future_complete(node, land_future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Land request odoslany.");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Chyba pri odosielani land requestu.");
        }
        // pockaj na pristatie
        RCLCPP_INFO(node->get_logger(), "Cakam na pristatie (alt < 0.2 m)...");
        rclcpp::Time start = node->now();
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (local_pose_received && current_local_pose.pose.position.z < 0.2) {
                RCLCPP_INFO(node->get_logger(), "Dron pristal (alt=%.2f).", current_local_pose.pose.position.z);
                break;
            }
            if ((node->now() - start).seconds() > 30.0) {
                RCLCPP_WARN(node->get_logger(), "Land timeout, pokracujem.");
                break;
            }
            rclcpp::sleep_for(200ms);
        }    
    } else {
        RCLCPP_WARN(node->get_logger(), "Nemám žiadne body trajektórie.");
    }

    rclcpp::shutdown();
    return 0;
}
