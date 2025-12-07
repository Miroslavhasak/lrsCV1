#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

double dist_sq(double x1, double y1, double z1, double x2, double y2, double z2) {
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);
}

double get_yaw(const geometry_msgs::msg::PoseStamped& pose) {
    double qx = pose.pose.orientation.x;
    double qy = pose.pose.orientation.y;
    double qz = pose.pose.orientation.z;
    double qw = pose.pose.orientation.w;
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

struct MissionPoint {
    double x, y, z;
    std::string type;
    std::string command;
};

class UAV {
public:
    UAV(rclcpp::Node* node, int id, const std::vector<MissionPoint>& mission) 
        : node_(node), id_(id), mission_(mission) {
        
        namespace_ = "/drone" + std::to_string(id_);
        
        // Kompenzácia pre Globálnu dráhu
        physical_start_offset_x_ = 0.0;
        if (id == 2) physical_start_offset_x_ = -1.0;
        if (id == 3) physical_start_offset_x_ = -2.0;

        last_pose_time_ = node_->now();
        current_wp_index_ = 0;
        mission_started_ = false;
        mission_finished_ = false;
        takeoff_done_ = false;
        is_taking_off_ = false; 
        is_holding_ = false;
        returning_home_ = false; // Nový stav
        
        local_pose_.pose.position.x = 0; local_pose_.pose.position.y = 0; local_pose_.pose.position.z = 0;
        local_pose_.pose.orientation.w = 1.0;

        auto qos_state = rclcpp::SystemDefaultsQoS();
        qos_state.keep_last(10);
        auto qos_pose = rclcpp::SensorDataQoS();

        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
            namespace_ + "/state", qos_state, std::bind(&UAV::state_cb, this, _1));

        local_pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            namespace_ + "/local_position/pose", qos_pose, std::bind(&UAV::local_pos_cb, this, _1));

        local_pos_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            namespace_ + "/setpoint_position/local", 10);

        arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(namespace_ + "/cmd/arming");
        set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(namespace_ + "/set_mode");
        takeoff_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>(namespace_ + "/cmd/takeoff");
        land_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>(namespace_ + "/cmd/land");
        
        current_state_.connected = false;
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        local_pose_ = *msg;
        last_pose_time_ = node_->now(); 
    }

    geometry_msgs::msg::PoseStamped get_global_pose() {
        geometry_msgs::msg::PoseStamped global = local_pose_;
        global.pose.position.x += physical_start_offset_x_;
        return global;
    }

    bool check_connection_status(double timeout_sec) {
        if ((node_->now() - last_pose_time_).seconds() > timeout_sec) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] CONNECTION LOST!", namespace_.c_str());
            return false;
        }
        return true;
    }

    bool is_connected() const { return current_state_.connected; }
    std::string get_namespace() const { return namespace_; }
    bool is_takeoff_complete() const { return takeoff_done_ && !is_taking_off_; }
    bool is_mission_complete() const { return mission_finished_; }
    bool is_returning_home() const { return returning_home_; }

    void update_takeoff_logic() {
        if (takeoff_done_) {
            if (is_taking_off_) {
                if (local_pose_.pose.position.z >= 1.5) {
                    is_taking_off_ = false;
                    RCLCPP_INFO(node_->get_logger(), "[%s] Altitude OK. Ready for mission.", namespace_.c_str());
                }
            }
            return;
        }
        takeoff_done_ = true;
        is_taking_off_ = true;
        RCLCPP_INFO(node_->get_logger(), "[%s] Taking off...", namespace_.c_str());

        std::thread([this]() {
            set_mode("GUIDED");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            arm_drone(true);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            takeoff_drone(2.5);
        }).detach();
    }

    void hold_position() {
        if (!is_holding_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] TRAFFIC CONTROL: Holding position.", namespace_.c_str());
            is_holding_ = true;
        }
        local_pos_pub_->publish(local_pose_);
    }

    void update_mission_logic() {
        if (is_taking_off_ || mission_finished_) return;

        if (is_holding_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] Clear. Continuing.", namespace_.c_str());
            is_holding_ = false;
        }

        // --- KONIEC MISIE -> NÁVRAT DOMOV ---
        if (current_wp_index_ >= (int)mission_.size()) {
            if (!returning_home_) {
                returning_home_ = true;
                RCLCPP_INFO(node_->get_logger(), "[%s] Mission points done. Returning to START.", namespace_.c_str());
            }

            // Let na LOKÁLNU 0,0 (Miesto, kde sa zapol)
            publish_setpoint_local(0.0, 0.0, 2.0);

            // Kontrola či sme doma
            double dist_home = std::sqrt(
                std::pow(local_pose_.pose.position.x, 2) + 
                std::pow(local_pose_.pose.position.y, 2));

            if (dist_home < 0.3) {
                perform_land();
                mission_finished_ = true;
            }
            return;
        }

        if (!mission_started_) {
             mission_started_ = true;
             while(current_wp_index_ < (int)mission_.size() && mission_[current_wp_index_].command.find("takeoff") != std::string::npos) {
                 current_wp_index_++;
             }
        }

        // Spracovanie bodu (Globalna misia)
        MissionPoint target = mission_[current_wp_index_];
        
        // Prepočet: Global -> Local
        double target_local_x = target.x - physical_start_offset_x_;
        double target_local_y = target.y; 
        double target_local_z = target.z;

        publish_setpoint_local(target_local_x, target_local_y, target_local_z, target.command);

        double dist = std::sqrt(dist_sq(target_local_x, target_local_y, target_local_z, 
                                        local_pose_.pose.position.x, 
                                        local_pose_.pose.position.y, 
                                        local_pose_.pose.position.z));

        bool yaw_reached = true;
        double required_yaw = 0.0;
        if (target.command.find("yaw180") != std::string::npos) required_yaw = M_PI;
        
        double current_yaw = get_yaw(local_pose_);
        if (std::abs(normalize_angle(current_yaw - required_yaw)) > 0.2) yaw_reached = false;

        if (dist < 0.5 && yaw_reached) {
            RCLCPP_INFO(node_->get_logger(), "[%s] Reached WP %d", namespace_.c_str(), current_wp_index_);
            
            if (target.command.find("land") != std::string::npos && target.command.find("takeoff") == std::string::npos) {
                perform_land();
                mission_finished_ = true;
            } 
            else if (target.command.find("landtakeoff") != std::string::npos) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                current_wp_index_++;
            }
            else {
                current_wp_index_++;
            }
        }
    }

    void perform_land() {
        RCLCPP_WARN(node_->get_logger(), "[%s] Landing...", namespace_.c_str());
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        if(land_client_->service_is_ready()) land_client_->async_send_request(req);
    }

private:
    void publish_setpoint_local(double x, double y, double z, std::string cmd = "") {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node_->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        if (cmd.find("yaw180") != std::string::npos) {
            pose.pose.orientation.z = 1.0; pose.pose.orientation.w = 0.0;
        } else {
            pose.pose.orientation.z = 0.0; pose.pose.orientation.w = 1.0;
        }
        local_pos_pub_->publish(pose);
    }

    void set_mode(std::string mode) {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = mode;
        if(set_mode_client_->service_is_ready()) set_mode_client_->async_send_request(req);
    }

    void arm_drone(bool arm) {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = arm;
        if(arming_client_->service_is_ready()) arming_client_->async_send_request(req);
    }

    void takeoff_drone(double altitude) {
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        req->altitude = altitude;
        if(takeoff_client_->service_is_ready()) takeoff_client_->async_send_request(req);
    }

    rclcpp::Node* node_;
    int id_;
    std::string namespace_;
    std::vector<MissionPoint> mission_;
    int current_wp_index_;
    bool mission_started_;
    bool mission_finished_;
    bool takeoff_done_;
    bool is_taking_off_; 
    bool is_holding_;
    bool returning_home_; // NOVÁ
    
    double physical_start_offset_x_;
    
    rclcpp::Time last_pose_time_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped local_pose_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
};

class SwarmController : public rclcpp::Node {
public:
    SwarmController() : Node("swarm_control_node") {
        const char* home = std::getenv("HOME");
        std::string home_path = home ? home : "/home/ubuntu";
        std::string filename = home_path + "/LRS-FEI/swarm_ws/src/cpp_swarm_control/mission_1_all.csv";
        
        load_mission(filename);

        uavs_.push_back(std::make_shared<UAV>(this, 1, global_mission_));
        uavs_.push_back(std::make_shared<UAV>(this, 2, global_mission_));
        uavs_.push_back(std::make_shared<UAV>(this, 3, global_mission_));

        timer_ = this->create_wall_timer(
            100ms, std::bind(&SwarmController::control_loop, this));
        
        start_time_ = this->now();
        emergency_mode_ = false;
    }

private:
    void load_mission(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN MISSION FILE");
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> parts;
            while(std::getline(ss, segment, ',')) {
                segment.erase(std::remove(segment.begin(), segment.end(), ' '), segment.end());
                parts.push_back(segment);
            }
            if (parts.size() >= 5) {
                MissionPoint wp;
                try {
                    wp.x = std::stod(parts[0]); wp.y = std::stod(parts[1]); wp.z = std::stod(parts[2]);
                    wp.type = parts[3]; wp.command = parts[4];
                    wp.command.erase(std::remove(wp.command.begin(), wp.command.end(), '\r'), wp.command.end());
                    wp.command.erase(std::remove(wp.command.begin(), wp.command.end(), '\n'), wp.command.end());
                    global_mission_.push_back(wp);
                } catch (...) { }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", global_mission_.size());
    }

    void control_loop() {
        if ((this->now() - start_time_).seconds() < 2.0) return;

        // Safety
        for (auto& uav : uavs_) {
            if (!uav->check_connection_status(5.0)) {
                if (!emergency_mode_) {
                    RCLCPP_ERROR(this->get_logger(), "!!! SAFETY STOP !!!");
                    emergency_mode_ = true;
                }
                for (auto& u : uavs_) u->perform_land();
                return;
            }
        }

        // Connection Check
        bool all_connected = true;
        for (auto& uav : uavs_) if (!uav->is_connected()) all_connected = false;
        if (!all_connected) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for connection...");
            return;
        }

        // 1. Spoločný vzlet
        for (auto& uav : uavs_) uav->update_takeoff_logic();
        if (!uavs_[0]->is_takeoff_complete()) return; 

        // 2. Globálne pozície
        auto p1 = uavs_[0]->get_global_pose();
        auto p2 = uavs_[1]->get_global_pose();
        auto p3 = uavs_[2]->get_global_pose();

        double dist_1_2 = std::sqrt(dist_sq(p1.pose.position.x, p1.pose.position.y, p1.pose.position.z,
                                            p2.pose.position.x, p2.pose.position.y, p2.pose.position.z));
        
        double dist_2_3 = std::sqrt(dist_sq(p2.pose.position.x, p2.pose.position.y, p2.pose.position.z,
                                            p3.pose.position.x, p3.pose.position.y, p3.pose.position.z));

        // 3. Stavy misií
        bool d1_done = uavs_[0]->is_mission_complete();
        bool d2_done = uavs_[1]->is_mission_complete();
        
        // --- OPRAVA: Kontrola, či sa drony vracajú domov ---
        bool d2_going_home = uavs_[1]->is_returning_home();
        bool d3_going_home = uavs_[2]->is_returning_home();

        // 4. Riadenie letu
        
        // DRON 1: Letí vždy
        uavs_[0]->update_mission_logic();

        // DRON 2: Brzdí ak je blízko, ALE NIE ak D1 skončil ALEBO ak D2 už ide domov
        if (dist_1_2 < 2.0 && !d1_done && !d2_going_home) { 
            uavs_[1]->hold_position();
        } else {
            uavs_[1]->update_mission_logic();
        }

        // DRON 3: Brzdí ak je blízko, ALE NIE ak D2 skončil ALEBO ak D3 už ide domov
        if (dist_2_3 < 2.0 && !d2_done && !d3_going_home) {
            uavs_[2]->hold_position();
        } else {
            uavs_[2]->update_mission_logic();
        }
    }

    std::vector<std::shared_ptr<UAV>> uavs_;
    std::vector<MissionPoint> global_mission_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    bool emergency_mode_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmController>());
    rclcpp::shutdown();
    return 0;
}