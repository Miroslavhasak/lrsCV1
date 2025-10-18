#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <thread>
#include <vector>
#include <array>

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node
{
public:
    DroneController(const std::vector<std::array<double,3>>& path)
        : Node("drone_controller"), path_(path)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        RCLCPP_INFO(this->get_logger(), "DroneController spusteny.");

        // vzlet
        takeoff(1.0);  // vyska vzletu 1 meter
        
        // natočenie drona o 90 stupnov
	rotate(M_PI/2);

        // let po trajektorii
        fly_path();

        // pristatie
        land();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::vector<std::array<double,3>> path_;

    void takeoff(double z)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Vzlet do vysky %.2f m", z);

        for(int i=0; i<50; i++)  // publikuj niekolko krat pre PX4/MAVROS setpoint stream
        {
            pose.header.stamp = this->now();
            pose_pub_->publish(pose);
            std::this_thread::sleep_for(100ms);
        }
    }

    void fly_path()
    {
        RCLCPP_INFO(this->get_logger(), "Let po trajektorii: %zu bodov", path_.size());
        for(const auto& p : path_)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = p[0];
            pose.pose.position.y = p[1];
            pose.pose.position.z = p[2];
            pose.pose.orientation.w = 1.0;

            pose_pub_->publish(pose);
            RCLCPP_INFO(this->get_logger(), "Bod: x=%.2f y=%.2f z=%.2f", p[0], p[1], p[2]);

            std::this_thread::sleep_for(200ms);  // interval medzi bodmi
        }
    }

    void land()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Pristatie");

        for(int i=0; i<50; i++)
        {
            pose.header.stamp = this->now();
            pose_pub_->publish(pose);
            std::this_thread::sleep_for(100ms);
        }
    }
    
    void rotate(double yaw_rad)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";

      // zachovame poziciu drona ako pri vzlete
      pose.pose.position.x = 0.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 1.0;  // dron uz je vo vzduchu

      // nastavenie orientacie pomocou yaw
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw_rad);  // roll, pitch, yaw
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      RCLCPP_INFO(this->get_logger(), "Natočenie drona o %.2f radianov", yaw_rad);

      for(int i=0; i<50; i++)
      {
        pose.header.stamp = this->now();
        pose_pub_->publish(pose);
        std::this_thread::sleep_for(100ms);
      }
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // tu by si mal predat vyhladenu trajektoriu z planner node
    // pre testovaciu verzia zadaj minimalnu trajektoriu
    std::vector<std::array<double,3>> test_path = {{0,0,1},{1,1,1},{2,1,1}};

    auto node = std::make_shared<DroneController>(test_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

