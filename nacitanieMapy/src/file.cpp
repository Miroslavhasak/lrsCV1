// pcd_map_loader.cpp
// ROS2 node na nacitanie .pcd suboru a publikovanie ako sensor_msgs/PointCloud2
// bez diakritiky v komentoch a v kode

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <filesystem>

using namespace std::chrono_literals;

class PcdMapLoader : public rclcpp::Node
{
public:
    PcdMapLoader()
    : Node("pcd_map_loader")
    {
        // parametre: path k pcd suboru a topic na publikovanie
        this->declare_parameter<std::string>("pcd_path", "map.pcd");
        this->declare_parameter<std::string>("pub_topic", "/map/pointcloud");
        this->declare_parameter<std::string>("frame_id", "map");

        this->get_parameter("pcd_path", pcd_path_);
        this->get_parameter("pub_topic", pub_topic_);
        this->get_parameter("frame_id", frame_id_);

        RCLCPP_INFO(this->get_logger(), "PCD path: %s", pcd_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish topic: %s", pub_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Frame id: %s", frame_id_.c_str());

        // QoS transient_local aby novy subscriber dostal posledny publikovany cloud
        auto qos = rclcpp::QoS(1).transient_local();

        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic_, qos);

        // nalozime pcd a publikujeme ho raz
        if (!load_and_publish()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load and publish PCD.");
        } else {
            RCLCPP_INFO(this->get_logger(), "PCD loaded and published successfully.");
        }

        // ak chceme, aby nod nebolel okamzite, mozeme spinovat
        // alebo ukoncit po publikovani - tu budeme spinovat pre pripadne dalsie prikazy
        timer_ = this->create_wall_timer(1s, [this]() {
            // udrziavame node zivotaschopny - v buducnosti mozes pridat service na reload
        });
    }

private:
    bool load_and_publish()
    {
        // overime existenciu suboru
        if (!std::filesystem::exists(pcd_path_)) {
            RCLCPP_ERROR(this->get_logger(), "PCD file not found: %s", pcd_path_.c_str());
            return false;
        }

        // nacitame PCL cloud (XYZ - uprav podla typu suboru ak treba)
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, pcl_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file or unsupported format.");
            return false;
        }

        // pre konverziu do ROS message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(pcl_cloud, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = frame_id_;

        // publikujeme raz
        pc_pub_->publish(output);

        return true;
    }

    std::string pcd_path_;
    std::string pub_topic_;
    std::string frame_id_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdMapLoader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

