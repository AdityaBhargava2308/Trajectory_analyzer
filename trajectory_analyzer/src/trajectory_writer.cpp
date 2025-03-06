#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "trajectory_analyzer/srv/save_trajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>
#include "rcpputils/filesystem_helper.hpp"

using namespace std::chrono_literals;

class TrajectorySaverNode : public rclcpp::Node
{
public:
    TrajectorySaverNode()
        : Node("trajectory_saver_node")
    {
        this->declare_parameter("marker_topic", "visualization_marker_array");
        marker_topic_ = this->get_parameter("marker_topic").as_string();

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectorySaverNode::odom_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

        save_trajectory_service_ = this->create_service<trajectory_analyzer::srv::SaveTrajectory>(
            "save_trajectory", std::bind(&TrajectorySaverNode::save_trajectory_callback, this, std::placeholders::_1, std::placeholders::_2));


        start_time_ = this->now();
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&TrajectorySaverNode::publish_marker_array, this));

    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto position = msg->pose.pose.position;
        auto orientation = msg->pose.pose.orientation;
        auto timestamp = (this->now() - start_time_).seconds();
        trajectory_data_.push_back({timestamp, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w});
    }

    void publish_marker_array()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < trajectory_data_.size(); ++i)
        {
            auto& data = trajectory_data_[i];
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = std::get<1>(data);
            marker.pose.position.y = std::get<2>(data);
            marker.pose.position.z = std::get<3>(data);
            marker.pose.orientation.x = std::get<4>(data);
            marker.pose.orientation.y = std::get<5>(data);
            marker.pose.orientation.z = std::get<6>(data);
            marker.pose.orientation.w = std::get<7>(data);
            marker.scale.x = 0.05;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker_array.markers.push_back(marker);
        }
        marker_publisher_->publish(marker_array);
    }

    void save_trajectory_callback(const std::shared_ptr<trajectory_analyzer::srv::SaveTrajectory::Request> request,
                                  std::shared_ptr<trajectory_analyzer::srv::SaveTrajectory::Response> response)
    {
        std::string filename = request->filename;
        double duration = request->duration;

        std::string package_dir = ament_index_cpp::get_package_share_directory("trajectory_analyzer");
        std::string src_dir = package_dir;
        size_t pos = src_dir.find("/install/trajectory_analyzer/share/");
        if (pos != std::string::npos)
        {
            src_dir.replace(pos, std::string("/install/trajectory_analyzer/share/").length(), "/src/");
        }
        std::string csv_folder = src_dir + "/csv";
        if (!rcpputils::fs::exists(csv_folder)) {
            rcpputils::fs::create_directories(csv_folder);
        }


        std::string file_path = csv_folder + "/" + filename;
        double current_time = (this->now() - start_time_).seconds();
        std::vector<std::tuple<double, double, double, double, double, double, double, double>> filtered_data;
        for (const auto& data : trajectory_data_)
        {
            if (current_time - std::get<0>(data) <= duration)
            {
                filtered_data.push_back(data);
            }
        }
        if (filtered_data.empty())
        {
            response->success = false;
            response->message = "No trajectory data available for the requested duration.";
            return;
        }
        try
        {
            std::ofstream csvfile(file_path);
            csvfile << "Timestamp,X,Y,Z,OX,OY,OZ,OW\n";
            for (const auto& data : filtered_data)
            {
                csvfile << std::fixed << std::setprecision(6) << std::get<0>(data) << ","
                        << std::get<1>(data) << "," << std::get<2>(data) << "," << std::get<3>(data) << ","
                        << std::get<4>(data) << "," << std::get<5>(data) << "," << std::get<6>(data) << ","
                        << std::get<7>(data) << "\n";
            }
            response->success = true;
            response->message = "Trajectory saved to " + file_path;
        }
        catch (const std::exception& e)
        {
            response->success = false;
            response->message = "Error saving file: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Error saving file: %s", e.what());
        }
        RCLCPP_INFO(this->get_logger(), "The service has delivered!");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Service<trajectory_analyzer::srv::SaveTrajectory>::SharedPtr save_trajectory_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    std::vector<std::tuple<double, double, double, double, double, double, double, double>> trajectory_data_;
    std::string marker_topic_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectorySaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}