#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcpputils/filesystem_helper.hpp>

class TrajectoryVisualizerNode : public rclcpp::Node
{
public:
    TrajectoryVisualizerNode()
        : Node("trajectory_visualizer_node")
    {
        this->declare_parameter<std::string>("trajectory_file", "default_trajectory.csv");
        trajectory_file_ = this->get_parameter("trajectory_file").as_string();

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
        file_path_ = csv_folder + "/" + trajectory_file_;
        RCLCPP_INFO(this->get_logger(), "Trajectory file path: %s", file_path_.c_str());

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualized_trajectory", 10);

        if (loadTrajectoryData()) {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&TrajectoryVisualizerNode::publishTrajectory, this));
        } else {
            RCLCPP_ERROR(this->get_logger(), "No trajectory data loaded. Skipping publishing.");
        }
    }

private:
    bool loadTrajectoryData()
    {
        std::ifstream file(file_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory file not found: %s", file_path_.c_str());
            return false;
        }

        std::string line;
        bool is_header = true; 

        while (std::getline(file, line)) {
            if (is_header) {
                is_header = false;
                RCLCPP_INFO(this->get_logger(), "Skipping header line: %s", line.c_str());
                continue;
            }

            std::istringstream ss(line);
            std::string token;
            std::vector<double> row;

            while (std::getline(ss, token, ',')) {
                try {
                    double value = std::stod(token);
                    row.push_back(value);
                } catch (const std::invalid_argument& e) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid number in trajectory file: %s", token.c_str());
                    return false;
                }
            }

            if (row.size() < 8) {
                RCLCPP_ERROR(this->get_logger(), "Invalid row in trajectory data: expected 8 columns, got %zu", row.size());
                return false;
            }

            trajectory_data_.push_back(row);
        }
        return true;
    }

    void publishTrajectory()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        int downsample_factor = 10;

        for (size_t idx = 0; idx < trajectory_data_.size(); idx += downsample_factor) {
            auto row = trajectory_data_[idx];

            double x = row[1], y = row[2], z = row[3];
            double ox = row[4], oy = row[5], oz = row[6], ow = row[7];

            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = idx;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.x = ox;
            marker.pose.orientation.y = oy;
            marker.pose.orientation.z = oz;
            marker.pose.orientation.w = ow;
            marker.scale.x = 0.05;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string trajectory_file_;
    std::string file_path_;
    std::vector<std::vector<double>> trajectory_data_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}