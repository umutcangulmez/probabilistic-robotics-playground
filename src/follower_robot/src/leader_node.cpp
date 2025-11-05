#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include <fstream>
#include <random>

using namespace std::chrono_literals;

class LeaderNode : public rclcpp::Node
{
public:
    LeaderNode() : Node("leader_node"), t_(0.0)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/leader/cmd_vel", 10);

        // Timer for movement
        timer_ = this->create_wall_timer(100ms, std::bind(&LeaderNode::timer_callback, this));

        // Spawn robot after 1 second to let Gazebo start
        spawn_timer_ = this->create_wall_timer(1s, std::bind(&LeaderNode::spawn_robot, this));
    }

private:
    void spawn_robot()
    {
        spawn_timer_->cancel();

        auto client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        if (!client->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Spawn service not available");
            return;
        }

        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "leader";
        request->robot_namespace = "leader";
        request->xml = read_model_sdf("/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf");
        request->initial_pose.position.x = 0.0;
        request->initial_pose.position.y = 0.0;
        request->initial_pose.position.z = 0.01;

        auto result = client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Spawn request sent for leader");
    }

    std::string read_model_sdf(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open SDF file: %s", path.c_str());
            return "";
        }
        std::string line, content;
        while (std::getline(file, line)) content += line + "\n";
        return content;
    }

    void timer_callback()
    {
        static auto start = this->now();
        double t = (this->now() - start).seconds();
        double cycle = std::fmod(t, 16.0);
        geometry_msgs::msg::Twist cmd;
        if (cycle < 5.0) {
            cmd.linear.x = 0.1; cmd.angular.z = 0.0;
        } else if (cycle < 8.0) {
            cmd.linear.x = 0.0; cmd.angular.z = 0.2;
        } else if (cycle < 13.0) {
            cmd.linear.x = 0.1; cmd.angular.z = 0.2;
        } else {
            cmd.linear.x = 0.0; cmd.angular.z = 0.0;
        }


        // actuator noise (Gaussian)
        static std::default_random_engine gen((unsigned)std::chrono::system_clock::now().time_since_epoch().count());
        static std::normal_distribution<double> lv_noise(0.0, 0.01);
        static std::normal_distribution<double> ang_noise(0.0, 0.02);
        cmd.linear.x += lv_noise(gen);
        cmd.angular.z += ang_noise(gen);

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    double t_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LeaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
