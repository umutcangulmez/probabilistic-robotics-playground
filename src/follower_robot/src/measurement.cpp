#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class DistanceNode : public rclcpp::Node
{
public:
    DistanceNode() : Node("leader_follower_distance_node")
    {
        leader_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/leader/odom", 10,
            std::bind(&DistanceNode::leader_callback, this, std::placeholders::_1));

        follower_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/follower/odom", 10,
            std::bind(&DistanceNode::follower_callback, this, std::placeholders::_1));

        dist_pub_ = this->create_publisher<std_msgs::msg::Float64>("/leader_follower_distance", 10);
    }

private:
    void leader_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        leader_x_ = msg->pose.pose.position.x;
        leader_y_ = msg->pose.pose.position.y;
        leader_ready_ = true;
        compute_and_publish();
    }

    void follower_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        follower_x_ = msg->pose.pose.position.x;
        follower_y_ = msg->pose.pose.position.y;
        follower_ready_ = true;
        compute_and_publish();
    }

    void compute_and_publish()
    {
        if (!leader_ready_ || !follower_ready_) return;

        double dx = leader_x_ - follower_x_;
        double dy = leader_y_ - follower_y_;
        double dist = std::sqrt(dx*dx + dy*dy);

        std_msgs::msg::Float64 msg;
        msg.data = dist;
        dist_pub_->publish(msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_pub_;

    double leader_x_ = 0.0, leader_y_ = 0.0;
    double follower_x_ = 0.0, follower_y_ = 0.0;
    bool leader_ready_ = false, follower_ready_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}
