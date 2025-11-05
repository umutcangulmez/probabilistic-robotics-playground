#ifndef TURTLE_EXPERIMENTS_EXPERIMENT_RUNNER_HPP_
#define TURTLE_EXPERIMENTS_EXPERIMENT_RUNNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <fstream>
#include <chrono>
#include <random>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options){
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(),"Starting state publisher");

        loop_rate_=std::make_shared<rclcpp::Rate>(33ms);

        timer_=this->create_wall_timer(33ms,std::bind(&StatePublisher::publish,this));
    }

    void publish();
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Rate::SharedPtr loop_rate_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Robot state variables
    // degree means one degree
    const double degree=M_PI/180.0;
    double tilt = 0.;
    double tinc = degree;
    double swivel = 0.;
    double angle = 0.;
    double height = 0.;
    double hinc = 0.005;
};


class TurtleExperiments : public rclcpp::Node {
public:
    explicit TurtleExperiments(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void timer_callback();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr noisy_scan_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    double cmd_v_, cmd_w_;
    std::string logfile_path_;
    std::shared_ptr<StatePublisher> state_publisher_;
};





#endif  // TURTLE_EXPERIMENTS_EXPERIMENT_RUNNER_HPP_
