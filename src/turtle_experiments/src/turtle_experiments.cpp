#include "turtle_experiments.h"
#include <fstream>
#include <chrono>
#include <random>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;


void StatePublisher::publish(){
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp=this->get_clock()->now();

    // Specify joints' name which are defined in the r2d2.urdf.xml and their content
    joint_state.name={"swivel","tilt","periscope"};
    joint_state.position={swivel,tilt,height};

    t.header.stamp=this->get_clock()->now();

    // odom is the base coordinate system of tf2
    t.header.frame_id="odom";
    // axis is defined in r2d2.urdf.xml file and it is the base coordinate of model
    t.child_frame_id="axis";

    // add translation change
    t.transform.translation.x=cos(angle)*2;
    t.transform.translation.y=sin(angle)*2;
    t.transform.translation.z=0.7;
    tf2::Quaternion q;
    // euler angle into Quanternion and add rotation change
    q.setRPY(0,0,angle+M_PI/2);
    t.transform.rotation.x=q.x();
    t.transform.rotation.y=q.y();
    t.transform.rotation.z=q.z();
    t.transform.rotation.w=q.w();

    // update state for next time
    tilt+=tinc;
    if (tilt<-0.5 || tilt>0.0){
        tinc*=-1;
    }
    height+=hinc;
    if (height>0.2 || height<0.0){
        hinc*=-1;
    }
    swivel+=degree;  // Increment by 1 degree (in radians)
    angle+=degree;    // Change angle at a slower pace

    // send message
    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);

    RCLCPP_INFO(this->get_logger(),"Publishing joint state");
}



TurtleExperiments::TurtleExperiments(const rclcpp::NodeOptions & options)
: rclcpp::Node("experiment_runner", options), cmd_v_(0.2), cmd_w_(0.0)
{
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  noisy_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_noisy", 10);



  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TurtleExperiments::odom_callback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TurtleExperiments::scan_callback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&TurtleExperiments::imu_callback, this, std::placeholders::_1));
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&TurtleExperiments::image_callback, this, std::placeholders::_1));

  logfile_path_ = "/tmp/experiment_log.csv";
  {
    std::ofstream ofs(logfile_path_, std::ofstream::out);
    ofs << "time,x,y,yaw,v_cmd,w_cmd\n";
  }
  state_publisher_ = std::make_shared<StatePublisher>();
  timer_ = this->create_wall_timer(100ms, std::bind(&TurtleExperiments::timer_callback, this));
}


void TurtleExperiments::timer_callback()
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

  // temporary
  // cmd.linear.x = 0.0; cmd.angular.z = 0.0;

  // actuator noise (Gaussian)
  static std::default_random_engine gen((unsigned)std::chrono::system_clock::now().time_since_epoch().count());
  static std::normal_distribution<double> lv_noise(0.0, 0.01);
  static std::normal_distribution<double> ang_noise(0.0, 0.02);
  cmd.linear.x += lv_noise(gen);
  cmd.angular.z += ang_noise(gen);

  cmd_pub_->publish(cmd);
}

void TurtleExperiments::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  auto q = msg->pose.pose.orientation;
  double yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));

  std::ofstream ofs(logfile_path_, std::ofstream::app);
  ofs << this->now().seconds() << "," << x << "," << y << "," << yaw << "," << cmd_v_ << "," << cmd_w_ << "\n";
}

void TurtleExperiments::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Make a copy of the incoming scan
  sensor_msgs::msg::LaserScan noisy_scan = *msg;

  // Create random noise generator (static so it's created once)
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::normal_distribution<float> dist(0.0, 0.02); // mean=0, stddev=0.02m

  // Add noise to valid scan ranges
  for (auto &range : noisy_scan.ranges)
  {
    if (std::isfinite(range))
    {
      range += dist(gen);
    }
  }

  // Publish noisy scan
  noisy_scan_pub_->publish(noisy_scan);
}

void TurtleExperiments::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  (void)msg;
}

void TurtleExperiments::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  (void)msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleExperiments>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
