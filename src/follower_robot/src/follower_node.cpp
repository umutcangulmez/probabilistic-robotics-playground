#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include <fstream>
#include "sensor_msgs/msg/laser_scan.hpp"

#include <visualization_msgs/msg/marker_array.hpp>


using namespace std::chrono_literals;

class FollowerNode : public rclcpp::Node
{
public:
    FollowerNode() : Node("follower_node")
    {
        this->declare_parameter<double>("desired_distance", 1.25);
        this->declare_parameter<double>("stop_distance", 0.75);
        this->declare_parameter<double>("kp_dist", 0.7);
        this->declare_parameter<double>("kp_angle", 1.2);
        this->declare_parameter<double>("max_linear", 0.4);
        this->declare_parameter<double>("max_angular", 1.2);
        this->declare_parameter<double>("smoothing_alpha", 0.2); // EMA alpha
        this->declare_parameter<double>("search_angular_vel", 0.6);
        this->declare_parameter<double>("lost_timeout", 3.0); // seconds
        this->declare_parameter<double>("front_sector_deg", 30.0);
        this->declare_parameter<double>("control_rate_hz", 20.0);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/follower/cmd_vel", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/follower/beam_likelihood_markers", 10);

        // Load params
        this->get_parameters();

        // Subscribe to follower's laser scan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/follower/scan", rclcpp::QoS(10),
          std::bind(&FollowerNode::scan_callback, this, std::placeholders::_1));

        // Control timer
        auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
        timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(period),
          std::bind(&FollowerNode::control_loop, this));

        // Parameter change callback so you can tune at runtime
        param_cb_handle_ = this->add_on_set_parameters_callback(
          std::bind(&FollowerNode::on_parameter_changed, this, std::placeholders::_1));
        // Spawn follower robot after 2 seconds
        spawn_timer_ = this->create_wall_timer(2s, std::bind(&FollowerNode::spawn_robot, this));
        RCLCPP_INFO(this->get_logger(), "Follower node started (listening to /follower/scan)");

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
        request->name = "follower";
        request->robot_namespace = "follower";
        request->xml = read_model_sdf("/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf");
        request->initial_pose.position.x = 2.0;
        request->initial_pose.position.y = 2.0;
        request->initial_pose.position.z = 0.01;

        auto result = client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Spawn request sent for follower");
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


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::mutex scan_mutex_;
    bool have_measurement_ = false;
    double detected_range_ = NAN;
    double detected_angle_ = NAN;
    rclcpp::Time last_detection_time_;

    // Parameters
    double desired_distance_;
    double stop_distance_;
    double kp_dist_;
    double kp_angle_;
    double max_linear_;
    double max_angular_;
    double smoothing_alpha_;
    double search_angular_vel_;
    double lost_timeout_;
    double front_sector_deg_;
    double control_rate_hz_;

    // helper to load parameters
    void get_parameters()
    {
        this->get_parameter("desired_distance", desired_distance_);
        this->get_parameter("stop_distance", stop_distance_);
        this->get_parameter("kp_dist", kp_dist_);
        this->get_parameter("kp_angle", kp_angle_);
        this->get_parameter("max_linear", max_linear_);
        this->get_parameter("max_angular", max_angular_);
        this->get_parameter("smoothing_alpha", smoothing_alpha_);
        this->get_parameter("search_angular_vel", search_angular_vel_);
        this->get_parameter("lost_timeout", lost_timeout_);
        this->get_parameter("front_sector_deg", front_sector_deg_);
        this->get_parameter("control_rate_hz", control_rate_hz_);
    }

    // Parameter update handler
    rcl_interfaces::msg::SetParametersResult on_parameter_changed(
      const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &p : params) {
            try {
                if (p.get_name() == "desired_distance") desired_distance_ = p.as_double();
                else if (p.get_name() == "stop_distance") stop_distance_ = p.as_double();
                else if (p.get_name() == "kp_dist") kp_dist_ = p.as_double();
                else if (p.get_name() == "kp_angle") kp_angle_ = p.as_double();
                else if (p.get_name() == "max_linear") max_linear_ = p.as_double();
                else if (p.get_name() == "max_angular") max_angular_ = p.as_double();
                else if (p.get_name() == "smoothing_alpha") smoothing_alpha_ = p.as_double();
                else if (p.get_name() == "search_angular_vel") search_angular_vel_ = p.as_double();
                else if (p.get_name() == "lost_timeout") lost_timeout_ = p.as_double();
                else if (p.get_name() == "front_sector_deg") front_sector_deg_ = p.as_double();
                else if (p.get_name() == "control_rate_hz") {
                    control_rate_hz_ = p.as_double();
                    // timer rate change would require re-creating timer; ignore for simplicity
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Param set error: %s", e.what());
            }
        }
        return result;
    }


  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(scan_mutex_);
      // --- Beam Model Parameters ---
      constexpr double sigma = 0.2;     // Std dev for hit model
      constexpr double lambda = 1.0;    // Short model decay
      constexpr double w_hit = 0.7;
      constexpr double w_short = 0.1;
      constexpr double w_max = 0.1;
      constexpr double w_rand = 0.1;


      double scan_likelihood = 1.0;

      double r_max = msg->range_max;

      double best_p = 0.0;
      double best_r = NAN;
      double best_angle = NAN;
      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.clear();

      // Scan each beam
      for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double r = msg->ranges[i];

        if (!std::isfinite(r) || r < 0.01) continue;

        double r_star = msg->range_max;  // every obstacle is unexpected since this is a world without obstacles
        // ---- p_hit ----
        double p_hit = 0.0;
        if (r >= 0.0 && r <= r_max)
        {
          double denom = std::sqrt(2 * M_PI * sigma * sigma);
          p_hit = std::exp(-0.5 * std::pow((r - r_max) / sigma, 2)) / denom;
        }

        // ---- p_short ----
        double p_short = 0.0;
        if (r >= 0.0 && r <= r_star)
        {
          p_short = lambda * std::exp(-lambda * r);
        }
        // ---- p_max ----
        double p_max = (r == r_max) ? 1.0 : 0.0;

        // ---- p_rand ----
        double p_rand = (r >= 0.0 && r < r_max) ? (1.0 / r_max) : 0.0;
        // Combine with weights
        double p = w_hit * p_hit +
                   w_short * p_short +
                   w_max * p_max +
                   w_rand * p_rand;

        // Multiply into total scan likelihood
        scan_likelihood *= p;


        if (p > best_p)
        {
          best_p = p;
          best_r = r;
          best_angle = msg->angle_min + i * msg->angle_increment;
        }


        // --- RViz Marker ---
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = this->now();
        marker.ns = "beam_likelihood";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position in polar coordinates -> Cartesian
        marker.pose.position.x = r * std::cos(msg->angle_min + i * msg->angle_increment);
        marker.pose.position.y = r * std::sin(msg->angle_min + i * msg->angle_increment);
        marker.pose.position.z = 0.0;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        const float scaled_p = std::clamp(p, 0.0, 1.0);  // scale up small values due to beam likelihood is low

        marker.color.r = scaled_p;        // likelihood mapped to red
        marker.color.g = 0.0;
        marker.color.b = 1.0 - scaled_p;  // inverse for blue
        marker.color.a = 1.0;

        // Marker Lifetime
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(marker);

      }
      if (best_p > 0.001)  // threshold to accept detection
      {
        detected_range_ = best_r;
        detected_angle_ = best_angle;
        have_measurement_ = true;
        last_detection_time_ = this->now();
      }
      marker_pub_->publish(marker_array);

    }

 void control_loop()
  {
    geometry_msgs::msg::Twist cmd;
    rclcpp::Time now = this->now();
    // copy measurements under lock
    double r = NAN;
    double angle = NAN;
    bool valid_detection = false;
    {
      std::lock_guard<std::mutex> guard(scan_mutex_);
      if (have_measurement_ && std::isfinite(detected_range_) && std::isfinite(detected_angle_)) {
        r = detected_range_;
        angle = detected_angle_;
        valid_detection = true;
      }
    }

    if (!valid_detection) {
      // No detection -> stop
      RCLCPP_DEBUG(this->get_logger(), "No measurement available (never). Spinning to search...");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    // Check detection timeout, lost target -> stop
    double dt_lost = (now - last_detection_time_).seconds();
    if (dt_lost > lost_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Lost target for %.2f s -> searching", dt_lost);
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    // Normal control law
    // distance error: positive -> too far -> forward
    double dist_err = r - desired_distance_;


    auto normalize_angle = [](double a) {
      while (a > M_PI) a -= 2.0 * M_PI;
      while (a < -M_PI) a += 2.0 * M_PI;
      return a;
    };

    angle = normalize_angle(angle);


    // further moves faster
    double v = kp_dist_ * dist_err;
    double w = kp_angle_ * angle;
    if (std::abs(angle) > 0.6) {
      // turn-in-place if leader is behind or large angle
      v = 0.0;
    }
    // Stop if too close (leader could be moving towards follower )
    if (r <= stop_distance_) {
        constexpr double backup_speed = 0.2;
        v = -backup_speed;
        w = 0.0;
        RCLCPP_DEBUG(this->get_logger(),
            "Leader too close (r=%.3f). Backing up.", r);
    }

    // Oscillation check
    if (std::abs(detected_angle_) < 0.05) w = 0.0;

    // Limit velocities
    v = std::clamp(v, -max_linear_, max_linear_);
    w = std::clamp(w, -max_angular_, max_angular_);

    cmd.linear.x = v;
    cmd.angular.z = w;

    cmd_pub_->publish(cmd);

  }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
