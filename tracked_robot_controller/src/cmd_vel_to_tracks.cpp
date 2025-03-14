#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

// You may need to replace these with your actual motor control messages
#include "std_msgs/msg/int32.hpp"

class CmdVelToTracks : public rclcpp::Node
{
public:
  CmdVelToTracks() : Node("cmd_vel_to_tracks")
  {
    // Subscribe to cmd_vel topic
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&CmdVelToTracks::velocityCallback, this, std::placeholders::_1));
      
    // Publishers for the left and right track commands
    left_track_pub_ = this->create_publisher<std_msgs::msg::Int32>("left_track_cmd", 10);
    right_track_pub_ = this->create_publisher<std_msgs::msg::Int32>("right_track_cmd", 10);
    
    // Parameters for track control
    this->declare_parameter<double>("max_linear_speed", 0.5);  // m/s
    this->declare_parameter<double>("max_angular_speed", 1.0); // rad/s
    this->declare_parameter<int>("max_pwm", 255);              // Max PWM value
    this->declare_parameter<double>("wheel_separation", 0.3);  // Distance between tracks in meters
    
    // Get parameters
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    max_pwm_ = this->get_parameter("max_pwm").as_int();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Track controller node initialized");
  }

private:
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Extract linear and angular velocities
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;
    
    // Clamp values to max speeds
    linear_x = std::max(-max_linear_speed_, std::min(linear_x, max_linear_speed_));
    angular_z = std::max(-max_angular_speed_, std::min(angular_z, max_angular_speed_));
    
    // Calculate left and right track velocities using differential drive kinematics
    double left_vel = linear_x - angular_z * wheel_separation_ / 2.0;
    double right_vel = linear_x + angular_z * wheel_separation_ / 2.0;
    
    // Convert to PWM values (-max_pwm to max_pwm)
    int left_pwm = static_cast<int>((left_vel / max_linear_speed_) * max_pwm_);
    int right_pwm = static_cast<int>((right_vel / max_linear_speed_) * max_pwm_);
    
    // Create and publish messages
    auto left_msg = std::make_unique<std_msgs::msg::Int32>();
    auto right_msg = std::make_unique<std_msgs::msg::Int32>();
    
    left_msg->data = left_pwm;
    right_msg->data = right_pwm;
    
    left_track_pub_->publish(std::move(left_msg));
    right_track_pub_->publish(std::move(right_msg));
    
    RCLCPP_DEBUG(this->get_logger(), "Publishing track commands: left=%d, right=%d", left_pwm, right_pwm);
  }

  // Subscribers and publishers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_track_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_track_pub_;
  
  // Parameters
  double max_linear_speed_;
  double max_angular_speed_;
  int max_pwm_;
  double wheel_separation_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelToTracks>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}