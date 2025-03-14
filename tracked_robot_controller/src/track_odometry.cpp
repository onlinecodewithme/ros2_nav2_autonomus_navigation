#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TrackOdometry : public rclcpp::Node
{
public:
  TrackOdometry() : Node("track_odometry")
  {
    // Subscribe to track encoder feedback
    // Replace with your actual encoder topics
    left_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "left_track_encoder", 10, std::bind(&TrackOdometry::leftEncoderCallback, this, std::placeholders::_1));
      
    right_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "right_track_encoder", 10, std::bind(&TrackOdometry::rightEncoderCallback, this, std::placeholders::_1));
      
    // Create odometry publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Parameters
    this->declare_parameter<double>("wheel_separation", 0.3);  // Distance between tracks in meters
    this->declare_parameter<double>("encoder_resolution", 360.0); // Ticks per revolution
    this->declare_parameter<double>("wheel_radius", 0.05);     // Track wheel radius in meters
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    
    // Get parameters
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    encoder_resolution_ = this->get_parameter("encoder_resolution").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    
    // Initialize odometry
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    
    last_left_encoder_ = 0;
    last_right_encoder_ = 0;
    first_encoder_reading_ = true;
    
    last_time_ = this->now();
    
    // Create timer for odometry publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&TrackOdometry::publishOdometry, this));
      
    RCLCPP_INFO(this->get_logger(), "Track odometry node initialized");
  }

private:
  void leftEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (first_encoder_reading_) {
      last_left_encoder_ = msg->data;
      first_encoder_reading_ = false;
      return;
    }
    
    left_encoder_ = msg->data;
  }
  
  void rightEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (first_encoder_reading_) {
      last_right_encoder_ = msg->data;
      first_encoder_reading_ = false;
      return;
    }
    
    right_encoder_ = msg->data;
  }
  
  void publishOdometry()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    
    if (dt == 0.0) {
      return;
    }
    
    // Calculate wheel distance traveled
    double delta_left = (left_encoder_ - last_left_encoder_) * 2.0 * M_PI * wheel_radius_ / encoder_resolution_;
    double delta_right = (right_encoder_ - last_right_encoder_) * 2.0 * M_PI * wheel_radius_ / encoder_resolution_;
    
    // Update last encoder values
    last_left_encoder_ = left_encoder_;
    last_right_encoder_ = right_encoder_;
    
    // Calculate forward distance and rotation
    double delta_distance = (delta_right + delta_left) / 2.0;
    double delta_theta = (delta_right - delta_left) / wheel_separation_;
    
    // Update position and orientation
    x_ += delta_distance * cos(th_ + delta_theta / 2.0);
    y_ += delta_distance * sin(th_ + delta_theta / 2.0);
    th_ += delta_theta;
    
    // Keep theta within -pi to pi
    while (th_ > M_PI) th_ -= 2.0 * M_PI;
    while (th_ < -M_PI) th_ += 2.0 * M_PI;
    
    // Calculate velocities
    double vx = delta_distance / dt;
    double vy = 0.0;
    double vth = delta_theta / dt;
    
    // Create quaternion from yaw
    double cy = cos(th_ * 0.5);
    double sy = sin(th_ * 0.5);
    double cr = 1.0;
    double sr = 0.0;
    double cp = 1.0;
    double sp = 0.0;
    
    double qw = cy * cr * cp + sy * sr * sp;
    double qx = cy * sr * cp - sy * cr * sp;
    double qy = cy * cr * sp + sy * sr * cp;
    double qz = sy * cr * cp - cy * sr * sp;
    
    // Publish transform
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = current_time;
    transform_stamped.header.frame_id = odom_frame_;
    transform_stamped.child_frame_id = base_frame_;
    
    transform_stamped.transform.translation.x = x_;
    transform_stamped.transform.translation.y = y_;
    transform_stamped.transform.translation.z = 0.0;
    
    transform_stamped.transform.rotation.w = qw;
    transform_stamped.transform.rotation.x = qx;
    transform_stamped.transform.rotation.y = qy;
    transform_stamped.transform.rotation.z = qz;
    
    tf_broadcaster_->sendTransform(transform_stamped);
    
    // Publish odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    
    // Set position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.y = qy;
    odom.pose.pose.orientation.z = qz;
    
    // Set velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    
    // Publish
    odom_pub_->publish(odom);
    
    last_time_ = current_time;
  }

  // Subscribers and publishers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Parameters
  double wheel_separation_;
  double encoder_resolution_;
  double wheel_radius_;
  std::string odom_frame_;
  std::string base_frame_;
  
  // State variables
  double x_, y_, th_;
  int32_t left_encoder_, right_encoder_;
  int32_t last_left_encoder_, last_right_encoder_;
  bool first_encoder_reading_;
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackOdometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}