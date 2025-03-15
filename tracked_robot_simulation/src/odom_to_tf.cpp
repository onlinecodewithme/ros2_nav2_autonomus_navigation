#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class OdomToTF : public rclcpp::Node
{
public:
  OdomToTF() : Node("odom_to_tf")
  {
    // Parameters
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("parent_frame", "odom");  // Changed from 'map' to 'odom'
    this->declare_parameter("child_frame", "base_link");
    
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    
    // Create TF publisher
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
    
    // Create static transform broadcaster
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Publish static transform from map to odom
    publishMapToOdom();
    
    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&OdomToTF::odomCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Odometry to TF node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing transform from %s to %s", 
                parent_frame_.c_str(), child_frame_.c_str());
    
    // Set up timer to republish map->odom transform periodically
    map_odom_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OdomToTF::publishMapToOdom, this));
  }

private:
  void publishMapToOdom()
  {
    geometry_msgs::msg::TransformStamped transform;
    
    // Set header
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    
    // Set child frame
    transform.child_frame_id = "odom";
    
    // Set identity transform
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    
    // Publish static transform
    static_broadcaster_->sendTransform(transform);
    RCLCPP_DEBUG(this->get_logger(), "Published static transform: map -> odom");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    
    // Set header
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = parent_frame_;
    
    // Set child frame
    transform.child_frame_id = child_frame_;
    
    // Set translation
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    
    // Set rotation
    transform.transform.rotation = msg->pose.pose.orientation;
    
    // Create TF message
    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms.push_back(transform);
    
    // Publish transform
    tf_publisher_->publish(tf_message);
  }

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // TF publisher
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  
  // Static TF broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  
  // Timer for republishing map->odom transform
  rclcpp::TimerBase::SharedPtr map_odom_timer_;
  
  // Parameters
  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToTF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
