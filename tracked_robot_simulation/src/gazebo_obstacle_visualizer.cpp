#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

class GazeboObstacleVisualizer : public rclcpp::Node
{
public:
  GazeboObstacleVisualizer() : Node("gazebo_obstacle_visualizer")
  {
    // Parameters
    this->declare_parameter("marker_topic", "/gazebo_obstacles");
    this->declare_parameter("update_rate", 1.0);  // Hz
    this->declare_parameter("marker_lifetime", 2.0);
    this->declare_parameter("marker_scale", 1.0);
    this->declare_parameter("marker_color_r", 1.0);
    this->declare_parameter("marker_color_g", 0.0);
    this->declare_parameter("marker_color_b", 0.0);
    this->declare_parameter("marker_color_a", 0.8);
    
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    update_rate_ = this->get_parameter("update_rate").as_double();
    marker_lifetime_ = this->get_parameter("marker_lifetime").as_double();
    marker_scale_ = this->get_parameter("marker_scale").as_double();
    marker_color_r_ = this->get_parameter("marker_color_r").as_double();
    marker_color_g_ = this->get_parameter("marker_color_g").as_double();
    marker_color_b_ = this->get_parameter("marker_color_b").as_double();
    marker_color_a_ = this->get_parameter("marker_color_a").as_double();
    
    // Create publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, 10);
    
    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_),
      std::bind(&GazeboObstacleVisualizer::publishObstacles, this));
    
    RCLCPP_INFO(this->get_logger(), "Gazebo Obstacle Visualizer initialized");
  }

private:
  void publishObstacles()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete all previous markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // Add markers for the walls and obstacles in the Gazebo world
    addWallMarker(marker_array, "wall1", 2.0, 2.0, 0.5, 4.0, 0.2, 1.0, 0.0, 0.0, 0.0);
    addWallMarker(marker_array, "wall2", -2.0, -2.0, 0.5, 3.0, 0.2, 1.0, 0.0, 0.0, 0.7);
    addWallMarker(marker_array, "wall3", 0.0, 3.0, 0.5, 4.0, 0.2, 1.0, 0.0, 0.0, 1.57);
    addWallMarker(marker_array, "wall4", -3.0, 1.0, 0.5, 3.0, 0.2, 1.0, 0.0, 0.0, 1.57);
    
    // Add box obstacle
    addBoxMarker(marker_array, "box1", 1.0, -1.0, 0.25, 0.5, 0.5, 0.5, 0.8, 0.1, 0.1);
    
    // Add cylinder obstacle
    addCylinderMarker(marker_array, "cylinder1", -1.0, 2.0, 0.3, 0.3, 0.6, 0.1, 0.1, 0.8);
    
    // Add sphere obstacle
    addSphereMarker(marker_array, "sphere1", 3.0, -2.0, 0.3, 0.3, 0.1, 0.8, 0.1);
    
    // Publish the marker array
    marker_pub_->publish(marker_array);
  }
  
  void addWallMarker(
    visualization_msgs::msg::MarkerArray& marker_array,
    const std::string& name,
    double x, double y, double z,
    double length, double width, double height,
    double r, double g, double b,
    double yaw = 0.0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "walls";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    
    // Convert yaw to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    marker.pose.orientation.w = cy;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sy;
    
    marker.scale.x = length;
    marker.scale.y = width;
    marker.scale.z = height;
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = marker_color_a_;
    
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    
    marker_array.markers.push_back(marker);
  }
  
  void addBoxMarker(
    visualization_msgs::msg::MarkerArray& marker_array,
    const std::string& name,
    double x, double y, double z,
    double size_x, double size_y, double size_z,
    double r, double g, double b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "boxes";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = size_x;
    marker.scale.y = size_y;
    marker.scale.z = size_z;
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = marker_color_a_;
    
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    
    marker_array.markers.push_back(marker);
  }
  
  void addCylinderMarker(
    visualization_msgs::msg::MarkerArray& marker_array,
    const std::string& name,
    double x, double y, double z,
    double radius, double height,
    double r, double g, double b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "cylinders";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = height;
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = marker_color_a_;
    
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    
    marker_array.markers.push_back(marker);
  }
  
  void addSphereMarker(
    visualization_msgs::msg::MarkerArray& marker_array,
    const std::string& name,
    double x, double y, double z,
    double radius,
    double r, double g, double b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "spheres";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = marker_color_a_;
    
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    
    marker_array.markers.push_back(marker);
  }

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  std::string marker_topic_;
  double update_rate_;
  double marker_lifetime_;
  double marker_scale_;
  double marker_color_r_;
  double marker_color_g_;
  double marker_color_b_;
  double marker_color_a_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboObstacleVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
