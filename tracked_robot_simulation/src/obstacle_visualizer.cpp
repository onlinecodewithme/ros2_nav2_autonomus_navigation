#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

class ObstacleVisualizer : public rclcpp::Node
{
public:
  ObstacleVisualizer() : Node("obstacle_visualizer")
  {
    // Parameters
    this->declare_parameter("costmap_topic", "/global_costmap/costmap");
    this->declare_parameter("marker_topic", "/obstacle_markers");
    this->declare_parameter("obstacle_threshold", 70);
    this->declare_parameter("marker_lifetime", 0.5);
    this->declare_parameter("marker_scale", 0.1);
    this->declare_parameter("marker_color_r", 1.0);
    this->declare_parameter("marker_color_g", 0.0);
    this->declare_parameter("marker_color_b", 0.0);
    this->declare_parameter("marker_color_a", 1.0);
    
    costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
    marker_lifetime_ = this->get_parameter("marker_lifetime").as_double();
    marker_scale_ = this->get_parameter("marker_scale").as_double();
    marker_color_r_ = this->get_parameter("marker_color_r").as_double();
    marker_color_g_ = this->get_parameter("marker_color_g").as_double();
    marker_color_b_ = this->get_parameter("marker_color_b").as_double();
    marker_color_a_ = this->get_parameter("marker_color_a").as_double();
    
    // Create subscribers and publishers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, 10, 
      std::bind(&ObstacleVisualizer::costmapCallback, this, std::placeholders::_1));
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, 10);
    
    RCLCPP_INFO(this->get_logger(), "Obstacle Visualizer initialized");
  }

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete all previous markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // Create a new marker for obstacles
    visualization_msgs::msg::Marker obstacle_marker;
    obstacle_marker.header = msg->header;
    obstacle_marker.ns = "obstacles";
    obstacle_marker.id = 0;
    obstacle_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set the scale of the marker
    obstacle_marker.scale.x = marker_scale_;
    obstacle_marker.scale.y = marker_scale_;
    obstacle_marker.scale.z = marker_scale_;
    
    // Set the color
    obstacle_marker.color.r = marker_color_r_;
    obstacle_marker.color.g = marker_color_g_;
    obstacle_marker.color.b = marker_color_b_;
    obstacle_marker.color.a = marker_color_a_;
    
    // Set the lifetime
    obstacle_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    
    // Set the frame
    obstacle_marker.frame_locked = true;
    
    // Iterate through the costmap and add points for obstacles
    for (unsigned int i = 0; i < msg->data.size(); ++i)
    {
      if (msg->data[i] > obstacle_threshold_)
      {
        unsigned int mx = i % msg->info.width;
        unsigned int my = i / msg->info.width;
        double wx = msg->info.origin.position.x + (mx + 0.5) * msg->info.resolution;
        double wy = msg->info.origin.position.y + (my + 0.5) * msg->info.resolution;
        
        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        point.z = 0.1;  // Slightly above ground
        
        obstacle_marker.points.push_back(point);
      }
    }
    
    // Only add the marker if there are points
    if (!obstacle_marker.points.empty())
    {
      marker_array.markers.push_back(obstacle_marker);
    }
    
    // Publish the marker array
    marker_pub_->publish(marker_array);
  }

  // Subscribers and publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // Parameters
  std::string costmap_topic_;
  std::string marker_topic_;
  int obstacle_threshold_;
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
  auto node = std::make_shared<ObstacleVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
