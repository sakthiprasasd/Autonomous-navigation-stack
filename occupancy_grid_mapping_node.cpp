#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

class OccupancyGridMappingNode
{
public:
  OccupancyGridMappingNode() : nh_("~")
  {
    // Get ROS parameters
    nh_.param("resolution", resolution_, 0.05);
    nh_.param("width", width_, 100);
    nh_.param("height", height_, 100);
    nh_.param("threshold", threshold_, 0.75);

    // Subscribe to scan topic
    scan_sub_ = nh_.subscribe("/scan", 1, &OccupancyGridMappingNode::scanCallback, this);

    // Advertise occupancy grid map topic
    occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_map", 1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
  {
    // Initialize occupancy grid map
    nav_msgs::OccupancyGrid occupancy_grid_map;
    occupancy_grid_map.header = scan_msg->header;
    occupancy_grid_map.info.resolution = resolution_;
    occupancy_grid_map.info.width = width_;
    occupancy_grid_map.info.height = height_;
    occupancy_grid_map.info.origin.position.x = -width_/2.0 * resolution_;
    occupancy_grid_map.info.origin.position.y = -height_/2.0 * resolution_;
    occupancy_grid_map.info.origin.position.z = 0.0;
    occupancy_grid_map.info.origin.orientation.x = 0.0;
    occupancy_grid_map.info.origin.orientation.y = 0.0;
    occupancy_grid_map.info.origin.orientation.z = 0.0;
    occupancy_grid_map.info.origin.orientation.w = 1.0;
    occupancy_grid_map.data.resize(width_ * height_);

    // Populate occupancy grid map with scan data
    for (int i = 0; i < scan_msg->ranges.size(); i++) {
      double range = scan_msg->ranges[i];
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      if (std::isfinite(range)) {
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);
        int map_x = static_cast<int>(std::round((x - occupancy_grid_map.info.origin.position.x) / resolution_));
        int map_y = static_cast<int>(std::round((y - occupancy_grid_map.info.origin.position.y) / resolution_));
        if (map_x >= 0 && map_x < width_ && map_y >= 0 && map_y < height_) {
          int index = map_y * width_ + map_x;
          occupancy_grid_map.data[index] = 100;
        }
      }
    }

      // Apply threshold to occupancy grid map
    for (int i = 0; i < occupancy_grid_map.data.size(); i++) {
      if (occupancy_grid_map.data[i] >= threshold_ * 100) {
        occupancy_grid_map.data[i] = 100;
      } else if (occupancy_grid_map.data[i] <= (1 - threshold_) * 100) {
        occupancy_grid_map.data[i] = 0;
      } else {
        occupancy_grid_map.data[i] = -1;
      }
    }

    // Publish occupancy grid map
    occupancy_grid_map_pub_.publish(occupancy_grid_map);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher occupancy_grid_map_pub_;
  double resolution_;
  int width_;
  int height_;
  double threshold_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid_mapping_node");
  OccupancyGridMappingNode node;
  ros::spin();
  return 0;
}

