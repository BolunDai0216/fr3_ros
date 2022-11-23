#pragma once
#include <array>
#include <string>
#include <vector>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace fr3_ros {

  class MarkerListVisualizer{
  
  public:
    MarkerListVisualizer(ros::NodeHandle &nh, int markers_count, int max_pub_rate);
    void publish(std::vector<Eigen::Matrix<double, 7, 1>> &marker_poses);

  private:
    // pinocchio model & data
    int markers_count;
    ros::Publisher publisher;
    ros::Time last_transmission;
  };
}  // namespace fr3_ros