#include <fr3_ros/visualization_utils.h>

namespace fr3_ros {

  MarkerListVisualizer::MarkerListVisualizer(ros::NodeHandle &nh, int markers_count, int max_pub_rate) {
    publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1);
    markers_count = markers_count;
  }

  void MarkerListVisualizer::publish(std::vector<Eigen::Matrix<double, 7, 1>> &marker_poses){

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "/markers_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "fr3_ros";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
   
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    //iterate over the poses and load the markers array with marker
    for(int i; i<marker_poses.size(); i++)
    {
      marker.pose.position.x = marker_poses[i](0);
      marker.pose.position.y = marker_poses[i](1);
      marker.pose.position.z = marker_poses[i](2);
      marker.pose.orientation.x = marker_poses[i](0);
      marker.pose.orientation.y = marker_poses[i](1);
      marker.pose.orientation.z = marker_poses[i](2);
      marker.pose.orientation.w = marker_poses[i](3);
      marker.id = i;
      markers.markers.push_back(marker);
    }
    publisher.publish(markers);
  }

}  // namespace fr3_ros