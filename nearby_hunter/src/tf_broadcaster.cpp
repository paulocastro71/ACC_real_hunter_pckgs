#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf_broadcaster.hpp>


TfBroadcaster::TfBroadcaster():
  Node("tf_broadcaster"){
    // Initialize the transforms broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Define the "parent" frame
    transf_.header.frame_id = "geo";
    // Define the "child" frame
    transf_.child_frame_id = "self";
  }
  
  void TfBroadcaster::UpdateTransform(const geometry_msgs::msg::PoseStamped geo_pose){
    transf_.transform.translation.x = geo_pose.pose.position.x;
    transf_.transform.translation.y = geo_pose.pose.position.y;
    transf_.transform.translation.z = geo_pose.pose.position.z;
    
    transf_.transform.rotation.x = geo_pose.pose.orientation.x;
    transf_.transform.rotation.y = geo_pose.pose.orientation.y;
    transf_.transform.rotation.z = geo_pose.pose.orientation.z;
    transf_.transform.rotation.w = geo_pose.pose.orientation.w;
    //transf_.header.stamp = this->get_clock()->now();
    transf_.header.stamp = geo_pose.header.stamp;
    transf_.header.frame_id = "geo";
    transf_.child_frame_id = geo_pose.header.frame_id;
    tf_broadcaster_->sendTransform(transf_);
 }

  TfBroadcaster::~TfBroadcaster(){}