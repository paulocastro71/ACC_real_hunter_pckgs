#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class TfBroadcaster : public rclcpp::Node{
private:
    // Declare a timer for the tf_broadcaster_
    rclcpp::TimerBase::SharedPtr transform_timer_;
    // Declare the transforms broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Subscription for cmdVel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    // Declare the transform to broadcast
    geometry_msgs::msg::TransformStamped transf_;

public:
    TfBroadcaster();
    ~TfBroadcaster();
    void UpdateTransform(const geometry_msgs::msg::PoseStamped geo_pose);
};
