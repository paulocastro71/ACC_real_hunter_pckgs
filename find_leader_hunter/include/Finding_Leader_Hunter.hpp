#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>  
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
using namespace std;  
using namespace std::chrono_literals;

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/char.hpp>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include "custom_msgs/msg/pose_array.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/paulo/ros2_ws/install/custom_msgs/include/custom_msgs",
#include "custom_msgs/msg/positioning_array.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/paulo/ros2_ws/install/custom_msgs/include/custom_msgs",
#include "custom_msgs/msg/positioning.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/paulo/ros2_ws/install/custom_msgs/include/custom_msgs",




#include <math.h>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class finding_leader_hunter_class : public rclcpp::Node {
    public:
    finding_leader_hunter_class();
    ~finding_leader_hunter_class();
    //calback to get nearby vehicles motion state
    void Nearby_Hunter_Callback(const custom_msgs::msg::PositioningArray msg);

    //GNSS CALLBACK
    void GNSS_Callback(const sensor_msgs::msg::NavSatFix::ConstPtr& gnss_received);

    //ROS2 Transform (geo frame to host frame)
    void ConvertFrame();

    //PUblish Leader data
    void PublishData();

    //Find Leader Algorithm
    bool Find_Leading_Vehicle();

    //Custom msg that holds leader pose and speed
    custom_msgs::msg::Positioning leading_vehicle;

    //flags for state change
    bool nearby_cars_flag;
    bool converted;


    private:

    //SUBSCRIPTION
    rclcpp::Subscription<custom_msgs::msg::PositioningArray>::SharedPtr PoseSub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr GnssSub;

    //PUBLISHERS

        //publish leader motion state
        rclcpp::Publisher<custom_msgs::msg::Positioning>::SharedPtr LeaderPub;
        custom_msgs::msg::Positioning leaderMSG;

    //TF2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener; 

    //Positioning Arrays with nearby vehicles
    custom_msgs::msg::PositioningArray nearby_cars_world; //pose in the world frame
    custom_msgs::msg::PositioningArray nearby_cars_vehicle; //pose in the vehicle frame

    //Aux msgs for tf2
    geometry_msgs::msg::PoseStamped transform_pose;
    geometry_msgs::msg::PoseStamped transformed_pose;

    //GPS debug - not in use now
    double lat_hunter; double long_hunter; double lat_leader; double long_leader;
    
 

};