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
#include <sensor_msgs/msg/imu.hpp>
#include "custom_msgs/msg/pose_array.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/user/ros2_ws/install/custom_msgs/include/custom_msgs",
#include "custom_msgs/msg/positioning_array.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/user/ros2_ws/install/custom_msgs/include/custom_msgs",
#include "custom_msgs/msg/positioning.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/user/ros2_ws/install/custom_msgs/include/custom_msgs",
#include <nav_msgs/msg/odometry.hpp>

#include <UTM.hpp>
#include <tf_broadcaster.hpp>

#include <math.h>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class nearby_hunter_class : public rclcpp::Node {

    public:

        nearby_hunter_class();
        ~nearby_hunter_class();

        //Convert Lat Long to X, Y
        void ConvertGeo2Cart(const float Long, const float Lat);
    
        //Nearby Speed
        void Velocity_Leader_Callback(const std_msgs::msg::Float32::ConstPtr& velocity_leader_received);

        //Nearby Pose
        void Pose_Leader_Callback(const geometry_msgs::msg::PoseStamped::ConstPtr& pose_leader_received);

        //Update Nearby Msg
        void UpdateArray();

        //Publish Nearby Msg
        void PublishData();

        custom_msgs::msg::Positioning leading_vehicle;
        geometry_msgs::msg::PoseStamped geo_pose_zero;

        bool vehicle_zero_exists;
 

    private:

        //SUBSCRIPTION
    
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocityLeaderSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr PoseLeaderSub;

        //custom_msgs::msg::PoseArray poses;


        //PUBLISHER
        rclcpp::Publisher<custom_msgs::msg::PositioningArray>::SharedPtr NearbyHunterPub;
        custom_msgs::msg::PositioningArray NearbyHunterMsg;
        custom_msgs::msg::Positioning vehicle_zero;
        
        

        float oriX;
        float oriY;
        float oriZ;
        float oriW;
        float longitude;
        float latitude;
        float altitude;
        float speed;
        
        double lat_hunter_zero;
        double long_hunter_zero;
        float  yaw_hunter_zero;
    
        int utm_zone;
};