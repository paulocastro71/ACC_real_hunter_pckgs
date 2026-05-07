#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>  
using namespace std;  


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "custom_msgs/msg/pose_array.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/'username'/ros2_ws/install/custom_msgs/include/custom_msgs",
#include "custom_msgs/msg/positioning_array.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/'username'/ros2_ws/install/custom_msgs/include/custom_msgs",
#include "custom_msgs/msg/positioning.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/'username'/ros2_ws/install/custom_msgs/include/custom_msgs",
#include "custom_msgs/srv/enable_acc.hpp" // Adicionar a includePath em c_cpp_properties.json ---> "/home/'username'/ros2_ws/install/custom_msgs/include/custom_msgs",

#include <math.h>
#include <cmath>


//Definitions for GPS debug - no longer in use
#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES PI / 180
#define RADIANES_GRADOS 180 / PI

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


//HUNTER PARAMETERS
#define WHEEL_BASE 0.650
#define WHEEL_TRACK 0.605

//ACC PARAMETERS
#define MINIMUM_SAFE_DISTANCE 1.5 //independent of velocity
#define TARGET_HEADWAY_TIME 1.5   //time interval between host and leader
#define MINIMUM_ACC_VELOCITY 0.1  //threshold to deactivate ACC mode


bool enable_global;
bool request_received;

//ROS2 SERVICE
void Enable_Response(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);


class adaptive_cruise_control_hunter_class : public rclcpp::Node {
    public:

        adaptive_cruise_control_hunter_class();
        ~adaptive_cruise_control_hunter_class();

        //Control Task
        float AdaptiveCruiseControl(const float max_speed);

        //Publish Data
        void PublishData();
        
        //Switch Modes
        void setEnableACC(bool a);
        
        //Leader Pose relative to Host
        void Leader_Callback(const custom_msgs::msg::Positioning::ConstPtr& leader_info_received);
        
        //Host Speed
        void Velocity_Callback(const std_msgs::msg::Float32::ConstPtr& velocity_received);

        //Leader Speed
        void Pose_Callback(const geometry_msgs::msg::PoseStamped::ConstPtr& pose_received);
    


        //FLAGS
        bool received_position;
        bool enable_acc;
        bool leader_pose_received;;
        //PARAM
        float timestep;
        float sim_timestep;
        float curr_sim_time;
        float curr_vel;
        float target_vel;
        float leader_vel;
        float safety_gain;
        float lambda_v;

    private:
    //SUBSCRIBERS
        //Leader pose realtive to host
        rclcpp::Subscription<custom_msgs::msg::Positioning>::SharedPtr LeaderSub;

        //HOST MOTION STATE
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr PoseSub;
        
    //ROS2 Service
       rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;

    //PUBLISHERS
        //PUBLISHERS
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr VelPublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr RelDistPublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr SafeDistPublisher;

        //Deactivate ACC
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr DeactivateACCPublisher;

        //ROS2 MESSAGES
        geometry_msgs::msg::Twist vel;
        std_msgs::msg::String state;
        std_msgs::msg::Bool deactivate_acc_msg;
        std_msgs::msg::Float32 rdist_msg;
        std_msgs::msg::Float32 sdist_msg;
        rclcpp::Node::SharedPtr ServerNodeACC;
        


    //ROS2
        geometry_msgs::msg::Vector3 Position;
        geometry_msgs::msg::Vector3 RobotSize;
        geometry_msgs::msg::PoseStamped ACCPosition;
    
        
    //Positioning
        float phi_robot;
        float x_robot;
        float y_robot;
        float linear_velocity;

    //Adaptive Cruise Control
        float x_acc;
        float y_acc;
        float last_x_acc;
        float last_y_acc;
        bool following;
    
    //GPS - VARS NOT IN USE now
        double lat_hunter;
        double long_hunter;
        double lat_leader;
        double long_leader;
        float roll_hunter;
        float roll_leader;
        double lat_hunter_old;
        double long_hunter_old;
        
        bool first_cycle;
 
};

