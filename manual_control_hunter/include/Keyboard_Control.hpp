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


#include <math.h>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#define n_setores 21
#define MAX_DIST 3000
#define WHEEL_BASE 0.650
#define WHEEL_TRACK 0.465 //60.5?
#define MAX_LINEAR_SPEED 0.7 //1.5
#define MAX_ANGULAR_SPEED 0.3

enum direction {go_forward,go_backwards,turn_left,turn_right,robot_stop,robot_standby};

class keyboard_control_class : public rclcpp::Node {
    public:
        keyboard_control_class();
        ~keyboard_control_class();
        void Start_Control();
        void Throttle_Control();
        void Steering_Control();
        void ConvergeToZero();
        void MovementControl();
        void PublishData();
        void Start_Callback(const std_msgs::msg::Bool::ConstPtr& start_received);
        void Throttle_Callback(const std_msgs::msg::Float32::ConstPtr& throttle_received);
        void Steering_Callback(const std_msgs::msg::Float32::ConstPtr& steering_received);
        void KeyboardControl_Callback(const std_msgs::msg::Char::ConstPtr& key_received);

        //FLAGS
        bool start_moving;
       
        //PARAM
        float timestep;
        float sim_timestep;
        float curr_sim_time;
        float steering_angle;
        float linear_velocity;
        direction robot_state;
    private:
    //SUBSCRIBERS
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ThrottleSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SteeringSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr StartSub;
        rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr KeyboardSub;

    //PUBLISHERS
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr VelPublisher;
        geometry_msgs::msg::Twist vel;
        std_msgs::msg::String state;

    //ROS2
        std_msgs::msg::Float32 throttle;
        std_msgs::msg::Float32 steering;
        std_msgs::msg::Bool start;

    //Robot
        
        char key;
   
};
