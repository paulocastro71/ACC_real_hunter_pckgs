#pragma once
#include <time.h>
#include <ctime>
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
#include <std_srvs/srv/set_bool.hpp>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <SDL2/SDL.h>


#include <math.h>
#include <cmath>


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#define n_setores 21
#define MAX_DIST 3000
#define WHEEL_BASE 0.650
#define WHEEL_TRACK 0.465 //60.5?
#define MAX_LINEAR_SPEED 1.5
#define MAX_ANGULAR_SPEED 0.9
#define MAX_ACCEL 5
#define MAX_JOYSTICK_VAL 32767

enum direction {FORWARD,BACKWARD,turn_left,turn_right,robot_stop,robot_standby};

class joystick_control_class : public rclcpp::Node {
    public:
        joystick_control_class();
        ~joystick_control_class();
        void Throttle_Control();
        void Steering_Control();
        void Stop_Control();
        void PublishData();
        void SendEnableAccRequest();
        void SendDisableAccRequest();
        int SendRequest(bool command);
        void Throttle_Callback(const std_msgs::msg::Float32::ConstPtr& throttle_received);
        void Steering_Callback(const std_msgs::msg::Float32::ConstPtr& steering_received);
        void Stop_Callback(const std_msgs::msg::Bool::ConstPtr& stop_received);
        void ACC_Callback(const geometry_msgs::msg::Twist::ConstPtr& vel_received);
        void Enable_ACC_Callback(const std_msgs::msg::Bool::ConstPtr& enable_received);
        void Deactivate_ACC_Callback(const std_msgs::msg::Bool::ConstPtr& msg_received);
        //FLAGS
        bool start_moving;
        bool enable_acc;
        bool received_acc_vel;
        bool received_enable_command;
        bool send_request_flag;
        
        //PARAM
        float timestep;
        float sim_timestep;
        float curr_sim_time;
        float steering_angle;
        float linear_velocity;
        float angular_velocity;
        float last_linear_velocity;
        bool stop;
        float value_throttle;
        float value_steering;
        bool value_stop;
        direction robot_state;
        float vel_from_acc;
        

    private:

    //SUBSCRIBERS
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ThrottleSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SteeringSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr StopSub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ACCSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr EnableACCSub;
        
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr DeactivateACCSub;

    //PUBLISHERS
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr VelPublisher;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr EnablePublisher;
        
        rclcpp::Node::SharedPtr Client_Node;

        geometry_msgs::msg::Twist vel;
        std_msgs::msg::String state;
        std_msgs::msg::Bool enable_msg;
    //ROS2
        std_msgs::msg::Float32 throttle;
        std_msgs::msg::Float32 steering;
        std_msgs::msg::Bool start;

    //Service
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _client;
    
};
