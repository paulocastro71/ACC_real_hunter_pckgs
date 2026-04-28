#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <ctime>  
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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>



#include <math.h>
#include <cmath>

#include <UTM.hpp>             //Library that handles GPS to XYZ conversion using Universal Tranversal Mercator Projection
#include <tf_broadcaster.hpp>  //Class that handles the publishing of the tf's with the positioning and orientation info in relation to "geo" frame
#include <kalman.hpp>

#undef pi
#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#define WHEEL_BASE 0.650
#define WHEEL_TRACK 0.465 //60.5?

enum STATE_ {
        STANDBY,
        CALIBRATION,
        INIT_FILTER,
        FILTER
};

bool enable_global;
bool request_received;
void Enable_Response(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);


class positioning_hunter_class : public rclcpp::Node {
    public:
        positioning_hunter_class();
        ~positioning_hunter_class();
        void PublishData();
        void Convert_GNSS_Geo();
        void ConvertFiltered(bool flag, double utm_x, double utm_y);
        void Save_First_Position();
        void Save_Last_Position();
        void Calibrate_Orientation();
        void setEnableACC(bool a);
        void GNSS_Callback(const sensor_msgs::msg::NavSatFix::ConstPtr& gnss_received);
        void Imu_Callback(const sensor_msgs::msg::Imu::ConstPtr& imu_received);
        void Speedometer_Callback(const std_msgs::msg::Float32::ConstPtr& speed_received);
        void Odometry_Callback(const nav_msgs::msg::Odometry::ConstPtr& odom_received);
        void ZedOrientation_Callback(const sensor_msgs::msg::Imu::ConstPtr& ori_received);
        void Latency_Callback(const std_msgs::msg::Bool::ConstPtr& ping_received);
        void SendPing();

       // void Enable_Response(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

       // void Enable_Response(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

       
        //PARAM
        float timestep;
        float sim_timestep;
        float curr_sim_time;
        float curr_vel;
        float orientation;
        float speed;

        float phi_robot;
        double x_geo;
        double y_geo;
        double x_geo_filtered;
        double y_geo_filtered;
        double lat_filtered;
        double lon_filtered;

        //Flags
        bool received_gnss_data;

        //XY GeoLocation
        geometry_msgs::msg::PoseStamped geo_pose;

        double first_x_geo; double first_y_geo;
        double last_x_geo; double last_y_geo;
        bool calibrated;
        bool first_position_saved;
        bool last_position_saved;
    private:
    //SUBSCRIBERS


        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr GnssSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr EnableSub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ImuSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SpeedometerSub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdomSub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ZedSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr LatencySub;

        
    //Service
       //rclcpp::Service<std_srvs::srv::SetBool>::Service::SharedPtr EnableAccService;
       rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;

    //PUBLISHERS
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr PosePublisher;
        geometry_msgs::msg::PoseStamped pose_msg;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr Pose2Publisher;
        geometry_msgs::msg::PoseStamped pose2_msg;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr SpeedPublisher;
        std_msgs::msg::Float32 speed_msg;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr LatencyPublisher;
        std_msgs::msg::Bool latency_msg;



    //ROS2
        geometry_msgs::msg::Vector3 Position;
        geometry_msgs::msg::Vector3 RobotSize;
        geometry_msgs::msg::PoseStamped ACCPosition;
        geometry_msgs::msg::Vector3 Orientation;
        geometry_msgs::msg::Vector3 TargetPosition;
        std_msgs::msg::Float32 SimTime;
        
    //Positioning
        
        float x_robot;
        float y_robot;
        float linear_velocity;
        float speedometer;
        //GNSS
        float latitude;
        float longitude;
        float altitude;
        //IMU
        float o_x;
        float o_y;
        float o_z;
        float o_w;
        float AccX;
        float AccY;
        float AccZ;
        float GyroX;
        float GyroY;
        float GyroZ;
        std_msgs::msg::Header pose_header_stamp;

        //GNSS
        
        double z_geo;
        int utm_zone;

        
        

        double angle_offset;
        tf2::Quaternion quat;

        chrono::system_clock::time_point t0;
        chrono::system_clock::time_point t1;
        
};
