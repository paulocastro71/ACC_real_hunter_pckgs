#include <Adaptive_Cruise_Control_Hunter.hpp>


//ROS2 Service response
void Enable_Response(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    request_received=true;
    response->success=true;
    response->message="Command received sucessfully";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %d",
    request->data);
    enable_global=request->data;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s -> RESPONSE: [%d]",response->message,response->success);
}


adaptive_cruise_control_hunter_class::adaptive_cruise_control_hunter_class()
: Node("AdaptiveCruiseControlHunter")
{
 //SUBSCRIBERS 
    //Leader Pose and Speed
    LeaderSub = this->create_subscription<custom_msgs::msg::Positioning>("leader_pose", 1, std::bind(&adaptive_cruise_control_hunter_class::Leader_Callback, this, _1));
    
    //Vehicle Speed
    VelocitySub = this->create_subscription<std_msgs::msg::Float32>("vehicle_speed", 1, std::bind(&adaptive_cruise_control_hunter_class::Velocity_Callback, this, _1));
    
    //Vehicle Pose
    PoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("vehicle_pose", 1, std::bind(&adaptive_cruise_control_hunter_class::Pose_Callback, this, _1));

   
    //PUBLISHERS

    //ACC SPEED FOR Hunter
    VelPublisher = this->create_publisher<geometry_msgs::msg::Twist>("acc_vel",1);
    vel = geometry_msgs::msg::Twist();

    //Deactivate ACC Mode
    DeactivateACCPublisher = this->create_publisher<std_msgs::msg::Bool>("deactivate_acc_topic",1);
    deactivate_acc_msg = std_msgs::msg::Bool();

    //ROS2 SERVICE
    service = this->create_service<std_srvs::srv::SetBool>("enable_acc_service", &Enable_Response);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACC SERVER READY");

//INITS
    enable_acc=false;
    x_robot = 0.0;
    y_robot = 0.0;
    phi_robot = 0.0;
    linear_velocity=0.0;
    received_position=false;
    x_acc=0;
    y_acc=0;
    last_x_acc=0;
    last_y_acc=0;
    following = false;
    curr_sim_time=0;
    leader_pose_received=false;
    target_vel=0;
    safety_gain=1;
    lambda_v = 8;
    timestep=0.05;

}

adaptive_cruise_control_hunter_class::~adaptive_cruise_control_hunter_class()
{

}

void adaptive_cruise_control_hunter_class::Pose_Callback(const geometry_msgs::msg::PoseStamped::ConstPtr& pose_received)
{
    //Latitude Longitude
    //Pose is received in LAT LONG but it is not used
    //Find Leader module already sends the position of leader realtive to host
    //So, no host position data is required for ACC control task
    double longitude=pose_received->pose.position.x;
    double latitude=pose_received->pose.position.y;
    float altitude=pose_received->pose.position.z;

    //Orientation
    float OriX=pose_received->pose.orientation.x;
    float OriY=pose_received->pose.orientation.y;
    float OriZ=pose_received->pose.orientation.z;
    float OriW=pose_received->pose.orientation.w;

    lat_hunter=latitude;
    long_hunter=longitude;

    //Quaternion -> EULER Orientation
    phi_robot = atan2(2*(OriX*OriY + OriZ*OriW),1-2*(OriY*OriY + OriZ*OriZ));

    //Update Flag
    received_position=true;
}

void adaptive_cruise_control_hunter_class::Velocity_Callback(const std_msgs::msg::Float32::ConstPtr& velocity_received)
{
    
    curr_vel=velocity_received->data;

}




void adaptive_cruise_control_hunter_class::Leader_Callback(const custom_msgs::msg::Positioning::ConstPtr& leader_info_received)
{

    x_acc=(leader_info_received->pose.pose.position.x);
    y_acc=(leader_info_received->pose.pose.position.y);
    leader_vel=(leader_info_received->velx.data);

    //UPDATE FLAG - crucial for ACC to enter the correct mode
    leader_pose_received=true;

}


//ACC Control Task
float adaptive_cruise_control_hunter_class::AdaptiveCruiseControl(const float max_speed)
{
    if(curr_vel<MINIMUM_ACC_VELOCITY){
        enable_acc=false;
        deactivate_acc_msg.data=false; //sending false deactivates acc
        DeactivateACCPublisher->publish(deactivate_acc_msg);
        RCLCPP_INFO(this->get_logger(),"ACC DEACTIVATED DUE TO LOW SPEED");
        return curr_vel;
    }

    if (leader_pose_received){ //IF LEADER EXISTS

        leader_pose_received=false;  //clear flag

        //Check distance to leader
        //Leader pose is given relative to host so distance is a simple sqrt(y²+x²)
        float relative_distance = sqrt(pow((y_acc),2) + pow((x_acc),2));

        //Check Safe Distance
        float safe_distance = MINIMUM_SAFE_DISTANCE + (curr_vel * TARGET_HEADWAY_TIME);


        //Condition to switch between Cruise and Following Mode
        if ( relative_distance < (safe_distance * safety_gain))
        {
            //   //    //    //  //  //
            //     FOLLOWING MODE    //
            //  //    //    //  //   //
            //   Leader is in range  //
            //  //  //  //  //  //   //

            safety_gain=1.7; //Acts as ON-OFF controller

            following = true;

            //Display mode in terminal
            RCLCPP_INFO(this->get_logger(),"------FOLLOWING MODE!------");

            float front_car_velocity =  leader_vel;

            //Distance Error between actual distance and safe distance
            float dist_error = relative_distance - safe_distance;

            //Compute Desired Speed
            float vel_des = front_car_velocity * (relative_distance/safe_distance);

            if (vel_des>max_speed) vel_des=max_speed; //grantir que a velocidade está dentro do limite possível

            //Dynamic Linear System
            float g_v = - lambda_v * (curr_vel - vel_des);  //DLS
            float acc_vel = curr_vel + (timestep * g_v);    //Euler Method
            
            //Show info in terminal
            RCLCPP_INFO(this->get_logger(),"Desired Speed: %f",vel_des);
            RCLCPP_INFO(this->get_logger(),"Target Speed: %f",acc_vel);
            RCLCPP_INFO(this->get_logger(),"Current Speed: %f",curr_vel);
            RCLCPP_INFO(this->get_logger(),"Safe Distance: %f",safe_distance);
            RCLCPP_INFO(this->get_logger(),"Relative Distance: %f",relative_distance);
            RCLCPP_INFO(this->get_logger(),"Cruise Set Speed: %f",max_speed);
            RCLCPP_INFO(this->get_logger(),"---------------------------------------------");

            //logging to txt file
            //cout << relative_distance << ", " << safe_distance << ", " << curr_vel << ", " << acc_vel << ", " << vel_des << ", " << leader_vel << ", Following" << endl;
            return acc_vel;
    
        }else{

            //   //    //    //  //  //  //  //  //  //
            //     CRUISE MODE WITH LEADER DETECTED  //
            //  //    //    //  //   //  //  //  //  //
            //   Speed Control with leader far away  //
            //  //  //  //  //  //   //  //  //  //  //

            float g_v = - lambda_v * (curr_vel - max_speed);  //sistema dinâmico linear
            float cruise_vel = curr_vel + (timestep * g_v);      //método de euler

            //Show info in terminal
            RCLCPP_INFO(this->get_logger(),"----------Cruise Mode with Leader in Range-------");
            RCLCPP_INFO(this->get_logger(),"Current Speed: %f",curr_vel);
            RCLCPP_INFO(this->get_logger(),"Cruise Set Speed: %f",max_speed);
            RCLCPP_INFO(this->get_logger(),"Safe Distance: %f",safe_distance);
            RCLCPP_INFO(this->get_logger(),"Relative Distance: %f",relative_distance);
            RCLCPP_INFO(this->get_logger(),"--------------------------------------------------");

            //logging to txt file
            //cout << relative_distance << ", " << safe_distance << ", " << curr_vel << ", " << cruise_vel << ", " << max_speed << ", 0.0" <<", CruiseWithLeader" << endl;
            return cruise_vel;

        }
    }else{

        //   //    //    //  //  //  //  //  //  //
        //       CLASSIC CRUISE CONTROL MODE     //
        //  //    //    //  //   //  //  //  //  //
        //   Speed Control-Leader doesn't exist  //
        //  //  //  //  //  //   //  //  //  //  //

        //calculate step
        float g_v = - lambda_v * (curr_vel - max_speed);  //sistema dinâmico linear
        float cruise_vel = curr_vel + (timestep * g_v);      //método de euler

        //Show info in terminal
        RCLCPP_INFO(this->get_logger(),"------Classic Cruise Mode------");
        RCLCPP_INFO(this->get_logger(),"Cruise Set Speed: %f",max_speed);
        RCLCPP_INFO(this->get_logger(),"Current Speed: %f",curr_vel);
        RCLCPP_INFO(this->get_logger(),"--------------------------------------------------");

        //logging to txt file
        //cout << "0.0" << ", " << "0.0" << ", " << curr_vel << ", " << cruise_vel << ", " << max_speed << ", 0.0" <<", CruiseMode" << endl;
        return cruise_vel;
    }


}

void adaptive_cruise_control_hunter_class::PublishData()
{
    vel.linear.x=AdaptiveCruiseControl(target_vel);
    VelPublisher->publish(vel);
}
void adaptive_cruise_control_hunter_class::setEnableACC(bool a){
    enable_acc=a;
    if(enable_acc){
        target_vel=curr_vel;
        RCLCPP_INFO(this->get_logger(),"[ACC] ADAPTIVE CRUISE CONTROL ACTIVATED!");

        //logging to txt file
        //cout << " ACC ACTIVATED" << endl;
    }else{
        RCLCPP_INFO(this->get_logger(),"[ACC] ADAPTIVE CRUISE CONTROL DEACTIVATED!");

        //logging to txt file
        //cout << " ACC DEACTIVATED" << endl;
    }
}



int main(int argc, char **argv)
{  
    using namespace std;

    //open file for logging
    //freopen( "LOG_HUNTER_ACC.txt", "w", stdout );
    //freopen( "error.txt", "w", stderr );
    
    //cout << "HUNTER_ACC_LOG" << endl;
    //cout << "RelDistance, SafeDistance, VelHost, VelTarget, VelDesejadaACC, LeaderVel, Mode" << endl;
    //cerr << "Error message" << endl;

    //Initializing flags
    request_received=false;
    enable_global=false;

    //Initializing ROS2
    rclcpp::init(argc,argv); //ROS2

    auto node = std::make_shared<adaptive_cruise_control_hunter_class>(); //ROS2
    rclcpp::Rate rate(20);
    
    while(rclcpp::ok())
    { 
        //MODE SWITCH REQUEST
        if(request_received){
            request_received=false; //clear flag
            node->setEnableACC(enable_global); //UPDATE MODE
        }
        if (node->enable_acc){ //IF ACC MODE IS ACTIVE
            
            node->PublishData();
        }
        
        
        rclcpp::spin_some(node);
	    rate.sleep();
    
        
    }
   
    rclcpp::shutdown();
}