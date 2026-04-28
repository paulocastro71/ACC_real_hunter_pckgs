#include <Finding_Leader_Hunter.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

//Constructor
finding_leader_hunter_class::finding_leader_hunter_class()
: tfBuffer(this->get_clock()), tfListener(tfBuffer), Node("Find_Leading_Vehicle")
{
    
    rclcpp::QoS qos_profile=rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    //SUBSCRIBERS
    
        //Get Pose from all nearby vehicles
        PoseSub = this->create_subscription<custom_msgs::msg::PositioningArray>("nearby_hunter", 1, std::bind(&finding_leader_hunter_class::Nearby_Hunter_Callback, this, _1)); 
        GnssSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("device/gps/navsatfix", qos_profile, std::bind(&finding_leader_hunter_class::GNSS_Callback, this, _1));
    
    //PUBLISHERS

        //Publish Leader Pose and Speed
        LeaderPub = this->create_publisher<custom_msgs::msg::Positioning>("leader_pose",1);
    
    //Init Flags
    nearby_cars_flag = false;
    converted = false;

}

finding_leader_hunter_class::~finding_leader_hunter_class(){
    //destructor

}
void finding_leader_hunter_class::GNSS_Callback(const sensor_msgs::msg::NavSatFix::ConstPtr& gnss_received)
{

    lat_hunter=gnss_received->latitude;
    long_hunter=gnss_received->longitude;


}

//RETRIEVE MOTION STATE OF ALL CARS
void finding_leader_hunter_class::Nearby_Hunter_Callback(const custom_msgs::msg::PositioningArray msg){

    nearby_cars_world = msg;
    nearby_cars_flag = true;

}

//CONVERT FROM GEO FRAME TO HOST FRAME
void finding_leader_hunter_class::ConvertFrame(){
    
    //CLEAR ARRAY FOR NEW ITERATION
    nearby_cars_vehicle.positioning_array.clear();
    custom_msgs::msg::Positioning aux;

    //LOGGING INFO TO TERMINAL
    RCLCPP_INFO(this->get_logger(),"Number of Hunters detected: %d",nearby_cars_world.positioning_array.size());

    
    //long_leader=nearby_cars_world.positioning_array.begin()->pose.pose.position.x;
    //lat_leader=nearby_cars_world.positioning_array.begin()->pose.pose.position.y;

    for(int i=0; i<nearby_cars_world.positioning_array.size();i++){

        transform_pose.header = nearby_cars_world.positioning_array[i].pose.header;
        transform_pose.header.stamp = rclcpp::Time();

        try{

            //transform tf2
                //frame : geo (mundo)       -->   frame: host
                    //child_frame: hunterx  -->      child_frame: hunterx
            
            tfBuffer.transform(transform_pose, transformed_pose,"host",tf2::Duration(std::chrono::seconds(1)));
            aux.pose=transformed_pose;
            aux.velx=nearby_cars_world.positioning_array[i].velx;

            //Save transformed poses to new array - this array will be processed to find the leader in the next step
            nearby_cars_vehicle.positioning_array.push_back(aux);
            
            //Log original pose to terminal
            RCLCPP_INFO(this->get_logger(),"ID:%d GeoFrame x: %f, y: %f",i,transform_pose.pose.position.x, transform_pose.pose.position.y);
            
            //log transformed pose to terminal
            RCLCPP_INFO(this->get_logger(),"ID:%d HostFrame x: %f, y: %f",i,transformed_pose.pose.position.x,transformed_pose.pose.position.y);
            
            converted = true;
        }

        catch(tf2::TransformException& ex){
            RCLCPP_INFO(this->get_logger(),"Received an exception trying to transform a point from \"geo\" to \"host\": %s", ex.what());
        }
    }

    
}

//Quaternion to Euler - aux function
float Quat2Euler(const float q0, const float q1,const float q2,const float q3){

    double yaw = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    double pitch = asin(2*(q0*q2 - q3*q1));
    double roll = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
    return yaw;
}

//Find Leader Algorithm
bool finding_leader_hunter_class::Find_Leading_Vehicle(){


    float pos_x=-1;
    float pos_y=-1;
    float ori_z= 3;
    float quat_x=0;
    float quat_y=0;
    float quat_z=0;
    float quat_w=0;

    //aux vars to check distance
    float distance_to_self = 1001;
    float closest_distance_to_self = 1000;
    bool  leader_found=false;

    if (converted==true){ //Check transformed data is available
    
        converted=false;

        //Go through every candidate
        for(int i=0;i<nearby_cars_vehicle.positioning_array.size();i++){
           
            pos_x=nearby_cars_vehicle.positioning_array[i].pose.pose.position.x;
            pos_y=nearby_cars_vehicle.positioning_array[i].pose.pose.position.y;
            quat_x=nearby_cars_vehicle.positioning_array[i].pose.pose.orientation.x;
            quat_y=nearby_cars_vehicle.positioning_array[i].pose.pose.orientation.y;
            quat_z=nearby_cars_vehicle.positioning_array[i].pose.pose.orientation.z;
            quat_w=nearby_cars_vehicle.positioning_array[i].pose.pose.orientation.w;

            //if( (pos_x<5) && (pos_y < 0.15*pos_x) && (pos_y > (-0.15*pos_x))){ //triangle (space too restricted)
            if( (pos_x<10) && (pos_x>0.1) && (pos_y<1) && (pos_y>-1)){  //rectangle

                //Euler orientaton
                ori_z=Quat2Euler(quat_x,quat_y,quat_z,quat_w);

                //check distance to vehicle
                distance_to_self = sqrt(pow(pos_x,2)+pow(pos_y,2));

                //Verify proximityand similar orientation
                if (distance_to_self<closest_distance_to_self && ori_z < 1.5 && ori_z> -1.5){  //Verificaçao de proximidade e orientação

                    closest_distance_to_self = distance_to_self;
                    leading_vehicle = nearby_cars_vehicle.positioning_array[i];
                    
                    leader_found = true;
                }
            }
        }

        if(leader_found){
            RCLCPP_INFO(this->get_logger(),"LEADER FOUND! POSITION: (%f,%f)",leading_vehicle.pose.pose.position.x,leading_vehicle.pose.pose.position.y);
            return true;
        }
    }
    //RCLCPP_INFO(this->get_logger(),"NO LEADING VEHICLE DETECTED!!");
    return false;
}

void finding_leader_hunter_class::PublishData(){
    
    //LOG DISTANCE TO TERMINAL TO ENSURE CORRECT MEASUREMENT
    RCLCPP_INFO(this->get_logger(),"Distance to Leader: %f",sqrt(pow(leading_vehicle.pose.pose.position.x,2)+pow(leading_vehicle.pose.pose.position.y,2)));
    
    //Publish Leader Data
    LeaderPub->publish(leading_vehicle);
}


int main(int argc, char **argv)
{  
    
    //INIT ROS2 AND ROS2 NODE
    rclcpp::init(argc,argv);    
    auto node = std::make_shared<finding_leader_hunter_class>();

    //20hz cycle
    rclcpp::Rate rate(20);


    RCLCPP_INFO(node->get_logger(),"[Find] Find Leader Node Initialized...");

    while(rclcpp::ok())
    { 
        //If there are nearby vehicles
        if(node->nearby_cars_flag){

            //convert from geo frame to host frame
            node->ConvertFrame();

            //Find leader algorithm
            if(node->Find_Leading_Vehicle()){

                //publish data when leader exists
                node->PublishData();
            }
            //clear flags
            node->nearby_cars_flag=false;
        }

        rclcpp::spin_some(node);
	    rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(),"[Find] Exitting...");
    rclcpp::shutdown();
   

}