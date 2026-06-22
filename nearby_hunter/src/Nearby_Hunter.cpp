#include <Nearby_Hunter.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


nearby_hunter_class::nearby_hunter_class()
: Node("Nearby_Hunter")
{
    
    //SUBSCRIBERS

    //Since there is only two hunters to work with we subscribe directly to "vehicle_xxxxx_leader"
    VelocityLeaderSub = this->create_subscription<std_msgs::msg::Float32>("vehicle_speed_leader", 1, std::bind(&nearby_hunter_class::Velocity_Leader_Callback, this, _1));
    PoseLeaderSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("vehicle_pose_leader", 1, std::bind(&nearby_hunter_class::Pose_Leader_Callback, this, _1));

    //vehicle number zero
    geo_pose_zero = geometry_msgs::msg::PoseStamped();
    vehicle_zero = custom_msgs::msg::Positioning();
    vehicle_zero_exists = false;

    //these two additional vehicles are added in case there's more vehicles to work with in the future
    //vehicle one
    VelocityOneSub = this->create_subscription<std_msgs::msg::Float32>("vehicle_one_speed", 1, std::bind(&nearby_hunter_class::V1_Velocity_Callback, this, _1));
    PoseOneSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("vehicle_one_pose_meters", 1, std::bind(&nearby_hunter_class::V1_Pose_Callback, this, _1));
    //vehicle two
    VelocityTwoSub = this->create_subscription<std_msgs::msg::Float32>("vehicle_two_speed", 1, std::bind(&nearby_hunter_class::V2_Velocity_Callback, this, _1));
    PoseTwoSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("vehicle_two_pose", 1, std::bind(&nearby_hunter_class::V2_Pose_Callback, this, _1));

    //vehicle one msgs
    geo_pose_one = geometry_msgs::msg::PoseStamped();
    vehicle_one = custom_msgs::msg::Positioning();
    vehicle_one_exists = false;

    //vehicle two msgs
    geo_pose_two = geometry_msgs::msg::PoseStamped();
    vehicle_two = custom_msgs::msg::Positioning();
    vehicle_two_exists = false;
    

    //PUBLISHER
    //Publish Array with all nearby vehicles
    NearbyHunterPub = this->create_publisher<custom_msgs::msg::PositioningArray>("nearby_hunter",1);
    NearbyHunterMsg = custom_msgs::msg::PositioningArray();

    //oriX=0; oriY=0; oriZ=0; oriW=0;
    longitude=0; latitude=0; altitude=0;
    speed=0;


    //"utm_zone": Kepping this parameter as zero indicates to the function that deals with it that
    //it should calculate the zone itself
    //UTM Zone for Portugal is 29 (29North)
    utm_zone=0;

    lat_hunter_zero=0;
    long_hunter_zero=0;
    yaw_hunter_zero=0;
    
}

nearby_hunter_class::~nearby_hunter_class(){
    //destructor

}


void nearby_hunter_class::Pose_Leader_Callback(const geometry_msgs::msg::PoseStamped::ConstPtr& pose_leader_received)
{
    //Get vehicle zero pose
    vehicle_zero.pose.pose = pose_leader_received->pose;
    vehicle_zero_exists = true;
   
}
void nearby_hunter_class::Velocity_Leader_Callback(const std_msgs::msg::Float32::ConstPtr& velocity_leader_received)
{
    //Get vehicle zero speed
    vehicle_zero.velx.data = velocity_leader_received->data;
    vehicle_zero_exists = true;

}

//------------------VEHICLE ONE------------------------------------------------------------------------------//
void nearby_hunter_class::V1_Pose_Callback(const geometry_msgs::msg::PoseStamped::ConstPtr& pose_one_msg)
{
    //Get vehicle zero pose
    vehicle_one.pose.pose = pose_one_msg->pose;
    vehicle_one_exists = true;
   
}
void nearby_hunter_class::V1_Velocity_Callback(const std_msgs::msg::Float32::ConstPtr& speed_one_msg)
{
    //Get vehicle zero speed
    vehicle_one.velx.data = speed_one_msg->data;
    vehicle_one_exists = true;

}

//------------------VEHICLE TWO---------------------------------------------------------------------------//
void nearby_hunter_class::V2_Pose_Callback(const geometry_msgs::msg::PoseStamped::ConstPtr& pose_two_msg)
{
    //Get vehicle zero pose
    vehicle_two.pose.pose = pose_two_msg->pose;
    vehicle_two_exists = true;
   
}
void nearby_hunter_class::V2_Velocity_Callback(const std_msgs::msg::Float32::ConstPtr& speed_two_msg)
{
    //Get vehicle zero speed
    vehicle_two.velx.data = speed_two_msg->data;
    vehicle_two_exists = true;
}
//-------------------------------------------------------------------------------------------------------//


void nearby_hunter_class::UpdateArray(){
    double x_geo=0;
    double y_geo=0;
    custom_msgs::msg::Positioning msg0;
    custom_msgs::msg::Positioning msg1;
    custom_msgs::msg::Positioning msg2;
    
    NearbyHunterMsg.positioning_array.clear();
    
    if(vehicle_zero_exists){
        
        RCLCPP_INFO(this->get_logger(),"Reading position from other Hunter...");
        msg0 = vehicle_zero;
        msg0.pose.header.frame_id="leader";
        msg0.pose.header.stamp = this->get_clock()->now();
        NearbyHunterMsg.positioning_array.push_back(msg0);
        geo_pose_zero=msg0.pose;
        //lat_hunter_zero = geo_pose_zero.pose.position.y;
        //long_hunter_zero = geo_pose_zero.pose.position.x;
        //LatLonToUTMXY(lat_hunter_zero,long_hunter_zero,utm_zone,x_geo,y_geo);
        //geo_pose_zero.pose.position.x=x_geo;
        //geo_pose_zero.pose.position.y=y_geo;
        RCLCPP_INFO(this->get_logger(),"Hunter detected at:");
        RCLCPP_INFO(this->get_logger(),"X: %f, Y: %f;",geo_pose_zero.pose.position.x,geo_pose_zero.pose.position.y);
        
    }

    if(vehicle_one_exists){
        RCLCPP_INFO(this->get_logger(),"Reading position from other Hunter...");
        msg1 = vehicle_one;
        msg1.pose.header.frame_id="vehicle_one";
        msg1.pose.header.stamp = this->get_clock()->now();
        NearbyHunterMsg.positioning_array.push_back(msg1);
        geo_pose_one=msg1.pose;
        
        RCLCPP_INFO(this->get_logger(),"Hunter detected at:");
        RCLCPP_INFO(this->get_logger(),"X: %f, Y: %f;",geo_pose_one.pose.position.x,geo_pose_one.pose.position.y);
        
    }


    if(vehicle_two_exists){
        RCLCPP_INFO(this->get_logger(),"Reading position from other Hunter...");
        msg2 = vehicle_two;
        msg2.pose.header.frame_id="vehicle_two";
        msg2.pose.header.stamp = this->get_clock()->now();
        NearbyHunterMsg.positioning_array.push_back(msg2);
        geo_pose_two=msg2.pose;
        
        RCLCPP_INFO(this->get_logger(),"Hunter detected at:");
        RCLCPP_INFO(this->get_logger(),"X: %f, Y: %f;",geo_pose_two.pose.position.x,geo_pose_two.pose.position.y);
    } 

}



void nearby_hunter_class::PublishData( ){

    //Publish Nearby Vehicles motion states
    NearbyHunterPub->publish(NearbyHunterMsg); 
}



int main(int argc, char **argv)
{  
    
    rclcpp::init(argc,argv);    
    auto node = std::make_shared<nearby_hunter_class>();
    auto tf_node = std::make_shared<TfBroadcaster>();
    rclcpp::Rate rate(20);

    RCLCPP_INFO(node->get_logger()," Scanning Nearby Vehicles...");

    while(rclcpp::ok())
    { 
        //Check For Existing Vehicles
        node->UpdateArray();

        //Publish data
        node->PublishData();
        
        //Update TF2 Tree with nearby vheicles data. Find Leader relies on this tf2 tree!
        if(node->vehicle_zero_exists) tf_node->UpdateTransform(node->geo_pose_zero);
        if(node->vehicle_one_exists) tf_node->UpdateTransform(node->geo_pose_one);
        if(node->vehicle_two_exists) tf_node->UpdateTransform(node->geo_pose_two);

        rclcpp::spin_some(node);
	    rate.sleep();
    }

    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger()," Exitting...");
}