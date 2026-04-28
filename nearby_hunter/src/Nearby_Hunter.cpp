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

    //vehicle number one
    //...
    

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

void nearby_hunter_class::UpdateArray(){
    double x_geo=0;
    double y_geo=0;
    custom_msgs::msg::Positioning msg0;
    
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

        rclcpp::spin_some(node);
	    rate.sleep();
    }

    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger()," Exitting...");
}