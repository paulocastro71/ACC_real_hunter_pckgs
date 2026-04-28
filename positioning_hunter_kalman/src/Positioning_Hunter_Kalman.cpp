#include <Positioning_Hunter_Kalman.hpp>


positioning_hunter_class::positioning_hunter_class()
: Node("Positioning")
{
    //qos profile that matches the qos of ros_gps message (package do CEiiA)
    rclcpp::QoS qos_profile=rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);


 //SUBSCRIBERS

    //subscribe to GPS message which is type NavSatFix
        //this GPS data already went through the correction service
    GnssSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("device/gps/navsatfix", qos_profile, std::bind(&positioning_hunter_class::GNSS_Callback, this, _1));
    
    //subscribing to hunter odometry for speed data
    OdomSub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&positioning_hunter_class::Odometry_Callback, this, _1));
    
    //Subscribing to Zedx Camera node for IMU data -> Orientation in Quaternion
        //Zedx already provides filtered IMU data
    ZedSub = this->create_subscription<sensor_msgs::msg::Imu>("zed/zed_node/imu/data", 1, std::bind(&positioning_hunter_class::ZedOrientation_Callback, this, _1));
    

//PUBLISHERS
    
    //publish own vehicle pose - publishing raw X, Y coordinates
    PosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_pose",1);
    pose_msg = geometry_msgs::msg::PoseStamped();

    //publish own vehicle pose - publish X, Y coordinates with much lower value
    //example: (instead of 559123.00 meters in x, it publishes 123.00 meters)
    Pose2Publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_pose_pedroso",1);
    pose2_msg = geometry_msgs::msg::PoseStamped();

    //this pose message will be published in tf2 tree 
    geo_pose = geometry_msgs::msg::PoseStamped();

    //Publishing own vehicle speed
    SpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("vehicle_speed",1);
    speed_msg= std_msgs::msg::Float32();

//Measure Latency for testing and benchmark purposes
    LatencyPublisher = this->create_publisher<std_msgs::msg::Bool>("pingl",1);
    latency_msg = std_msgs::msg::Bool();
    LatencySub = this->create_subscription<std_msgs::msg::Bool>("pongl", 1, std::bind(&positioning_hunter_class::Latency_Callback, this, _1));


//INITS
    timestep=0.05; //ros rate is set at 20
    orientation = 0;
    x_robot = 0.0;
    y_robot = 0.0;
    phi_robot = 0.0;
    speed=0;
    //verification flag
    received_gnss_data=false;

    //initialize IMU parameters
    o_x=0;o_y=0;o_z=0;o_w=0;
    GyroX=0;GyroY=0;GyroZ=0; AccX=0;AccY=0;AccZ=0;

    //initalize gnss parameters
    latitude=0; longitude=0; altitude=0;

    //"utm_zone" : Leaving this parameter as 0 indicates to the function that deals with it
    // that it should calculate the zone itself.
    //For Portugal, UTM Zone is 29 (29 North);
    utm_zone = 0;

    x_geo=0;
    y_geo=0;
    x_geo_filtered=0;
    y_geo_filtered=0;

    first_x_geo=0;
    first_y_geo=0;
    last_x_geo=0;
    last_y_geo=0;

    calibrated=false;
    first_position_saved=false;
    last_position_saved=false;

    angle_offset=0;


}
positioning_hunter_class::~positioning_hunter_class()
{

}

//Calback that deals with GPS data, reading the topic from ros_gps node
void positioning_hunter_class::GNSS_Callback(const sensor_msgs::msg::NavSatFix::ConstPtr& gnss_received)
{
    //Before copying the data, we make sure it is not nan type. otherwise it crashes the kalman filter!
    //it happened a few times that the GPS data provided is nan and we lose positioning data
    if(!isnan(gnss_received->latitude) && !isnan(gnss_received->longitude) ){
        latitude=gnss_received->latitude;
        longitude=gnss_received->longitude;
        altitude=gnss_received->altitude;
    }
    received_gnss_data=true;

}


void positioning_hunter_class::Odometry_Callback(const nav_msgs::msg::Odometry::ConstPtr& odom_received){
        //Linear Velocity
         float velocity_x = odom_received->twist.twist.linear.x;
         float velocity_y = odom_received->twist.twist.linear.y;

        //Speed
         speed = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);

         //Position from odometry - not in use
         float position_hunter_x=odom_received->pose.pose.position.x;
         float position_hunter_y=odom_received->pose.pose.position.y;
         x_robot=position_hunter_x;
         y_robot=position_hunter_y;
         
}

void positioning_hunter_class::ZedOrientation_Callback(const sensor_msgs::msg::Imu::ConstPtr& ori_received){


         //Orientation from Odometry
         o_x=ori_received->orientation.x;
         o_y=ori_received->orientation.y;
         o_z=ori_received->orientation.z;
         o_w=ori_received->orientation.w;

         tf2::Quaternion odom_quat;
         odom_quat.setX(o_x);
         odom_quat.setY(o_y);
         odom_quat.setZ(o_z);
         odom_quat.setW(o_w);

         odom_quat = odom_quat * quat;

         o_x = odom_quat.x();
         o_y = odom_quat.y();
         o_z = odom_quat.z();
         o_w = odom_quat.w();


         //Yaw (orientation in relation to z axis)
         phi_robot = atan2(2*(o_x*o_y + o_z*o_w),1-2*(o_y*o_y + o_z*o_z));
         
}

 //calculates orientation in relation to world frame with simple geometry
void positioning_hunter_class::Calibrate_Orientation(){
   
    double dx=last_x_geo - first_x_geo;
    double dy=last_y_geo - first_y_geo;

    angle_offset = atan2(dy,dx);
    quat.setRPY(0,0,angle_offset);
    RCLCPP_INFO(this->get_logger(),"CALIBRATED SUCCESSFULY!");
   

}

//Latency Measurement Purposes
void positioning_hunter_class::SendPing(){
    latency_msg.data=true;
    t0 = chrono::system_clock::now();
    LatencyPublisher->publish(latency_msg);
    
}
void positioning_hunter_class::Latency_Callback(const std_msgs::msg::Bool::ConstPtr& ping_received){

         auto t1 = chrono::system_clock::now();
         auto duration_t0 = t0.time_since_epoch();
         auto duration_t1 = t1.time_since_epoch();

         auto mili_t0 = chrono::duration_cast<chrono::nanoseconds>(duration_t0).count();
         auto mili_t1 = chrono::duration_cast<chrono::nanoseconds>(duration_t1).count();
         auto latencia = (mili_t1 - mili_t0)/2;
         cout << latencia << "--------------------------------------------------------------------------------" << endl;         
}


void positioning_hunter_class::PublishData()
{

    //PREPARE MESSAGE TO SEND
        pose_msg.header=pose_header_stamp;

    //position converted to cartesian XY coordinates
        pose_msg.pose.position.x=x_geo_filtered;
        pose_msg.pose.position.y=y_geo_filtered;
        pose_msg.pose.position.z=altitude;

    //Orientation in quaternion
        pose_msg.pose.orientation.x=o_x;
        pose_msg.pose.orientation.y=o_y;
        pose_msg.pose.orientation.z=o_z;
        pose_msg.pose.orientation.w=o_w;

    //Speed
        speed_msg.data=speed;

    //Position with lower values
        pose2_msg=pose_msg;
        pose2_msg.pose.position.x-=559000;
        pose2_msg.pose.position.y-=4589000;


    //Publish everything
        PosePublisher->publish(pose_msg);
        Pose2Publisher->publish(pose2_msg);
        SpeedPublisher->publish(speed_msg);

}

//Converts GPS data to (X, Y) Coordinates
void positioning_hunter_class::Convert_GNSS_Geo()
{

    //WGS84 GPS -> WGS84 UTM
    //GNSS gets converted to X, Y which will be used as input for Kalman Filter
    LatLonToUTMXY(latitude,longitude,utm_zone,x_geo,y_geo);

    //Logging to terminal for visual feedback
    //X Y (not filtered)
    RCLCPP_INFO(this->get_logger(),"X: %f, Y: %f",x_geo,y_geo);
    RCLCPP_INFO(this->get_logger(),"Yaw: %f",phi_robot);
    RCLCPP_INFO(this->get_logger(),"Angle Offset: %f",angle_offset);

    //log file
    cout << setprecision(10);
    cout << lat_filtered << ", " << lon_filtered << ", " << x_geo_filtered << ", " << y_geo_filtered << ", " << latitude << ", " << longitude << ", " << x_geo << ", " << y_geo << ", " << phi_robot << ", " << angle_offset << ", " << speed << endl;

}

//For comparison and plot purposes in a map, we convert from X Y (filtered) back to GPS
void positioning_hunter_class::ConvertFiltered(bool flag, double utm_x, double utm_y)
{
    if(!flag) return; //flag indicates that the filter was initialized and filtered data exists
    x_geo_filtered=utm_x;
    y_geo_filtered=utm_y;

    //geo_pose is the position (filtered) in world/geo frame ("host" is child frame)
    //this will be published to tf2 tree
    geo_pose.pose.position.x=x_geo_filtered;
    geo_pose.pose.position.y=y_geo_filtered;
    geo_pose.pose.position.z=altitude;
    geo_pose.pose.orientation.x=o_x;
    geo_pose.pose.orientation.y=o_y;
    geo_pose.pose.orientation.z=o_z;
    geo_pose.pose.orientation.w=o_w;
    
    //X Y -> GPS
    UTMXYToLatLon(x_geo_filtered,y_geo_filtered,29,false,lat_filtered,lon_filtered);
    lat_filtered=RadToDeg(lat_filtered);
    lon_filtered=RadToDeg(lon_filtered);

    //Show Results in Terminal
    RCLCPP_INFO(this->get_logger(),"X_filt: %f, Y_filt: %f",x_geo_filtered,y_geo_filtered);
    RCLCPP_INFO(this->get_logger(),"LATfilt: %f, LONfiltered: %f",lat_filtered,lon_filtered);
    
}

//This is part of IMU calibration routine
//this two positions will be used 
void positioning_hunter_class::Save_First_Position(){
    first_x_geo=x_geo;
    first_y_geo=y_geo;
    first_position_saved=true;
}
void positioning_hunter_class::Save_Last_Position(){
    last_x_geo=x_geo;
    last_y_geo=y_geo;
    last_position_saved=true;
}



int main(int argc, char **argv)
{  
    int i=0;
    rclcpp::init(argc,argv);    //ROS2

    auto node = std::make_shared<positioning_hunter_class>(); //Positioning Node - Published vehicle state
    auto tf_node = std::make_shared<TfBroadcaster>();         //TF2 Node - Publishes the transform "host" <-> "geo"
    rclcpp::Rate rate(20);

    STATE_ STATE = STANDBY; //FIRST STATE BY DEFAULT

    int n = 4; // Number of states
    int m = 4; // Number of measurements

    double sigma_posx = 0.15;  //measurement noise covariance position_x
    double sigma_posy = 0.15;  //measurement noise covariance position_y
    double sigma_velx = 0.01;//measurement noise covariance velocity_x
    double sigma_vely = 0.01;//measurement noise covariance velocity_y

    double dt = 0.05; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Initialize all kalman matrices
    //transition matrix
    A <<1, 0, dt, 0,     
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    // Measurement gain
    C << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    //process noise matrix
    Q << 0.0000007812,       0.0000,   0.00003125,     0.0000,
               0.0000, 0.0000007812,       0.0000, 0.00003125,
           0.00003125,       0.0000,      0.00125,     0.0000,
               0.0000,   0.00003125,       0.0000,    0.00125;

    //measurement noise matrix
    R << sigma_posx,     0.0000,     0.0000,      0.0000,
             0.0000, sigma_posy,     0.0000,     0.0000,
             0.0000,     0.0000, sigma_velx,     0.0000,
             0.0000,     0.0000,     0.0000, sigma_vely;

    //confidence in initial measurements
    P << 0.05, 0, 0, 0,
         0, 0.05, 0, 0,
         0, 0, 0.02, 0,
         0, 0, 0, 0.02;
         
    /* //show matrices in terminal for initial testing
    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;
    */

    //Create Kalman Filter
    auto kf = std::make_shared<KalmanFilter>(dt, A, C, Q, R, P);

    //initialize flag
    bool filter_initialized = false;

    Eigen::VectorXd x0(4);
    Eigen::VectorXd z(4);
    Eigen::VectorXd kf_output(4);

    //open a log file
    freopen( "LOG_HUNTER_POSITIONING.txt", "w", stdout );
    cout << "HUNTER_POSITIONING_LOG" << endl;
    cout << "LAT_filt,LON_filt,XGEO_filt, YGEO_filt, LAT, LONG, XGEO, YGEO, YAW, AngleOffset, Velocity" << endl;

    //inform node state in terminal
    RCLCPP_INFO(node->get_logger(),"Kalman Filter Ready...");
    RCLCPP_INFO(node->get_logger(),"Positioning Initialized!");
    RCLCPP_INFO(node->get_logger(),"Waiting to receive GNSS data...");
    
    double time_filter=0;
    
    
    while(rclcpp::ok())
    { 
        /*
        //Starts publishing right away, publishing everything as zero in beginning
        node->PublishData();

        //node->received_gnss_data = true;
        //Verify if there is GNSS dat to work with
        if(node->received_gnss_data){

            //Save Filtered Pose to ROS2 msg and Convert Filtered data back to GPS
            node->ConvertFiltered(filter_initialized, kf_output[0], kf_output[1]);

            //Convert GPS to X Y coordinates to be used in Filter
            node->Convert_GNSS_Geo();

            //Publish new data to tf tree - find leader relies on this info, not in PublishData()
            tf_node->UpdateTransform(node->geo_pose);

            //save the very first position after receiving GNSS data
            if(!node->first_position_saved){
                node->Save_First_Position();
            }else{
                //if we have the first position but not the last and 10 seconds have passed
                if(node->first_position_saved && !node->last_position_saved && i>200){
                    //save current position
                    node->Save_Last_Position();

                    //calibrate orientation with simple calculation using both positions
                    node->Calibrate_Orientation();
                }
            }
            //if ten seconds have passed the filter starts
            if(i>=210){
                if(i==210){ //run once
                    

                //feed measurements to filter
                z << node->x_geo, node->y_geo, (node->speed * cos(node->phi_robot)), (node->speed * sin(node->phi_robot));
                RCLCPP_INFO(node->get_logger()," Mesurements Done!");
                kf->update(z);
                RCLCPP_INFO(node->get_logger()," FilterUpdated!");
                
                //get filter output
                kf_output=kf->state().transpose();
                RCLCPP_INFO(node->get_logger()," Output Calculated!");
                

                //latency measurement
                if(i%100==0){
                    node->SendPing();
                }   
                
            }
            
            i++;
        }*/

        switch (STATE){
        case STANDBY:
            RCLCPP_INFO(node->get_logger(),"WAITING FOR GNSS DATA TO START!");
            if(node->received_gnss_data) STATE = CALIBRATION;
            break;

        case CALIBRATION:
            node->Convert_GNSS_Geo();
            RCLCPP_INFO(node->get_logger()," CALIBRATION ROUTINE ACTIVE: MOVE FORWARD IN A STRAIGHT LINE");
            if(!node->first_position_saved){
                node->Save_First_Position();
            }else if(i>200){
                //save current position
                    node->Save_Last_Position();

                //calibrate orientation with simple calculation using both positions
                    node->Calibrate_Orientation();
                    STATE = INIT_FILTER;
            }
            i++;
            break;

        case INIT_FILTER:
            node->Convert_GNSS_Geo();
            x0 << node->last_x_geo, node->last_y_geo, 0, 0; 
            filter_initialized=true; //update flag
            kf->init(0,x0);
            RCLCPP_INFO(node->get_logger()," State Vector Initialized!");
            RCLCPP_INFO(node->get_logger()," Filter Initialized!");
            STATE = FILTER;
            break;


        case FILTER:

            RCLCPP_INFO(node->get_logger()," FILTER IS ACTIVE!");
            //Convert GPS to X Y coordinates to be fed to filter
            node->Convert_GNSS_Geo();

            //feed measurements to filter
            z << node->x_geo, node->y_geo, (node->speed * cos(node->phi_robot)), (node->speed * sin(node->phi_robot));
            kf->update(z);
            
            //get filter output
            kf_output=kf->state().transpose();

            //Save Filtered Pose to ROS2 msg and Convert Filtered data back to GPS
            node->ConvertFiltered(filter_initialized, kf_output[0], kf_output[1]);

            //Starts publishing right away, publishing everything as zero in beginning
            node->PublishData();

            //Publish new data to tf tree - find leader relies on this info, not in PublishData()
            tf_node->UpdateTransform(node->geo_pose);


            break;
            default:
            RCLCPP_INFO(node->get_logger()," INVALID STATE! ");
        }  
    

        rclcpp::spin_some(node);
        rclcpp::spin_some(tf_node);
        
	    rate.sleep();
        
    }
    rclcpp::shutdown();
}
