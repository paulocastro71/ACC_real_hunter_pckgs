#include <Positioning_Ben_Kalman.hpp>

positioning_ben_class::positioning_ben_class()
: Node("Positioning")
{

    rclcpp::QoS qos_profile=rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
 //SUBSCRIBERS

    //GnssSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("device/gps/navsatfix_leader", qos_profile, std::bind(&positioning_ben_class::GNSS_Callback, this, _1));
    //ImuSub = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&positioning_ben_class::Imu_Callback, this, _1));
    //SpeedometerSub = this->create_subscription<std_msgs::msg::Float32>("carla/ego_vehicle/speedometer", 1, std::bind(&positioning_ben_class::Speedometer_Callback, this, _1));
    //OdomSub = this->create_subscription<nav_msgs::msg::Odometry>("odom_leader", 1, std::bind(&positioning_ben_class::Odometry_Callback, this, _1));
    //ZedSub = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&positioning_ben_class::ZedOrientation_Callback, this, _1));
    
    BenSub = this->create_subscription<gps_msgs::msg::GPSFix>("ben_data", qos_profile, std::bind(&positioning_ben_class::GPSFix_Callback, this, _1));
    
    
    //PUBLISHERS

    //Publisher for Latitude and Longitude 
    PosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_one_pose_",1);
    pose_msg = geometry_msgs::msg::PoseStamped();

    //Publisher for UTM X Y Coordinates
    PoseMetersPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_one_pose_meters",1);
    pose_meters_msg = geometry_msgs::msg::PoseStamped();

    //Publisher for UTM X Y Coordinates with less units (130 meters instead of 559130 meters) (para parking)
    PoseParkPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_one_pose_park",1);
    pose_park_msg = geometry_msgs::msg::PoseStamped();

    //pose msg for tf2
    geo_pose = geometry_msgs::msg::PoseStamped();

    //Publisher for Speed
    SpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("vehicle_one_speed",1);
    speed_msg= std_msgs::msg::Float32();

    //Latency measurements//
    latency_msg.data=true;
    LatencyPublisher = this->create_publisher<std_msgs::msg::Bool>("pongl",1);
    latency_msg= std_msgs::msg::Bool();
    LatencySub = this->create_subscription<std_msgs::msg::Bool>("pingl", 1, std::bind(&positioning_ben_class::Latency_Callback, this, _1));


//INITS
    timestep=0.05;
    orientation=0;
    x_robot = 0.0;
    y_robot = 0.0;
    phi_robot = 0.0;
    speed=0;
    received_ben_data=false;
    received_imu_data=false;

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

positioning_ben_class::~positioning_ben_class()
{
    //destructor
}

void positioning_ben_class::Latency_Callback(const std_msgs::msg::Bool::ConstPtr& ping_received)
{
    LatencyPublisher->publish(latency_msg);
}

void positioning_ben_class::GNSS_Callback(const sensor_msgs::msg::NavSatFix::ConstPtr& gnss_received)
{

    if(!isnan(gnss_received->latitude) && !isnan(gnss_received->longitude) ){
        //Retrieve GNSS data
        latitude=gnss_received->latitude;
        longitude=gnss_received->longitude;
        altitude=gnss_received->altitude;
    }
    //signaling that calibration can start
    received_gnss_data=true;

}



void positioning_ben_class::Odometry_Callback(const nav_msgs::msg::Odometry::ConstPtr& odom_received){

        //Linear Velocity
         float velocity_x = odom_received->twist.twist.linear.x;
         float velocity_y = odom_received->twist.twist.linear.y;

        //Speed
         speed = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
         
         if (velocity_x<0) speed*=-1;

         //Position from odometry
         float position_hunter_x=odom_received->pose.pose.position.x;
         float position_hunter_y=odom_received->pose.pose.position.y;
         x_robot=position_hunter_x;
         y_robot=position_hunter_y;
        
         
}

void positioning_ben_class::ZedOrientation_Callback(const sensor_msgs::msg::Imu::ConstPtr& ori_received){

         

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


void positioning_ben_class::GPSFix_Callback(const gps_msgs::msg::GPSFix::ConstPtr& gps_received)
{

    if(!isnan(gps_received->latitude) && !isnan(gps_received->longitude) ){ //verify if data is valid
        //Retrieve GNSS data
        latitude=gps_received->latitude;
        longitude=gps_received->longitude;
        altitude=gps_received->altitude;
    }

    

     //Orientation from Odometry
        // o_x=gps_received->orientation.x;
        //o_y=gps_received->orientation.y;
        //o_z=gps_received->orientation.z;
        //o_w=gps_received->orientation.w;

        tf2::Quaternion odom_quat;
        //odom_quat.setX(o_x);
        //odom_quat.setY(o_y);
        //odom_quat.setZ(o_z);
        //odom_quat.setW(o_w);
        odom_quat.setRPY(0,0,gps_received->track);

        odom_quat = odom_quat * quat;

        o_x = odom_quat.x();
        o_y = odom_quat.y();
        o_z = odom_quat.z();
        o_w = odom_quat.w();


        //Yaw (orientation in relation to z axis)
        phi_robot = atan2(2*(o_x*o_y + o_z*o_w),1-2*(o_y*o_y + o_z*o_z));

        speed = gps_received->speed;

        //signaling that calibration can start
        received_ben_data=true;

}


void positioning_ben_class::Calibrate_Orientation(){

    double dx=last_x_geo - first_x_geo;
    double dy=last_y_geo - first_y_geo;

    angle_offset = atan2(dy,dx);
    quat.setRPY(0,0,angle_offset);
    RCLCPP_INFO(this->get_logger(),"CALIBRATED!!!!!!!!!!!!!!!!!");
   

}
void positioning_ben_class::PublishData()
{

    //PREPARE MESSAGE TO SEND
        pose_msg.header=pose_header_stamp;
        pose_meters_msg.header=pose_header_stamp;
        pose_park_msg.header=pose_header_stamp;

    //Position
        //position from odometry (x,y) with center located at the initial pose of the robot
        //pose_msg.pose.position.x=x_robot;
        //pose_msg.pose.position.y=y_robot;

        //position from GNSS (longitude, latitude)
        //pose_msg.pose.position.x=x_geo_filtered;
        //pose_msg.pose.position.y=y_geo_filtered;
        pose_msg.pose.position.x=lon_filtered;
        pose_msg.pose.position.y=lat_filtered;
        pose_msg.pose.position.z=altitude;

        pose_meters_msg.pose.position.x=x_geo_filtered;
        pose_meters_msg.pose.position.y=y_geo_filtered;
        pose_meters_msg.pose.position.z=altitude;

        pose_park_msg.pose.position.x= x_geo_filtered - 559000.0;
        pose_park_msg.pose.position.y= y_geo_filtered - 4589000.0;
        pose_park_msg.pose.position.z=altitude;

    //ORIENTATION
        pose_msg.pose.orientation.x=o_x;
        pose_msg.pose.orientation.y=o_y;
        pose_msg.pose.orientation.z=o_z;
        pose_msg.pose.orientation.w=o_w;

        pose_meters_msg.pose.orientation = pose_msg.pose.orientation;
        pose_park_msg.pose.orientation = pose_msg.pose.orientation;
    
    //SPEED
        speed_msg.data=speed;

    //PUBLISH
        PosePublisher->publish(pose_msg);
        PoseMetersPublisher->publish(pose_meters_msg);
        PoseParkPublisher->publish(pose_park_msg);
        SpeedPublisher->publish(speed_msg);

}

void positioning_ben_class::Convert_GNSS_Geo()
{
    //WGS84 GPS -> WGS84 UTM
    LatLonToUTMXY(latitude,longitude,utm_zone,x_geo,y_geo);
    geo_pose.pose.position.x=x_geo_filtered;
    geo_pose.pose.position.y=y_geo_filtered;
    geo_pose.pose.position.z=altitude;
    geo_pose.pose.orientation.x=o_x;
    geo_pose.pose.orientation.y=o_y;
    geo_pose.pose.orientation.z=o_z;
    geo_pose.pose.orientation.w=o_w;
    RCLCPP_INFO(this->get_logger(),"X: %f, Y: %f",x_geo,y_geo);
    RCLCPP_INFO(this->get_logger(),"Yaw: %f",phi_robot);
    RCLCPP_INFO(this->get_logger(),"Angle Offset: %f",angle_offset);
    cout << setprecision(10);
    cout << lat_filtered << ", " << lon_filtered << ", " << x_geo_filtered << ", " << y_geo_filtered << ", " << latitude << ", " << longitude << ", " << x_geo << ", " << y_geo << ", " << phi_robot << ", " << angle_offset << endl;
    //cout << lat_filtered << ", " << lon_filtered << ", " << x_geo_filtered << ", " << y_geo_filtered << endl;
}


void positioning_ben_class::ConvertFiltered(bool flag, double utm_x, double utm_y)
{   

    if(!flag) return;
    x_geo_filtered=utm_x;
    y_geo_filtered=utm_y;
    //WGS84 GPS -> WGS84 UTM

    UTMXYToLatLon(x_geo_filtered,y_geo_filtered,29,false,lat_filtered,lon_filtered);
    lat_filtered=RadToDeg(lat_filtered);
    lon_filtered=RadToDeg(lon_filtered);
    RCLCPP_INFO(this->get_logger(),"X_filt: %f, Y_filt: %f",x_geo_filtered,y_geo_filtered);
    RCLCPP_INFO(this->get_logger(),"LATfilt: %f, LONfiltered: %f",lat_filtered,lon_filtered);
    
}

//Save positions for calibration routine
void positioning_ben_class::Save_First_Position(){

    first_x_geo=x_geo;
    first_y_geo=y_geo;
    first_position_saved=true;

}

void positioning_ben_class::Save_Last_Position(){

    last_x_geo=x_geo;
    last_y_geo=y_geo;
    last_position_saved=true;

}

bool positioning_ben_class::SensorsReady(){

   // if(!received_gnss_data) RCLCPP_ERROR(this->get_logger(),"NO GNSS DATA RECEIVED!");
   // if(!received_imu_data) RCLCPP_ERROR(this->get_logger(),"NO IMU DATA RECEIVED!");
    if(!received_ben_data) RCLCPP_ERROR(this->get_logger(),"NO BEN DATA RECEIVED!");
    return received_ben_data; //&& received_imu_data;
    
}

int main(int argc, char **argv)
{  

    int i=0;
    rclcpp::init(argc,argv);    //ROS2

    auto node = std::make_shared<positioning_ben_class>(); //Positioning Node - Published vehicle state
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

  // Discrete LTI projectile motion, measuring position only
    A <<1, 0, dt, 0,     // first row
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    C << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

  // Reasonable covariance matrices
    Q << 0.0000007812,       0.0000,   0.00003125,     0.0000,
               0.0000, 0.0000007812,       0.0000, 0.00003125,
           0.00003125,       0.0000,      0.00125,     0.0000,
               0.0000,   0.00003125,       0.0000,    0.00125;

    R << sigma_posx,     0.0000,     0.0000,      0.0000,
             0.0000, sigma_posy,     0.0000,     0.0000,
             0.0000,     0.0000, sigma_velx,     0.0000,
             0.0000,     0.0000,     0.0000, sigma_vely;

    P << 0.05, 0, 0, 0,
         0, 0.05, 0, 0,
         0, 0, 0.02, 0,
         0, 0, 0, 0.02;
         

    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;

    auto kf = std::make_shared<KalmanFilter>(dt, A, C, Q, R, P);
    bool filter_initialized = false;

    Eigen::VectorXd x0(4);
    Eigen::VectorXd z(4);
    Eigen::VectorXd kf_output(4);

    freopen( "LOG_HUNTER_POSITIONING.txt", "w", stdout );
    cout << "HUNTER_POSITIONING_LOG" << endl;
    cout << "LAT_filt,LON_filt,XGEO_filt, YGEO_filt, LAT, LONG, XGEO, YGEO, YAW, AngleOffset" << endl;


    double time_filter=0;

    while(rclcpp::ok())
    { 
       /*

        //node->received_gnss_data = true;
        if(node->received_gnss_data){
            node->ConvertFiltered(filter_initialized, kf_output[0], kf_output[1]);
            node->Convert_GNSS_Geo();
            node->PublishData();
            //tf_node->UpdateTransform(node->geo_pose);
            if(!node->first_position_saved){
                node->Save_First_Position();
            }else{
                if(node->first_position_saved && !node->last_position_saved && i>200){
                    node->Save_Last_Position();
                    node->Calibrate_Orientation();
                }
            }
            if(i>=210){
                if(i==210){
                    x0 << node->first_x_geo, node->first_y_geo, 0, 0; 
                    RCLCPP_INFO(node->get_logger()," State Vector Initialized!");
                }
                if(!filter_initialized){
                    filter_initialized=true;
                    kf->init(0,x0);
                    RCLCPP_INFO(node->get_logger()," Filter Initialized!");
                }
                z << node->x_geo, node->y_geo, (node->speed * cos(node->phi_robot)), (node->speed * sin(node->phi_robot));
                RCLCPP_INFO(node->get_logger()," Mesurements Done!");
                kf->update(z);
                RCLCPP_INFO(node->get_logger()," FilterUpdated!");
                kf_output=kf->state().transpose();
                RCLCPP_INFO(node->get_logger()," Output Calculated!");
                
            }
            i++;
        }
        
        rclcpp::spin_some(node);
        rclcpp::spin_some(tf_node);
        
	    rate.sleep();
    */
        
    switch (STATE){
        case STANDBY:
            
            if(node->SensorsReady()) STATE = CALIBRATION;
            
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
            //runs once to init filter
            
            node->Convert_GNSS_Geo();
            //prepare initial state vector wit current position
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
