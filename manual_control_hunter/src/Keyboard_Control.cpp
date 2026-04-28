#include <Keyboard_Control.hpp>

keyboard_control_class::keyboard_control_class()
: Node("KeyboardControl")
{
 //SUBSCRIBERS
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SteeringSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr StartSub;
    ThrottleSub = this->create_subscription<std_msgs::msg::Float32>("throttle_topic", 1, std::bind(&keyboard_control_class::Throttle_Callback, this, _1)); 
    SteeringSub = this->create_subscription<std_msgs::msg::Float32>("steering_topic", 1, std::bind(&keyboard_control_class::Steering_Callback, this, _1));
    StartSub = this->create_subscription<std_msgs::msg::Bool>("start_topic", 1, std::bind(&keyboard_control_class::Start_Callback, this, _1));
    KeyboardSub = this->create_subscription<std_msgs::msg::Char>("keyboard_control_topic", 1, std::bind(&keyboard_control_class::KeyboardControl_Callback, this, _1));
    key=' ';
//PUBLISHER
    auto VelPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    vel = geometry_msgs::msg::Twist();

//INITS
    
    start_moving = false;
    robot_state = robot_stop;

    steering_angle = 0;
    linear_velocity = 0;
    
  
}
keyboard_control_class::~keyboard_control_class()
{

}

void keyboard_control_class::PublishData()
{
    auto PublishVel_Node = rclcpp::Node::make_shared("Publish_Vel_Node"); //2025
    auto VelPublisher = PublishVel_Node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    vel.linear.x=linear_velocity;
    vel.angular.z=steering_angle;
    VelPublisher->publish(vel);
}
void keyboard_control_class::MovementControl(){
     switch (robot_state){
        case go_forward:
            steering_angle=0;
            if (linear_velocity>=MAX_LINEAR_SPEED) return;
            linear_velocity+=0.05;
            
            
        break;
        case go_backwards:
            steering_angle=0;
            if (linear_velocity<=(-MAX_LINEAR_SPEED)) return;
            linear_velocity-=0.05;
            
            
        break;
        case turn_left:
            if (steering_angle>=(MAX_ANGULAR_SPEED)) return;
                steering_angle+=0.04;
                
        break;
        case turn_right:
            if (steering_angle<=(-MAX_ANGULAR_SPEED)) return;
                steering_angle-=0.04;
              
        break;
        case robot_stop:
            linear_velocity=0.0;
            steering_angle=0.0;
        break;
        case robot_standby:
            linear_velocity=linear_velocity;
        break;
        default:
            printf("invalid!");
        }
      //  robot_state=robot_standby;

}

void keyboard_control_class::ConvergeToZero(){
    
    //VELOCIDADE LINEAR
        if(linear_velocity>0){
            linear_velocity-=0.02;
            if (linear_velocity<0) linear_velocity=0;    
        } 
        if(linear_velocity<0) {
            linear_velocity+=0.02;
            if(linear_velocity>0) linear_velocity=0;
        }
    

    //STEERING ANGLE
    if(steering_angle>0){
        steering_angle-=0.02;
        if (steering_angle<0) steering_angle=0;    
    } 
    if(steering_angle<0) {
        steering_angle+=0.02;
        if(steering_angle>0) steering_angle=0;
    }
}
void keyboard_control_class::Start_Callback(const std_msgs::msg::Bool::ConstPtr& start_received)
{
    if (start_received->data==true){
        start_moving=true;
    }else{
        start_moving=false;
    }
}
void keyboard_control_class::Throttle_Callback(const std_msgs::msg::Float32::ConstPtr& throttle_received)
{

    
}
void keyboard_control_class::Steering_Callback(const std_msgs::msg::Float32::ConstPtr& steering_received)
{
    
   
}
void keyboard_control_class::KeyboardControl_Callback(const std_msgs::msg::Char::ConstPtr& key_received)
{
    key=key_received->data;
    switch (key){
        case 'w':
            robot_state=go_forward;
        break;
        case 's':
            robot_state=go_backwards;
        break;
        case 'a':
            robot_state=turn_left;
        break;
        case 'd':
            robot_state=turn_right;
        break;
        case 32:
            robot_state=robot_stop;
        break;
        default:
            robot_state=robot_standby;
        }
   
}
int main(int argc, char **argv)
{  
    
    rclcpp::init(argc,argv);    //ROS2
    auto node = std::make_shared<keyboard_control_class>(); //ROS2
    rclcpp::Rate rate(20);
    RCLCPP_INFO(node->get_logger(),"[Manual Control Active]");
    float speed;
    float steer;

    while(rclcpp::ok())
    { 
        rclcpp::spin_some(node);
	    rate.sleep();
        node->MovementControl();
       // node->ConvergeToZero();
        node->PublishData();
        RCLCPP_INFO(node->get_logger(),"LINEAR VELOCITY: %f | STEERING ANGLE: %f", node->linear_velocity, node->steering_angle);
    }
    rclcpp::shutdown();
}