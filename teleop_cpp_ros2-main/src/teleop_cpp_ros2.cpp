#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};


// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
---------------------------
Simple Teleoperation with arrow keys
          ⇧
        ⇦   ⇨
          ⇩

          A
        D   C
          B

---------------------------
t : up (+z)
b : down (-z)
s/S : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
NOTE : Increasing or Decreasing will take affect live on the moving robot.
      Consider Stopping the robot before changing it.
CTRL-C to quit
THIS IS A EXACT REPLICA OF https://github.com/ros-teleop/teleop_twist_keyboard 
WITH SOME ADD-ONS BUT IMPLEMENTED WITH C++ and ROS2-foxy.
)";

char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}


int main(int argc, char** argv){

  // node init
  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("teleop");
  // define publisher
  auto _pub = node->create_publisher<std_msgs::msg::Char>("/keyboard_control_topic", 10);
  std_msgs::msg::Char msg;


  while(rclcpp::ok()){
    // get the pressed key
    key = getch();
    msg.data=key;
    printf("\n\t\t Key Pressed:[%c] \n",key);
    _pub->publish(msg);
    rclcpp::spin_some(node);  

  }
  return 0;

}