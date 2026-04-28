#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <SDL2/SDL.h>

int SDL_ReadJoystick(Sint16 *throttle_value, Sint16 *steering_value, bool *stop){
     // Initialize the joystick subsystem
    
    SDL_Init(SDL_INIT_JOYSTICK);
     // If there are no joysticks connected, quit the program
     if (SDL_NumJoysticks() <= 0) {
         printf("There are no joysticks connected. Quitting now...\n");
         SDL_Quit();
         return -1;
     }
 
     // Open the joystick for reading and store its handle in the joy variable
     SDL_Joystick *joy = SDL_JoystickOpen(0);
 
     // If the joy variable is NULL, there was an error opening it.
     if (joy != NULL) {
         // Get information about the joystick
         const char *name = SDL_JoystickName(joy);
         const int num_axes = SDL_JoystickNumAxes(joy);
         const int num_buttons = SDL_JoystickNumButtons(joy);
         const int num_hats = SDL_JoystickNumHats(joy);
 
         printf("Now reading from joystick '%s' with:\n"
                "%d axes\n"
                "%d buttons\n"
                "%d hats\n\n",
                name,
                num_axes,
                num_buttons,
                num_hats);
 
         int quit = 0;
 
         // Keep reading the state of the joystick in a loop
         while (quit == 0) {
             if (SDL_QuitRequested()) {
                 quit = 1;
             }
 
             for (int i = 0; i < num_axes; i++) {
                 printf("Axis %d: %d\n", i, SDL_JoystickGetAxis(joy, i)); 
             }
 
             for (int i = 0; i < num_buttons; i++) {
                 printf("Button %d: %d\n", i, SDL_JoystickGetButton(joy, i));
             }
 
             for (int i = 0; i < num_hats; i++) {
                 printf("Hat %d: %d\n", i, SDL_JoystickGetHat(joy, i));
             }
 
             printf("\n");
             
             *throttle_value = SDL_JoystickGetAxis(joy, 1);
             *steering_value = SDL_JoystickGetAxis(joy, 3);
             if(SDL_JoystickGetButton(joy, 4)==0 || SDL_JoystickGetButton(joy, 5) == 0){
                *stop=true;
             }else{
                *stop=false;
             }
             SDL_Delay(50);
              
         }
 
         SDL_JoystickClose(joy);
     } else {
         printf("Couldn't open the joystick. Quitting now...\n");
     }
 
     SDL_Quit();
     return 0;

}
int main(int argc, char** argv){
    
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("joystick_reading");
    // define publisher
    auto pub_throttle = node->create_publisher<std_msgs::msg::Float32>("/joystick_throttle_topic", 10);
    auto pub_steering = node->create_publisher<std_msgs::msg::Float32>("/joystick_steering_topic", 10);
    auto pub_stop = node->create_publisher<std_msgs::msg::Bool>("/joystick_stop_topic", 10);
    auto pub_enable = node->create_publisher<std_msgs::msg::Bool>("/enable_acc_topic", 10);
    std_msgs::msg::Float32 msg_throttle;
    std_msgs::msg::Float32 msg_steering;
    std_msgs::msg::Bool msg_stop;
    std_msgs::msg::Bool msg_enable;
    Sint16 throttle_value=0;
    Sint16 steering_value=0;
    bool stop=false;
    bool enable=false;
    SDL_Init(SDL_INIT_JOYSTICK);
     // If there are no joysticks connected, quit the program
     if (SDL_NumJoysticks() <= 0) {
         printf("There are no joysticks connected. Quitting now...\n");
         SDL_Quit();
         return -1;
     }
 
     // Open the joystick for reading and store its handle in the joy variable
     SDL_Joystick *joy = SDL_JoystickOpen(0);
 
     // If the joy variable is NULL, there was an error opening it.
     if (joy != NULL) {
         // Get information about the joystick
         const char *name = SDL_JoystickName(joy);
         const int num_axes = SDL_JoystickNumAxes(joy);
         const int num_buttons = SDL_JoystickNumButtons(joy);
         const int num_hats = SDL_JoystickNumHats(joy);
 
         printf("Now reading from joystick '%s' with:\n"
                "%d axes\n"
                "%d buttons\n"
                "%d hats\n\n",
                name,
                num_axes,
                num_buttons,
                num_hats);
 
         int quit = 0;
 
         // Keep reading the state of the joystick in a loop
         while (quit == 0) {
             if (SDL_QuitRequested()) {
                 quit = 1;
             }
 
             for (int i = 0; i < num_axes; i++) {
                 printf("Axis %d: %d\n", i, SDL_JoystickGetAxis(joy, i)); 
             }
 
             for (int i = 0; i < num_buttons; i++) {
                 printf("Button %d: %d\n", i, SDL_JoystickGetButton(joy, i));
             }
 
             for (int i = 0; i < num_hats; i++) {
                 printf("Hat %d: %d\n", i, SDL_JoystickGetHat(joy, i));
             }
 
             printf("\n");
             
             throttle_value = SDL_JoystickGetAxis(joy, 1);
             steering_value = SDL_JoystickGetAxis(joy, 3);
             msg_throttle.data=throttle_value;
             msg_steering.data=steering_value;
             if(SDL_JoystickGetButton(joy, 4)==0 && SDL_JoystickGetButton(joy, 5) == 0){
                stop=false;
             }else{
                stop=true;
             }
            /* if(SDL_JoystickGetButton(joy, 0)==1){
                enable=true;
             }
             if(SDL_JoystickGetButton(joy, 2)==1){
                enable=false;
             }*/
             if(SDL_JoystickGetButton(joy, 0)==1){
                enable = true;
                msg_enable.data=enable;
                pub_enable->publish(msg_enable);
             }
             if(SDL_JoystickGetButton(joy, 2)==1){
                enable=false;
                msg_enable.data=enable;
                pub_enable->publish(msg_enable);
             }
             msg_stop.data=stop;
             pub_throttle->publish(msg_throttle);
             SDL_Delay(10);
             pub_steering->publish(msg_steering);
             pub_stop->publish(msg_stop);
             rclcpp::spin_some(node); 
             SDL_Delay(50);
              
         }
 
         SDL_JoystickClose(joy);
     } else {
         printf("Couldn't open the joystick. Quitting now...\n");
     }
 
     SDL_Quit();
     return 0;

    
    /*while(rclcpp::ok()){

        SDL_ReadJoystick(*throttle_value, *steering_value, *stop);
        pub_throttle->publish(msg_throttle);
        rclcpp::spin_some(node); 
  
    }*/
}