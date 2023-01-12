#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <roblab/JoyJoy.h>
#include <vector>


roblab::JoyJoy message;
std::string id;
ros::Publisher pub;
ros::Subscriber sub;
std::vector<int> last_buttons;


inline void joy_Callback( const sensor_msgs::Joy::ConstPtr &joy_msg );

int main( int argc, char **argv ){
    ros::init( argc, argv, "joyjoy" );
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("JoyName", id);

    sub = nh.subscribe( id, 1, joy_Callback );
    pub = nh.advertise<roblab::JoyJoy>("JoyJoy/"+id, 1000);

    ros::spin();
    return 0;
}


inline void joy_Callback( const sensor_msgs::Joy::ConstPtr &joy_msg ){
    
    message.header = joy_msg->header;
    message.axes = joy_msg->axes;
    message.buttons = joy_msg->buttons;
    for (int j = 0;j<last_buttons.size();j++){
        if(last_buttons[j]==0 && message.buttons[j]==1){
            message.rise[j] = 1;
        }
        else{
            message.rise[j] = 0;
        }
        if(last_buttons[j]==1 && message.buttons[j]==0){
            message.fall[j] = 1;
        }
        else{
            message.fall[j] = 0;
        }
        if(message.rise[j]==1){
            message.toggle[j] = !message.toggle[j];
        }
    }
    
    pub.publish(message);
    last_buttons = message.buttons;
}
