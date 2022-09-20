#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <roblab/JoyJoy.h>
#include <vector>


roblab::JoyJoy Message;
sensor_msgs::Joy message;
std::vector<std::string> ids;
std::vector<ros::Publisher> Pubs;
std::vector<ros::Publisher> pubs;
std::vector<std::vector<int>> last_buttons;
int length = 1;
float cycle = 0;


inline void joys_Callback( const sensor_msgs::Joy::ConstPtr &joy_msg );

int main( int argc, char **argv ){
    ros::init( argc, argv, "joyjoy" );
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("ControlCycle",cycle);
    
    pnh.getParam("NumberOfJoy",length);
    for (int i=0; i<length; i++){
        std::string a = "joy" + std::to_string(i);
        std::string b;
        pnh.getParam(a, b);
        ids.emplace_back(b);
    }

    ros::Subscriber sub_ps4 = nh.subscribe( "joy", 1, joys_Callback );
    for (int i=0; i<length; i++){
        std::string a = "joy" + std::to_string(i);
        if(cycle>0){
            ros::Publisher pub_joy = nh.advertise<roblab::JoyJoy>(a, 1000);
            Pubs.emplace_back(pub_joy);
        }
        else{
            ros::Publisher pub_joy = nh.advertise<sensor_msgs::Joy>(a, 1000);
            pubs.emplace_back(pub_joy);
        }
    }

    for (int i=0; i<length; i++){
        std::vector<int> b;
        last_buttons.emplace_back(b);
    }

    if(cycle>0){
        ros::Rate loop_rate(1/cycle);
        while (ros::ok()){
            if (Message.rise.empty()){
                Message.rise = Message.buttons;
                Message.fall = Message.buttons;
                Message.toggle = Message.buttons;
                for(auto s : last_buttons){
                    s = Message.buttons;
                }
            }
            else{
                for (int i=0; i<length; i++){
                    if(ids[i]==Message.header.frame_id){
                        for (int j = 0;j<last_buttons[i].size();j++){
                            if(last_buttons[i][j]==0 && Message.buttons[j]==1){
                                Message.rise[j] = 1;
                            }
                            else{
                                Message.rise[j] = 0;
                            }
                            if(last_buttons[i][j]==1 && Message.buttons[j]==0){
                                Message.fall[j] = 1;
                            }
                            else{
                                Message.fall[j] = 0;
                            }
                            if(Message.rise[j]==1){
                                Message.toggle[j] = !Message.toggle[j];
                            }
                        }
                        last_buttons[i] = Message.buttons;
                        Pubs[i].publish(Message);
                    }
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    ros::spin();
    return 0;
}


inline void joys_Callback( const sensor_msgs::Joy::ConstPtr &joy_msg ){
    if(cycle>0){
        Message.header = joy_msg->header;
        Message.axes = joy_msg->axes;
        Message.buttons = joy_msg->buttons;
    }
    else{
        message.header = joy_msg->header;
        message.axes = joy_msg->axes;
        message.buttons = joy_msg->buttons;
        for (int i=0; i<length; i++){
            if(ids[i]==message.header.frame_id){
                pubs[i].publish(message);
            }
        }
    }
}