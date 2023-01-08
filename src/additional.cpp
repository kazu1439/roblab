#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <roblab/JoyJoy.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <vector>


std::vector<std::string> topics;
std::vector<std::string> values;
std::vector<ros::Subscriber> subs;
float cycle = 0.02;

void commonIntCb(const std_msgs::Int32::ConstPtr& msg, const std::string name,const int index)
{
    // ROS_INFO("%s : %d", name.c_str(), msg->data);
    values[index] = "";
    values[index] += name;
    values[index] += " : ";
    values[index] += std::to_string(msg->data);
}

void commonIntMultiCb(const std_msgs::Int32MultiArray::ConstPtr& msg, const std::string name,const int index)
{
    values[index] = "";
    values[index] += name;
    values[index] += " : ";
    values[index] += "[";
    for(int i = 0;i<msg->data.size();i++){
        // ROS_INFO("%s%d : %d", name.c_str(), i,msg->data[i]);
        values[index] += std::to_string(msg->data[i]);
        if(i==msg->data.size()-1){
            values[index] += "]";
            break;
        }
        else{
            values[index] += ",";
        }
    }
}

void commonFloatCb(const std_msgs::Float32::ConstPtr& msg, const std::string name,const int index)
{
    // ROS_INFO("%s : %f", name.c_str(), msg->data);
    values[index] = "";
    values[index] += name;
    values[index] += " : ";
    values[index] += std::to_string(msg->data);
}

void commonFloatMultiCb(const std_msgs::Float32MultiArray::ConstPtr& msg, const std::string name,const int index)
{
    values[index] = "";
    values[index] += name;
    values[index] += " : ";
    values[index] += "[";
    for(int i = 0;i<msg->data.size();i++){
        // ROS_INFO("%s%f : %f", name.c_str(), i,msg->data[i]);
        values[index] += std::to_string(msg->data[i]);
        if(i==msg->data.size()-1){
            values[index] += "]";
            break;
        }
        else{
            values[index] += ",";
        }
    }
}

void commonBoolCb(const std_msgs::Bool::ConstPtr& msg, const std::string name,const int index)
{
    ROS_INFO("%s : %d", name.c_str(), msg->data);
    values[index] = "";
    values[index] += name;
    values[index] += " : ";
    values[index] += std::to_string(msg->data);
}

int main( int argc, char **argv ){
    ros::init( argc, argv, "additional" );
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("ControlCycle",cycle);

    ros::Publisher pub_status = nh.advertise<std_msgs::String>("topic_status", 1);
    

    if(cycle>0){
        ros::Rate loop_rate(1/cycle);
        while (ros::ok()){
            ros::master::V_TopicInfo master_topics;
            ros::master::getTopics(master_topics);

            std_msgs::String msg;
            msg.data = "";
            for(int i = 0;i<values.size();i++){
                // ROS_INFO((topics[i]+':'+values[i]).c_str());
                msg.data += values[i];
                std::cout << values[i] << std::endl;
                if(i==values.size()-1){
                    break;
                }
                else{
                    msg.data += '\n';
                }
            }
            pub_status.publish(msg);

            // ROS_INFO("")
            for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
                const ros::master::TopicInfo& info = *it;
                // std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << ": " << info.datatype << std::endl;
                bool found = std::find(topics.begin(), topics.end(), info.name) != topics.end();
                if(!found){
                    if(info.name=="/topic_status"){
                        continue;
                    }
                    
                    topics.emplace_back(info.name);
                    values.emplace_back("");
                    if(info.datatype=="std_msgs/Int32"){
                        subs.emplace_back(nh.subscribe<std_msgs::Int32>(info.name,1,boost::bind(commonIntCb, _1, info.name,topics.size()-1)));
                    }
                    else if(info.datatype=="std_msgs/Int32MultiArray"){
                        subs.emplace_back(nh.subscribe<std_msgs::Int32MultiArray>(info.name,1,boost::bind(commonIntMultiCb, _1, info.name,topics.size()-1)));
                    }
                    else if(info.datatype=="std_msgs/Float32"){
                        subs.emplace_back(nh.subscribe<std_msgs::Float32>(info.name,1,boost::bind(commonFloatCb, _1, info.name,topics.size()-1)));
                    }
                    else if(info.datatype=="std_msgs/Float32MultiArray"){
                        subs.emplace_back(nh.subscribe<std_msgs::Float32MultiArray>(info.name,1,boost::bind(commonFloatMultiCb, _1, info.name,topics.size()-1)));
                    }
                    else if(info.datatype=="std_msgs/Bool"){
                        subs.emplace_back(nh.subscribe<std_msgs::Bool>(info.name,1,boost::bind(commonBoolCb, _1, info.name,topics.size()-1)));
                    }
                    else{
                        values[topics.size()-1] = info.name+" : undefined";
                        ROS_WARN("%s[%s] is not defined.", info.datatype.c_str(), info.name.c_str());
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