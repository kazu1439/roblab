#pragma once
#include <ros/ros.h>

// ros::Timeを使ったタイマー
class RosClockTimer
{
public:
    RosClockTimer();
    // count_time_ms: 数える時間（ミリ秒)
    void StartCounting(int count_time_ms);
    bool CheckEnd();

private:
    ros::Time now_;
    ros::Time end_time_;
    bool is_end_;
};