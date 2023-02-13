#include "roblab/ros_clock_timer.h"

RosClockTimer::RosClockTimer()
{
    is_end_ = true;
}

void RosClockTimer::StartCounting(int count_time_ms)
{
    now_ = ros::Time::now();
    ros::Duration duration(static_cast<double>(count_time_ms) / 1000.0);
    end_time_ = now_ + duration;
    is_end_ = false;

    // ROS_INFO("timer startTime:\t%d.%d", now_.sec, now_.nsec);
}

bool RosClockTimer::CheckEnd()
{
    if (is_end_)
        return true;

    now_ = ros::Time::now();
    is_end_ = end_time_ < now_;

    if (is_end_) // 終了（初回)
    {
        // ROS_INFO("timer endTime:\t\t%d.%d", now_.sec, now_.nsec);
    }
    return is_end_;
}
