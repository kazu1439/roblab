#include "roblab/cycle_count_timer.h"

CycleCountTimer::CycleCountTimer(double cycle_time_s) : cycle_time_s_{cycle_time_s}
{
    is_end_ = true;
}

void CycleCountTimer::StartCounting(int count_time_ms)
{
    Update();
    end_count_ = (static_cast<double>(count_time_ms) / 1000.0 / cycle_time_s_);
    is_end_ = false;
    now_count_ = 0;

    auto now_time = ros::Time::now();
    // ROS_INFO("timer is set: %dms", count_time_ms);
    // ROS_INFO("timer startTime:\t%d.%d", now_time.sec, now_time.nsec);
}

void CycleCountTimer::Update()
{
    now_count_++;
    CheckEnd();
}

bool CycleCountTimer::CheckEnd()
{
    if (is_end_)
        return true;

    is_end_ = now_count_ > end_count_;

    if (is_end_) // 終了（初回)
    {
        auto now_time = ros::Time::now();
        // ROS_INFO("timer endTime:\t\t%d.%d", now_time.sec, now_time.nsec);
    }
    return is_end_;
}