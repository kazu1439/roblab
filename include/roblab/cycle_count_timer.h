#pragma once
#include <ros/ros.h>

// サイクルの実行回数を数えて経過時間を割り出すタイマー 毎周期Update()の実行が必要
class CycleCountTimer
{
public:
    // cycle_time_s: 周期（秒）
    CycleCountTimer(double cycle_time_s);
    // count_time_ms: 数える時間（ミリ秒)
    void StartCounting(int count_time_ms);
    // 毎周期必ず１回実行する
    void Update();
    bool CheckEnd();

private:
    double cycle_time_s_;
    int now_count_;
    int end_count_;
    bool is_end_;
};
