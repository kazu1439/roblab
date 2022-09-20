#include "roblab/timer.h"




Timer::Timer(){
};

unsigned char Timer::Start(float wait_time){
    if(first_flag_){
        wait_time_ = wait_time;
        start_ = std::chrono::system_clock::now();
        first_flag_ = false;
        return 1;
    }
    else{
        return 0;
    }
};

unsigned char Timer::CheckEnd(){
    end_ = std::chrono::system_clock::now();
    elapsed_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_-start_).count() / 1000;
    if(elapsed_>=wait_time_ && !first_flag_){
        first_flag_ = true;
        return 1;
    }
    else{
        return 0;
    }
};

void Timer::Reset(){
    first_flag_ = true;
};
