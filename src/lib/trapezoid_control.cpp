/**********************************************************************
File    trapezoid_control.cpp
Ver     1.1.6
Date    1.0.0   2019/3/10
1.1.0   2019/05/14      // pid制御のライブラリをの機能を追加しました．.controlでモータの出力を返してくれます(TakahiroYamazaki)
1.1.6   2019/05/20     // 継承を使って書き換えました．使うときには前のバージョンとあまり違いはありません．ただ，timerを引数に取らないようにするためtimerの関数を追加したのでmain文にはstartTimerのメンバ関数を書いておいてください(TakahiroYamazaki)
1.2.0   2019/11/23     // 2段階加速を追加しました．あと，v_maxの符号がターゲットと違っていても揃えるようにしました．(TakahiroYamazaki)
Auther  YamazakiTakahiro
note    台形制御をした
**********************************************************************/

/**********************************************************************
Includes
**********************************************************************/
#include "roblab/trapezoid_control.h"
#include <cmath>
#include <math.h>

/**********************************************************************
目標値、速さの最大値、加速時間、減速時間を設定し，速さが安定している時間を求める
メンバ変数を初期化，PIDの制御周期のセットをする関数．
引数1 目標値
引数2 速さの最大値
引数3 加速時間
引数4 減速時間
**********************************************************************/
trapezoid_control::trapezoid_control(float target, float v_max,float acceleration_time,float deceleration_time) : target_(target),v_max_(v_max),acceleration_time_(acceleration_time),deceleration_time_(deceleration_time){

  // 各メンバ変数を初期化
  timer_ = 0.0f;
  targetpid_ = 0.0f;
  tarapizoid_output_ = 0.0f;
  motor_output_ = 0.0f;
  stationary_time_ =  (target_-v_max_*(acceleration_time_ + deceleration_time_)/2)/v_max;
  acceleration_time_2_ = 0.0f;
  if (std::signbit( target_ ) ^ std::signbit( v_max_ ))
  v_max_ = -v_max_;
  if(v_max_ == 0.0) v_max_ = 1.0;
  //PIDの制御周期のセット
  ctrl_period_sec_ = kCTRL_PERIOD_SEC_;
  PID::set(ctrl_period_sec_);
}
/**********************************************************************
タイマーを始める関数．
**********************************************************************/
void trapezoid_control::startTimer(){
  timer_ += ctrl_period_sec_;
}
/**********************************************************************
目標値、速さの最大値、加速時間、減速時間を設定し，速さが安定している時間を求める
メンバ変数を初期化，関数．
引数1 目標値
引数2 速さの最大値
引数3 加速時間
引数4 減速時間
**********************************************************************/
void trapezoid_control::set(float target, float v_max,float acceleration_time,float deceleration_time){

  // 各メンバ変数を初期化
  timer_ = 0.0f;
  targetpid_ = 0.0f;
  tarapizoid_output_ = 0.0f;
  motor_output_ = 0.0f;
  acceleration_time_2_ = 0.0f;
  if (target == 0.0f){
    target_ = 0.0f;
    v_max_ = 0.0f;
    acceleration_time_ = 0.0f;
    deceleration_time_ = 0.0f;
    stationary_time_ = 0.0f;
    if(v_max_ == 0.0) v_max_ = 1.0;
    return;
  }

  target_ = target;
  v_max_ = v_max;
  acceleration_time_ = acceleration_time;
  deceleration_time_ = deceleration_time;
  if(v_max_ == 0.0) v_max_ = 1.0;
  if (std::signbit( target_ ) ^ std::signbit( v_max_ ))
  v_max_ = -v_max_;
  stationary_time_ =  (target_-v_max_*(acceleration_time_ + deceleration_time_)/2)/v_max_;
  if (stationary_time_ < 0){
    float diff = stationary_time_ / ( acceleration_time_ + deceleration_time_ );
    acceleration_time_ += diff * acceleration_time_;
    deceleration_time_ += diff * deceleration_time_;
    stationary_time_ = 0.0f;
  }
}

/**********************************************************************
目標値を設定し，速さが安定している時間を求める
メンバ変数を初期化，関数．
引数1 目標値
**********************************************************************/
void trapezoid_control::setTarget(float target){

  target_ = target;
  if (std::signbit( target_ ) ^ std::signbit( v_max_ ))
  v_max_ = -v_max_;

  stationary_time_ =  (target_-v_max_*(acceleration_time_ + deceleration_time_)/2)/v_max_;

}
/**********************************************************************
速さの最大値を設定し，速さが安定している時間を求める
メンバ変数を初期化，関数．
引数1 速さの最大値
**********************************************************************/
void trapezoid_control::setV_max(float v_max){

  v_max_ = v_max;

  if (std::signbit( target_ ) ^ std::signbit( v_max_ ))
  v_max_ = -v_max_;
  stationary_time_ =  (target_-v_max_*(acceleration_time_ + deceleration_time_)/2)/v_max_;

}
/**********************************************************************
制御周期を設定．未設定なら0.02[s]
引数1 制御周期[s]
**********************************************************************/
void trapezoid_control::setCtrlPeriodSec( float ctrl_period_sec ){
  ctrl_period_sec_ = ctrl_period_sec;
}
/**********************************************************************
タイマーの値を返す関数．
**********************************************************************/
float trapezoid_control::getTimer(){
  return timer_;
}
/**********************************************************************
終了時間を返す関数．
**********************************************************************/
float trapezoid_control::getFinishTime(){
  // if (acceleration_time_2_ > 0.0) return (acceleration_time_2_ + acceleration_time_ + stationary_time_ + deceleration_time_);
  // else
  return (acceleration_time_2_ + acceleration_time_ + stationary_time_ + deceleration_time_);
}
/**********************************************************************
台形制御のフィルタを返す関数．注意：control関数を起動させないと使えない
**********************************************************************/
float trapezoid_control::getFilterValue(){
  return (targetpid_);
}
/**********************************************************************
台形制御完了判定を確認する関数．
**********************************************************************/
unsigned char trapezoid_control::checkEnd(){
  if ( timer_ >=  trapezoid_control::getFinishTime()){
    return kEND_CONTROL_;
  }else{
    return kCONTINUE_CONTROL_;
  }
}
/**********************************************************************
台形制御をする関数．
引数1 台形制御を始めてから経過している時間
返り値 台形を描いている目標値
**********************************************************************/
float trapezoid_control::filter(){
  if (target_ == 0.0f){
    tarapizoid_output_ = 0.0f;
    return tarapizoid_output_;
  }

  if( timer_ <= acceleration_time_){
    tarapizoid_output_ = (v_max_/acceleration_time_)*timer_;
  }else if( timer_ <=(acceleration_time_ + stationary_time_) ){
    tarapizoid_output_ = v_max_;
  }else if( timer_ <= (acceleration_time_ + stationary_time_ + deceleration_time_ )){
    tarapizoid_output_ = v_max_ - (v_max_/deceleration_time_)*(timer_-acceleration_time_- stationary_time_);
  }else{
    tarapizoid_output_ = 0.0f;
  }

  return tarapizoid_output_;
}
/**********************************************************************
台形制御をする関数．
引数1 台形制御を始めてから経過している時間
返り値 台形を描いている目標値
**********************************************************************/
float trapezoid_control::filterAcc2( float acceleration_time_2,float acceleration_time,float v_max_2 ){
  if (target_ == 0.0f){
    tarapizoid_output_ = 0.0f;
    return tarapizoid_output_;
  }

  acceleration_time_2_ = acceleration_time_2;
  acceleration_time_ = acceleration_time;
  v_max_2_ = v_max_2;
  stationary_time_ =  (target_-(v_max_*(acceleration_time_ + deceleration_time_)+v_max_2_*acceleration_time_2_)/2)/v_max_;

  if( timer_ <= acceleration_time_2_){
    tarapizoid_output_ = (v_max_2_/acceleration_time_2_)*timer_;
  }else if( timer_ < acceleration_time_2_ + acceleration_time_){
    tarapizoid_output_ = (v_max_-v_max_2_)/acceleration_time_*(timer_  - acceleration_time_2_)+ v_max_2_;
  }else if( timer_ <=(acceleration_time_2_ + acceleration_time_ + stationary_time_) ){
    tarapizoid_output_ = v_max_;
  }else if( timer_ <= (acceleration_time_2_ + acceleration_time_ + stationary_time_ + deceleration_time_ )){
    tarapizoid_output_ = v_max_ - (v_max_/deceleration_time_)*(timer_ - acceleration_time_2_ - acceleration_time_- stationary_time_);
  }else{
    tarapizoid_output_ = 0.0f;
  }

  return tarapizoid_output_;
}
/**********************************************************************
S字制御をする関数．
引数1 S字制御を始めてから経過している時間
返り値 S字を描いている目標値
**********************************************************************/
float trapezoid_control::filterS(){
  if (target_ == 0.0f){
    tarapizoid_output_ = 0.0f;
    return tarapizoid_output_;
  }

  if( timer_ <= acceleration_time_){
    tarapizoid_output_ = v_max_/2.0f * (1.0f - cos(M_PI / acceleration_time_ * timer_));
  }else if( timer_ <=(acceleration_time_ + stationary_time_) ){
    tarapizoid_output_ = v_max_;
  }else if( timer_ <= (acceleration_time_ + stationary_time_ + deceleration_time_ )){
    tarapizoid_output_ = v_max_/2.0f * (1.0f + cos(M_PI / deceleration_time_ * ( timer_ - (acceleration_time_ + stationary_time_))));
  }else{
    tarapizoid_output_ = 0.0f;
  }

  return tarapizoid_output_;
}
/**********************************************************************
設定した目標値に対してPID制御を行う関数．
引数 フィードバック値
返り値 PID制御器の出力
**********************************************************************/
float trapezoid_control::control( float input ){
  startTimer();
  targetpid_ = trapezoid_control::filter( );
  PID::setTarget( targetpid_ );
  motor_output_ = PID::control(input);
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御+外乱オブザーバーを行う関数．
引数 フィードバック値
返り値 PID制御+外乱オブザーバーの出力
**********************************************************************/
float trapezoid_control::controlDob( float input ){
  startTimer();
  targetpid_ = trapezoid_control::filter();
  Dob::setTarget( targetpid_ );
  motor_output_ = Dob::control( input );
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御+補正を行う関数．
引数1 フィードバック値
引数2 補正値
返り値 PID制御の出力
**********************************************************************/
float trapezoid_control::controlCorrect( float input, float correction_value ){
  startTimer();
  targetpid_ = trapezoid_control::filter() + correction_value;
  PID::setTarget( targetpid_ );
  motor_output_ = PID::control(input);
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御+外乱オブザーバー+補正を行う関数．
引数1 フィードバック値
引数2 補正値
返り値 PID制御+外乱オブザーバーの出力
**********************************************************************/
float trapezoid_control::controlDobCorrect( float input, float correction_value ){
  startTimer();
  targetpid_ = trapezoid_control::filter() + correction_value;
  Dob::setTarget( targetpid_ );
  motor_output_ = Dob::control( input );
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御を行う関数．2段階加速型の台形制御
引数 フィードバック値
返り値 PID制御器の出力
**********************************************************************/
float trapezoid_control::controlAcc2( float input,float acceleration_time_2,float acceleration_time,float v_max_2 ){
  startTimer();
  targetpid_ = trapezoid_control::filterAcc2(acceleration_time_2, acceleration_time, v_max_2);
  PID::setTarget( targetpid_ );
  motor_output_ = PID::control(input);
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御+外乱オブザーバーを行う関数．2段階加速型の台形制御
引数 フィードバック値
返り値 PID制御+外乱オブザーバーの出力
**********************************************************************/
float trapezoid_control::controlDobAcc2( float input,float acceleration_time_2,float acceleration_time,float v_max_2 ){
  startTimer();
  targetpid_ = trapezoid_control::filterAcc2(acceleration_time_2, acceleration_time, v_max_2);
  Dob::setTarget( targetpid_ );
  motor_output_ = Dob::control( input );
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御を行う関数．
正し目標値は台形ではなく，s字を描いている
引数 フィードバック値
返り値 PID制御器の出力
**********************************************************************/
float trapezoid_control::controlS( float input ){
  startTimer();
  targetpid_ = trapezoid_control::filterS();
  PID::setTarget( targetpid_ );
  motor_output_ = PID::control(input);
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御+外乱オブザーバーを行う関数．
正し目標値は台形ではなく，s字を描いている
引数 フィードバック値
返り値 PID制御+外乱オブザーバーの出力
**********************************************************************/
float trapezoid_control::controlDobS( float input ){
  startTimer();
  targetpid_ = trapezoid_control::filterS();
  Dob::setTarget( targetpid_ );
  motor_output_ = Dob::control( input );
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御+補正を行う関数．
正し目標値は台形ではなく，s字を描いている
引数1 フィードバック値
引数2 補正値
返り値 PID制御の出力
**********************************************************************/
float trapezoid_control::controlSCorrect( float input, float correction_value ){
  startTimer();
  targetpid_ = trapezoid_control::filterS() + correction_value;
  PID::setTarget( targetpid_ );
  motor_output_ = PID::control(input);
  return motor_output_;
}
/**********************************************************************
設定した目標値に対してPID制御+外乱オブザーバー+補正を行う関数．
正し目標値は台形ではなく，s字を描いている
引数1 フィードバック値
引数2 補正値
返り値 PID制御+外乱オブザーバーの出力
**********************************************************************/
float trapezoid_control::controlDobSCorrect( float input, float correction_value ){
  startTimer();
  targetpid_ = trapezoid_control::filterS() + correction_value;
  Dob::setTarget( targetpid_ );
  motor_output_ = Dob::control( input );
  return motor_output_;
}
/**********************************************************************
メンバ変数を初期化する関数．
**********************************************************************/
void trapezoid_control::reset(){
  // 各メンバ変数を初期化
  timer_ = 0.0f;
  target_ = 0.0f;
  tarapizoid_output_ = 0.0f;
  targetpid_ = 0.0f;
  motor_output_ = 0.0f;
  v_max_ = 0.0f;
  acceleration_time_ = 0.0f;
  deceleration_time_ = 0.0f;
  stationary_time_ = 0.0f;
  if( stationary_time_ <= 0.0 ){
    stationary_time_  = 0.0;
  }
  PID::reset();
  Dob::reset();
}

/**********************************************************************
Usage example
**********************************************************************/
