/**********************************************************************
File    trapezoid_control.h
Ver     1.1.6
Date    1.0.0   2019/3/10
1.1.0   2019/05/14      // pid制御のライブラリをの機能を追加しました．.controlでモータの出力を返してくれます(TakahiroYamazaki)
1.1.6   2019/05/20     // 継承を使って書き換えました．使うときには前のバージョンとあまり違いはありません．ただ，timerを引数に取らないようにするためtimerの関数を追加したのでmain文にはstartTimerのメンバ関数を書いておいてください(TakahiroYamazaki)
1.2.0   2019/11/23     // 2段階加速を追加しました．あと，v_maxの符号がターゲットと違っていても揃えるようにしました．(TakahiroYamazaki)
Auther  YamazakiTakahiro
Auther  YamazakiTakahiro
note    台形制御をした
**********************************************************************/

/**********************************************************************
Include guard
**********************************************************************/
#ifndef __TRAPEZOID_CONTROL_H__
#define __TRAPEZOID_CONTROL_H__
// #ifndef M_PI
// #define M_PI           3.14159265358979323846
// #endif
/**********************************************************************
Includes
**********************************************************************/
#include "pid.h"
#include "dob.h"
/**********************************************************************
Differential class
**********************************************************************/
class trapezoid_control : public Dob{
public:

  trapezoid_control(){}
  trapezoid_control(float target, float v_max,float acceleration_time,float deceleration_time);

  //タイマーを始める関数
  void startTimer();

  // 各変数を設定，変更する関数
  void set(float target, float v_max,float acceleration_time,float deceleration_time);
  void setCtrlPeriodSec( float ctrl_period_sec );
  void setTarget(float target);
  void setV_max(float v_max);
  void setEndStatus( float end_err, float end_diff );     // 制御完了判定を変更する関数，変更前はコンストラクタで初期化されている

  // 時間を取得する関数
  float getFinishTime();
  float getTimer();
  float getFilterValue();//control関数を起動させないと使えない
  unsigned char checkEnd();           // 制御完了判定を確認する関数

  void reset();
  void resetforVelocity();
  float filter();
  float filterAcc2( float acceleration_time_2, float acceleration_time, float v_max_2 );
  float filterS();
  float control( float input );
  float controlDob( float input );
  float controlCorrect( float input, float correction_value );
  float controlDobCorrect( float input, float correction_value );
  float controlAcc2( float input,float acceleration_time_2,float acceleration_time,float v_max_2 );
  float controlDobAcc2( float input,float acceleration_time_2,float acceleration_time,float v_max_2 );
  float controlS( float input );
  float controlDobS( float input );
  float controlSCorrect( float input, float correction_value );
  float controlDobSCorrect( float input, float correction_value );

private:
  // 制御周期[s]
  static const float kCTRL_PERIOD_SEC_;
  // checkEnd関数で返す値
  const bool kEND_CONTROL_ = true;
  const bool kCONTINUE_CONTROL_ = false;
  const float M_PI = 3.14159265358979323846;

  float ctrl_period_sec_; // 制御周期[s]

  float timer_;
  float target_;// 台形制御の目標値
  float targetpid_;// pidの.controlに入れるための目標値
  float acceleration_time_2_;
  float v_max_2_;
  float v_max_;// 台形の上辺になる速さの最大値
  float acceleration_time_;// 加速時間[s]
  float stationary_time_;// 速度が一定な時間[s]
  float deceleration_time_;// 減速時間[s]

  float tarapizoid_output_;// 台形な速度の目標値
  float motor_output_;// モータの出力値


};

const float trapezoid_control::kCTRL_PERIOD_SEC_ = 0.005f;

#endif
