/**********************************************************************
File    dob.h
Ver     1.1.0
Date    1.0.0   2019/3/13
1.1.0   2019/6/17  //多重継承を使って書き換えました．
pidとlpfのライブラリを組み込むことで，pidのライブラリを使うように使えます(TakahiroYamazaki)
Auther  Yamazaki Takahiro
note    外乱オブザーバーのライブラリ
**********************************************************************/

/**********************************************************************
Include guard
**********************************************************************/
#ifndef __DOB_H__
#define __DOB_H__

/**********************************************************************
Includes
**********************************************************************/
#include "pid.h"
#include "lpf.h"
/**********************************************************************
Differential class
**********************************************************************/
class Dob: public PID,public LPF{
public:

  Dob(){}
  Dob(float sys_gain, float sys_t_const_sec,float ctrl_period_sec,float lpf_t_const_sec );//Kn:システムのゲインTn:システムの時定数Ts:制御周期Tf:ローパスフィルターの時定数

  void set(float sys_gain, float sys_t_const_sec,float ctrl_period_sec,float lpf_t_const_sec );
  float getCoeff_a_();
  float getCoeff_b_();
  float getCoeff_c_();
  float getPidOutput_();
  float getLpfOutput_();
  float getDobOutput_();
  float getEstimatedDisturbance_();
  void setLimitOutput( float limit_output );
  void reset();
  float filter(float velocity);
  float control(float velocity);

private:

  float sys_gain_;
  float sys_t_const_sec_;
  float ctrl_period_sec_;
  float lpf_t_const_sec_;
  float estimated_disturbance_ ;

  float coeff_a_;           // フィルタをかけるときに使用する係数a
  float coeff_b_;           // フィルタをかけるときに使用する係数b
  float coeff_c_;           // フィルタをかけるときに使用する係数c

  float velocity_prev_;
  float filter_output_prev_;
  float filter_output_;
  float pid_output_;// pidの出力値
  float lpf_output_;// lpfの出力値
  float dob_output_;// dobの出力値
  float motor_output_;// モータの出力値
  float limit_output_;// 出力値の上限
};

#endif
