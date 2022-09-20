/**********************************************************************
File    dob.cpp
Ver     1.1.0
Date    1.0.0   2019/3/13
1.1.0   2019/6/17  //多重継承を使って書き換えました．
pidとlpfのライブラリを組み込むことで，pidのライブラリを使うように使えます(TakahiroYamazaki)
Auther  Yamazaki Takahiro
note    外乱オブザーバーのライブラリ
**********************************************************************/

/**********************************************************************
Includes
**********************************************************************/
#include "roblab/dob.h"
#include <math.h>
/**********************************************************************
システムのゲインと時定数、制御周期、ローパスフィルターの時定数を設定し、係数を求める
メンバ変数を初期化，関数．
引数1 システムのゲイン
引数2 システムの時定数
引数3 制御周期
引数4 ローパスフィルターの時定数
**********************************************************************/
Dob::Dob(float sys_gain, float sys_t_const_sec,float ctrl_period_sec,float lpf_t_const_sec ) : sys_gain_(sys_gain),sys_t_const_sec_(sys_t_const_sec),ctrl_period_sec_(ctrl_period_sec),lpf_t_const_sec_(lpf_t_const_sec){

  coeff_a_ = lpf_t_const_sec_/(lpf_t_const_sec_ + ctrl_period_sec_);
  coeff_b_ = ( sys_t_const_sec_ + ctrl_period_sec_ )/(sys_gain_*(lpf_t_const_sec_ + ctrl_period_sec_));
  coeff_c_= sys_t_const_sec_/(sys_gain_*(lpf_t_const_sec_ + ctrl_period_sec_));

  // 出力値の初期化
  velocity_prev_ = 0.0;
  filter_output_prev_ = 0.0;
  filter_output_ = 0.0;
  pid_output_ = 0.0;
  lpf_output_ = 0.0;
  dob_output_ = 0.0;
  motor_output_ = 0.0;
  limit_output_ = 0x7f7fffff;

  PID::set(ctrl_period_sec_);
  LPF::set(lpf_t_const_sec_,ctrl_period_sec_);
}

/**********************************************************************
システムのゲインと時定数、制御周期、ローパスフィルターの時定数を設定し、係数を求める
メンバ変数を初期化，関数．
引数1 システムのゲイン
引数2 システムの時定数
引数3 制御周期
引数4 ローパスフィルターの時定数
**********************************************************************/
void Dob::set(float sys_gain, float sys_t_const_sec,float ctrl_period_sec,float lpf_t_const_sec ){

  sys_gain_ = sys_gain;
  sys_t_const_sec_ = sys_t_const_sec;
  ctrl_period_sec_ = ctrl_period_sec;
  lpf_t_const_sec_ = lpf_t_const_sec;

  coeff_a_ = lpf_t_const_sec_/(lpf_t_const_sec_ + ctrl_period_sec_);
  coeff_b_ = ( sys_t_const_sec_ + ctrl_period_sec_ )/(sys_gain_*(lpf_t_const_sec_ + ctrl_period_sec_));
  coeff_c_= sys_t_const_sec_/(sys_gain_*(lpf_t_const_sec_ + ctrl_period_sec_));

  // 出力値の初期化
  velocity_prev_ = 0.0;
  filter_output_prev_ = 0.0;
  filter_output_ = 0.0;
  pid_output_ = 0.0;
  lpf_output_ = 0.0;
  dob_output_ = 0.0;
  motor_output_ = 0.0;
  limit_output_ = 0x7f7fffff;

  PID::set(ctrl_period_sec_);
  LPF::set(lpf_t_const_sec_,ctrl_period_sec_);
}
/**********************************************************************
係数aを返す関数．
**********************************************************************/
float Dob::getCoeff_a_(){
  return (coeff_a_ );
}
/**********************************************************************
係数bを返す関数．
**********************************************************************/
float Dob::getCoeff_b_(){
  return (coeff_b_ );
}
/**********************************************************************
係数cを返す関数．
**********************************************************************/
float Dob::getCoeff_c_(){
  return (coeff_c_ );
}
/**********************************************************************
PID制御の出力値を返す関数．
**********************************************************************/
float Dob::getPidOutput_(){
  return (pid_output_);
}
/**********************************************************************
lpfの出力値を返す関数．
**********************************************************************/
float Dob::getLpfOutput_(){
  return (lpf_output_);
}
/**********************************************************************
外乱オブザーバーの出力値を返す関数．
**********************************************************************/
float Dob::getDobOutput_(){
  return (dob_output_);
}
/**********************************************************************
推定外乱に対して働くdutty比を返す関数．
**********************************************************************/
float Dob::getEstimatedDisturbance_(){
  estimated_disturbance_ = -dob_output_ + lpf_output_;
  return (estimated_disturbance_);
}
/**********************************************************************
外乱オブザーバーの出力値の上限を変更する関数．
引数1 上限値
**********************************************************************/
void Dob::setLimitOutput( float limit_output ){
  limit_output_ = limit_output;
}
/**********************************************************************
メンバ変数を初期化する関数．
**********************************************************************/
void Dob::reset(){
  // 各メンバ変数を初期化
  sys_gain_ = 0.0f;
  sys_t_const_sec_ = 0.0f;
  ctrl_period_sec_ = 0.0f;
  lpf_t_const_sec_ = 0.0f;
  filter_output_ = 0.0f;
  dob_output_ = 0.0f;
  pid_output_ = 0.0f;
  lpf_output_ = 0.0f;
  motor_output_ = 0.0f;
}

/**********************************************************************
逆モデルの入力を返す関数．
引数1 このフィルタへの入力値
返り値 このフィルタの出力値
**********************************************************************/
float Dob::filter(float velocity){

  filter_output_ = coeff_a_*filter_output_prev_ + coeff_b_*velocity - coeff_c_*velocity_prev_;
  filter_output_prev_ = filter_output_;
  velocity_prev_ = velocity;

  return filter_output_;
}
/**********************************************************************
モーターのdutty比を返してくれる関数
引数1 このフィルタへの入力値(基本的に速度)
返り値 このフィルタの出力値(基本的にモーターのdutty比)
**********************************************************************/
float Dob::control(float velocity){
  dob_output_ = Dob::filter(velocity);
  pid_output_ = PID::control(velocity);
  lpf_output_ = LPF::filter(PID::control(velocity_prev_));
  motor_output_ = -dob_output_ + lpf_output_ + pid_output_;

  if ( fabs( motor_output_ ) >= limit_output_ )
  return limit_output_ * motor_output_ / fabs( motor_output_ );

  return motor_output_;
}
/**********************************************************************
Usage example
**********************************************************************/
