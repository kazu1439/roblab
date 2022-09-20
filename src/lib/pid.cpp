/**********************************************************************
File    pid.cpp
Ver     1.2.2
Date    1.0.0   2017/10/04
1.1.0   2018/11/18      // 微分と積分を精度の高いものに変更
1.1.1   2018/11/20      // P制御，I制御，D制御それぞれの出力を返す関数を作成
1.2.1   2018/12/05      // PID出力,積分値に上限を付けられる変数と関数,Iの出力だけ初期化する関数を追加(Kiryu.H)
1.2.2   2019/05/14      // 制御周期をセットできる関数を追加しました．あとインスタンス生成時引数を渡さなくてもよくしました(TakahiroYamazaki)
Auther  Maegaki Noriyoshi
**********************************************************************/

/**********************************************************************
Includes
**********************************************************************/
#include "roblab/pid.h"
#include "math.h"
#include <iostream>
/**********************************************************************
PID型の変数を定義し，各変数を初期化するコンストラクタ．
引数1 制御周期
**********************************************************************/
PID::PID( float ctrl_period_sec ){

  ctrl_period_sec_ = ctrl_period_sec;

  // ゲインの初期化
  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;

  // 偏差を初期化
  err_ = 0.0;
  err_diff_ = 0.0;
  err_intg_ = 0.0;

  // 出力値の初期化
  p_output_ = 0.0;
  i_output_ = 0.0;
  d_output_ = 0.0;
  output_ = 0.0;
  limit_output_ = 0x7f7fffff;
  limit_i_output_ = 0x7f7fffff;

  input_ = 0.0;

  // 終了判定の初期化
  end_cnt_ = kEND_CNT_;
  end_err_ = kEND_ERROR_;
  err_diff_ = kEND_DIFFERENTIAL_;

  // 数値微分，数値積分の設定
  diff_time_const_ = kDIFF_TIME_CONST_sec_;
  diff.set( diff_time_const_, ctrl_period_sec_ );
  intg.set( ctrl_period_sec_ );

}
/**********************************************************************
サンプル周期(制御周期)を設定し，
メンバ変数を初期化する関数．
引数1 サンプル周期
**********************************************************************/
void PID::set(float ctrl_period_sec){

  ctrl_period_sec_ = ctrl_period_sec;

  // ゲインの初期化
  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;

  // 偏差を初期化
  err_ = 0.0;
  err_diff_ = 0.0;
  err_intg_ = 0.0;

  // 出力値の初期化
  p_output_ = 0.0;
  i_output_ = 0.0;
  d_output_ = 0.0;
  output_ = 0.0;
  limit_output_ = 0x7f7fffff;
  limit_i_output_ = 0x7f7fffff;

  input_ = 0.0;

  // 終了判定の初期化
  end_cnt_ = kEND_CNT_;
  end_err_ = kEND_ERROR_;
  err_diff_ = kEND_DIFFERENTIAL_;

  // 数値微分，数値積分の設定
  diff_time_const_ = kDIFF_TIME_CONST_sec_;
  diff.set( diff_time_const_, ctrl_period_sec_ );
  intg.set( ctrl_period_sec_ );

}
/**********************************************************************
PIDの目標値を設定，変更する関数．
引数1 目標値
**********************************************************************/
void PID::setTarget( float target ){
  target_ = target;
}

/**********************************************************************
PID出力値の上限を変更する関数．
引数1 上限値
**********************************************************************/
void PID::setLimitOutput( float limit_output ){
  limit_output_ = limit_output;
}

/**********************************************************************
積分項の上限を変更する関数．
引数1 上限値
**********************************************************************/
void PID::setLimitIOutput( float limit_i_output ){
  limit_i_output_ = limit_i_output;
}

/**********************************************************************
PIDゲインを設定，変更する関数．
引数1 比例ゲイン
引数2 積分ゲイン
引数3 微分ゲイン
**********************************************************************/
void PID::setGain( float kp, float ki, float kd ){
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

/**********************************************************************
PIDの制御完了判定を変更する関数．
引数1 偏差の終了判定
引数2 偏差の微分の終了判定
**********************************************************************/
void PID::setEndStatus( float end_err, float end_diff ){
  end_err_ = end_err;
  end_diff_ = end_diff;
}

/**********************************************************************
初期の状態をみて1つ前の偏差をつくる(一番最初の微分値の対策)
引数1 センサ等の初期の値(初期状態がどこかを入力)
**********************************************************************/
void PID::setFirstPosition( float first_pos ){
  diff.setInputPrev( first_pos );
}

/**********************************************************************
近似微分器の時定数を設定．未設定なら0.05[s]
引数1 近似微分器の時定数[s]
**********************************************************************/
void PID::setDiffTimeConst( float time_const ){
  diff_time_const_ = time_const;
  diff.set( diff_time_const_, ctrl_period_sec_ );
}

/**********************************************************************
返り値 P制御の出力
**********************************************************************/
float PID::getOutputP(){
  return p_output_;
}

/**********************************************************************
返り値 I制御の出力
**********************************************************************/
float PID::getOutputI(){
  return i_output_;
}

/**********************************************************************
返り値 D制御の出力
**********************************************************************/
float PID::getOutputD(){
  return d_output_;
}

/**********************************************************************
返り値 目標値の出力
**********************************************************************/
float PID::getTarget(){
  return target_;
}

/**********************************************************************
返り値 偏差
**********************************************************************/
float PID::getErr(){
  return err_;
}

/**********************************************************************
返り値 入力
**********************************************************************/
float PID::getInput(){
  return input_;
}

/**********************************************************************
返り値 設定した制御周期
**********************************************************************/
float PID::getCtrlPeriodSec(){
  return ctrl_period_sec_;
}

/**********************************************************************
返り値 Pgain
**********************************************************************/
float PID::getGainP(){
  return kp_;
}
/**********************************************************************
返り値 Igain
**********************************************************************/
float PID::getGainI(){
  return ki_;
}

/**********************************************************************
返り値 Dgain
**********************************************************************/
float PID::getGainD(){
  return kd_;
}

/**********************************************************************
PID制御完了判定を確認する関数．
**********************************************************************/
unsigned char PID::checkEnd(){
  if ( end_cnt_ == 0 ){
    return kEND_PID_;
  }else{
    return kCONTINUE_PID_;
  }
}

/**********************************************************************
終了判定をリセット．出力をリセットせずに次の目標値に収束させるためのやつ．
目標値が随時更新されていくような場合に使う？
**********************************************************************/
void PID::resetEndCnt(){
  end_cnt_ = kEND_CNT_;
}

/**********************************************************************
各変数を初期化する関数．ゲインはそのまま．
**********************************************************************/
void PID::reset(){
  // 偏差を初期化
  err_ = 0.0;
  err_diff_ = 0.0;
  err_intg_ = 0.0;

  // 出力値の初期化
  p_output_ = 0.0;
  i_output_ = 0.0;
  d_output_ = 0.0;
  output_ = 0.0;

  // 終了判定の初期化
  end_cnt_ = kEND_CNT_;

  // 微分，積分クラスの初期化
  diff.reset();
  intg.reset();
}

/**********************************************************************
速度制御用リセット．積分Iをリセットする
**********************************************************************/
void PID::resetforVelocity(){
  intg.reset();
}
/**********************************************************************
設定した目標値に対してPID制御を行う関数．
引数1 フィードバック値
返り値 PID制御器の出力
**********************************************************************/
float PID::control( float input ){

  // 偏差，偏差の微分，偏差の積分の更新
  input_ = input;
  err_ = target_ - input;
  err_intg_ = intg.filter( err_ );
  err_diff_ = diff.filter( err_ );

  // 各制御出力の更新
  p_output_ = kp_ * err_;
  i_output_ = ki_ * err_intg_;
  d_output_ = kd_ * err_diff_;

  if ( fabs( i_output_ ) > limit_i_output_ )
  i_output_ *= limit_i_output_ / fabs( i_output_ );

  output_ = p_output_ + i_output_ + d_output_;

  // 制御完了判定
  if ( fabsf( err_ ) <= end_err_ && fabsf( err_diff_ ) <= end_diff_ ){
    end_cnt_ = ( end_cnt_ > 0 ) ? end_cnt_ - 1 : 0;     // 収束したと判定してからend_cnt=0x0a=10つまり制御周期×10秒だけ繰り返している
  }else{
    end_cnt_ = kEND_CNT_;       // END_CNT=0x0a=10
  }

  if ( fabs( output_ ) >= limit_output_ )
  return limit_output_ * output_ / fabs( output_ );

  return output_;

}
/**********************************************************************
設定した目標値に対してPID制御を行う関数．
引数1 フィードバック値から求めた偏差
返り値 PID制御器の出力
**********************************************************************/
float PID::controlErr( float err ){

  // 偏差，偏差の微分，偏差の積分の更新
  err_ = err;
  err_intg_ = intg.filter( err_ );
  err_diff_ = diff.filter( err_ );

  // 各制御出力の更新
  p_output_ = kp_ * err_;
  i_output_ = ki_ * err_intg_;
  d_output_ = kd_ * err_diff_;

  if ( fabs( i_output_ ) > limit_i_output_ )
  i_output_ *= limit_i_output_ / fabs( i_output_ );

  output_ = p_output_ + i_output_ + d_output_;

  // 制御完了判定
  if ( fabsf( err_ ) <= end_err_ && fabsf( err_diff_ ) <= end_diff_ ){
    end_cnt_ = ( end_cnt_ > 0 ) ? end_cnt_ - 1 : 0;     // 収束したと判定してからend_cnt=0x0a=10つまり制御周期×10秒だけ繰り返している
  }else{
    end_cnt_ = kEND_CNT_;       // END_CNT=0x0a=10
  }

  if ( fabs( output_ ) >= limit_output_ )
  return limit_output_ * output_ / fabs( output_ );

  return output_;

}
/**********************************************************************
設定した目標値に対してI-P制御を行う関数．
引数1 フィードバック値
返り値 PID制御器の出力
**********************************************************************/
float PID::controlIP( float input ){

  // 偏差，偏差の微分，偏差の積分の更新
  err_ = target_ - input;
  err_intg_ = intg.filter( err_ );
  err_diff_ = diff.filter( err_ );

  // 各制御出力の更新
  p_output_ = -kp_ * input;
  i_output_ = ki_ * err_intg_;
  d_output_ = kd_ * err_diff_;

  if ( fabs( i_output_ ) > limit_i_output_ )
  i_output_ *= limit_i_output_ / fabs( i_output_ );

  output_ = p_output_ + i_output_ + d_output_;

  // 制御完了判定
  if ( fabsf( err_ ) <= end_err_ && fabsf( err_diff_ ) <= end_diff_ ){
    end_cnt_ = ( end_cnt_ > 0 ) ? end_cnt_ - 1 : 0;     // 収束したと判定してからend_cnt=0x0a=10つまり制御周期×10秒だけ繰り返している
  }else{
    end_cnt_ = kEND_CNT_;       // END_CNT=0x0a=10
  }

  if ( fabs( output_ ) >= limit_output_ )
  return limit_output_ * output_ / fabs( output_ );

  return output_;

}
