/**********************************************************************
File    pid.h
Ver     1.2.2
Date    1.0.0   2017/10/04
1.1.0   2018/11/18      // 微分と積分を精度の高いものに変更
1.1.1   2018/11/20      // P制御，I制御，D制御それぞれの出力を返す関数を作成
1.2.1   2018/12/05      // PID出力,積分値に上限を付けられる変数と関数,Iの出力だけ初期化する関数を追加(Kiryu.H)
1.2.2   2019/05/14      // 制御周期をセットできる関数を追加しました．あとインスタンス生成時引数を渡さなくてもよくしました(TakahiroYamazaki)
Auther  Maegaki Noriyoshi
**********************************************************************/

/**********************************************************************
Include guard
**********************************************************************/
#ifndef __PID_H__
#define __PID_H__

/**********************************************************************
Includes
**********************************************************************/
#include "differential.h"
#include "integral.h"

/**********************************************************************
PID class
**********************************************************************/
class PID
{

public:
  PID() {}
  PID(float ctrl_period_sec);

  // 各変数を設定，変更する関数
  void set(float ctrl_period_sec);
  void setTarget(float target_);
  void setGain(float kp, float ki, float kd);
  void setEndStatus(float end_err, float end_diff); // 制御完了判定を変更する関数，変更前はコンストラクタで初期化されている
  void setFirstPosition(float first_pos);           // 初期の状態をみて1つ前の偏差をつくる(一番最初の微分値の対策)
  void setDiffTimeConst(float time_const);          // 近似微分器の時定数を設定．初期値は0.05．
  void setLimitOutput(float limit_output);
  void setLimitIOutput(float limit_i_output);

  // 計算結果を出力する関数
  float getOutputP();
  float getOutputI();
  float getOutputD();
  float getGainP();
  float getGainI();
  float getGainD();
  float getTarget();
  float getErr();
  float getInput();
  float getgetCtrlPeriodSec();
  float getCtrlPeriodSec();

  unsigned char checkEnd(); // 制御完了判定を確認する関数
  void resetEndCnt();       // 新たに目標値に向かって収束させたいときend_cnt_をリセットする，
  void reset();
  void resetforVelocity();
  float control(float input);
  float controlIP(float input);
  float controlErr(float err);

private:
  // 制御完了の判定
  static const unsigned char kEND_CNT_ = 10; // control関数を実行中に，設定した偏差と偏差の微分を下回ったらカウントを下げ（control関数内），0になったらcheckEnd関数がEND_PIDを返す．
  const float kEND_ERROR_ = 0.0f;
  const float kEND_DIFFERENTIAL_ = 0.0f;

  // checkEnd関数で返す値
  static const unsigned char kEND_PID_ = 0;
  static const unsigned char kCONTINUE_PID_ = 1;

  float ctrl_period_sec_; // 制御周期[s]
  float target_;          // 目標値
  float input_;           // PIDの出力値
  float output_;          // PIDの出力値
  float limit_output_;    // 出力値の上限
  float limit_i_output_;  // 積分項の上限

  // 数値微分，数値積分するためのやつ
  const float kDIFF_TIME_CONST_sec_ = 0.05f;
  float diff_time_const_;
  Differential diff;
  Integral intg;

  // 偏差
  float err_;      // 目標値との偏差
  float err_intg_; // 偏差の積分値
  float err_diff_; // 偏差の微分値

  // 各要素の出力値
  float p_output_;
  float i_output_;
  float d_output_;

  // ゲイン
  float kp_; // 比例ゲイン
  float ki_; // 積分ゲイン
  float kd_; // 微分ゲイン

  // 終了判定
  unsigned char end_cnt_; // 制御完了判定のカウント
  float end_err_;         // 偏差の終了判定
  float end_diff_;        // 偏差微分の終了判定
};

#endif

/**********************************************************************
Usage example
**********************************************************************/
