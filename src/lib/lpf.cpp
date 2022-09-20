/**********************************************************************
	File	lpf.cpp
	Ver		1.0.1
	Date	1.0.0	2017/9/28
			1.0.1	2017/10/15	setTimeConstとsetInputPrevを追加
	Auther	Maegaki Noriyoshi
	note	ローパスフィルタを双一次変換して離散化した．
**********************************************************************/

/**********************************************************************
 Includes
**********************************************************************/
#include "roblab/lpf.h"
#include <math.h>

/**********************************************************************
 LPF型の変数を定義．時定数とサンプル周期(制御周期)を設定し，
 メンバ変数を初期化，時定数とサンプル周期から求まる係数を計算するコンストラクタ．
	引数1	時定数
	引数2	サンプル周期
**********************************************************************/
LPF::LPF(float t_const_sec, float ctrl_period_sec) : t_const_sec_(t_const_sec), ctrl_period_sec_(ctrl_period_sec){
	
	// 時定数がサンプル周期よりも小さいのはNG
	if(t_const_sec_ < ctrl_period_sec_){
		t_const_sec_ = ctrl_period_sec_;
	}
	
	// 各メンバ変数を初期化
	output_ = 0.0;
	input_prev_ = 0.0;
	
	// ２つの係数をあらかじめ計算
	coeff_a_ = (2.0*t_const_sec_ - ctrl_period_sec_) / (2.0*t_const_sec_ + ctrl_period_sec_);
	coeff_b_ = ctrl_period_sec_ / (2.0*t_const_sec_ + ctrl_period_sec_);
	
}

/**********************************************************************
 時定数とサンプル周期(制御周期)を設定し，
 メンバ変数を初期化，時定数とサンプル周期から求まる係数を計算する関数．
	引数1	時定数
	引数2	サンプル周期
**********************************************************************/
void LPF::set(float t_const_sec, float ctrl_period_sec){
	
	// 時定数がサンプル周期よりも小さいのはNG
	if(t_const_sec < ctrl_period_sec){
		t_const_sec_ = ctrl_period_sec_;
	}else{
		t_const_sec_ = t_const_sec;
	}
	
	ctrl_period_sec_ = ctrl_period_sec;
	
	// 各メンバ変数を初期化
	output_ = 0.0;
	input_prev_ = 0.0;
	
	// ２つの係数をあらかじめ計算
	coeff_a_ = (2.0*t_const_sec_ - ctrl_period_sec_) / (2.0*t_const_sec_ + ctrl_period_sec_);
	coeff_b_ = ctrl_period_sec_ / (2.0*t_const_sec_ + ctrl_period_sec_);
	
}

/**********************************************************************
 時定数を設定し，時定数とサンプル周期から求まる係数を計算する関数．
	引数1	時定数
**********************************************************************/
void LPF::setTimeConst(float t_const_sec){
	
	// 時定数がサンプル周期よりも小さいのはNG
	if(t_const_sec < ctrl_period_sec_){
		t_const_sec_ = ctrl_period_sec_;
	}else{
		t_const_sec_ = t_const_sec;
	}
	
	// ２つの係数をあらかじめ計算
	coeff_a_ = (2.0*t_const_sec_ - ctrl_period_sec_) / (2.0*t_const_sec_ + ctrl_period_sec_);
	coeff_b_ = ctrl_period_sec_ / (2.0*t_const_sec_ + ctrl_period_sec_);
	
}

/**********************************************************************
 ローパスフィルタの初期値を設定する関数．
	引数1	ローパスフィルタの初期値
**********************************************************************/
void LPF::setInputPrev(float input_prev){
	input_prev_ = input_prev;
	output_ = input_prev;
}

/**********************************************************************
 メンバ変数を初期化する関数．
**********************************************************************/
void LPF::reset(){
	
	// 各メンバ変数を初期化
	output_ = 0.0;
	input_prev_ = 0.0;
	
}

/**********************************************************************
 入力値に対してローパスフィルタをかける関数．
	引数1	このフィルタへの入力値
	返り値	このフィルタの出力値
**********************************************************************/
float LPF::filter(float input){
	
	output_ = (coeff_a_ * output_) + (coeff_b_ * (input + input_prev_));
	input_prev_ = input;
	
	return output_;
	
}

