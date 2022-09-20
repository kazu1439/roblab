/**********************************************************************
	File	differential.cpp
	Ver		1.0.0
	Date	1.0.0	2018/11/18
	Auther	Maegaki Noriyoshi
	note	近似微分器を双一次変換して離散化した．
**********************************************************************/

/**********************************************************************
 Includes
**********************************************************************/
#include "roblab/differential.h"
#include <iostream>
#include <math.h>

/**********************************************************************
 Differential型の変数を定義．時定数とサンプル周期(制御周期)を設定し，
 メンバ変数を初期化，時定数とサンプル周期から求まる係数を計算するコンストラクタ．
	引数1	時定数
	引数2	サンプル周期
**********************************************************************/
Differential::Differential(float t_const_sec, float ctrl_period_sec) : t_const_sec_(t_const_sec), ctrl_period_sec_(ctrl_period_sec){
	
	// 時定数がサンプル周期よりも小さいのはNG
	if(t_const_sec_ < ctrl_period_sec_){
		t_const_sec_ = ctrl_period_sec_;
	}
	
	// 各メンバ変数を初期化
	output_ = 0.0f;
	input_prev_ = 0.0f;
	
	// ２つの係数をあらかじめ計算
	coeff_a_ = (2.0f*t_const_sec_ - ctrl_period_sec_) / (2.0f*t_const_sec_ + ctrl_period_sec_);
	coeff_b_ = 2.0f / (2.0f*t_const_sec_ + ctrl_period_sec_);
	
}

/**********************************************************************
 時定数とサンプル周期(制御周期)を設定し，
 メンバ変数を初期化，時定数とサンプル周期から求まる係数を計算する関数．
	引数1	時定数
	引数2	サンプル周期
**********************************************************************/
void Differential::set(float t_const_sec, float ctrl_period_sec){
	
	// 時定数がサンプル周期よりも小さいのはNG
	if(t_const_sec < ctrl_period_sec){
		t_const_sec_ = ctrl_period_sec_;
	}else{
		t_const_sec_ = t_const_sec;
	}
	
	ctrl_period_sec_ = ctrl_period_sec;
	
	// 各メンバ変数を初期化
	output_ = 0.0f;
	input_prev_ = 0.0f;
	
	// ２つの係数をあらかじめ計算
	coeff_a_ = (2.0f*t_const_sec_ - ctrl_period_sec_) / (2.0f*t_const_sec_ + ctrl_period_sec_);
	coeff_b_ = 2.0f / (2.0f*t_const_sec_ + ctrl_period_sec_);
	
	
}

/**********************************************************************
 時定数を設定し，時定数とサンプル周期から求まる係数を計算する関数．
	引数1	時定数
**********************************************************************/
void Differential::setTimeConst(float t_const_sec){
	
	// 時定数がサンプル周期よりも小さいのはNG
	if(t_const_sec < ctrl_period_sec_){
		t_const_sec_ = ctrl_period_sec_;
	}else{
		t_const_sec_ = t_const_sec;
	}
	
	// ２つの係数をあらかじめ計算
	coeff_a_ = (2.0f*t_const_sec_ - ctrl_period_sec_) / (2.0f*t_const_sec_ + ctrl_period_sec_);
	coeff_b_ = 2.0f / (2.0f*t_const_sec_ + ctrl_period_sec_);
	
}

/**********************************************************************
 近似微分器の初期値を設定する関数．
 入力データにオフセットがある場合，微分の最初のデータはふっとんだ値になる．
 オフセット量がある程度わかっているときにこの関数で設定しておくとふっとんだ値になるのを防ぐことができる．
 エンコーダの値を微分するなら，この関数でそのときのエンコーダの値をセットしておくとよい．
	引数1	入力の初期値
**********************************************************************/
void Differential::setInputPrev(float input_prev){
	input_prev_ = input_prev;
}

/**********************************************************************
 近似微分器への出力の初期値を設定する関数．
 最初の出力が0ではなく，ある程度の値を持つことが分かっている場合，最初の方の遅れをなくせる．
 多分使わない．
	引数1	出力の初期値
**********************************************************************/
void Differential::setOutputPrev(float output_prev){
	output_ = output_prev;
}

/**********************************************************************
 メンバ変数を初期化する関数．
**********************************************************************/
void Differential::reset(){
	// 各メンバ変数を初期化
	output_ = 0.0f;
	input_prev_ = 0.0f;
}

/**********************************************************************
 入力値に対して近似微分器に通す関数．
	引数1	このフィルタへの入力値
	返り値	このフィルタの出力値
**********************************************************************/
float Differential::filter(float input){
	
	output_ = (coeff_a_ * output_) + (coeff_b_ * (input - input_prev_));
	input_prev_ = input;
	
	return output_;
	
}

