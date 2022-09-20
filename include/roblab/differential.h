/**********************************************************************
	File	differential.h
	Ver		1.0.0
	Date	1.0.0	2018/11/18
	Auther	Maegaki Noriyoshi
	note	近似微分器を双一次変換して離散化した．
**********************************************************************/

/**********************************************************************
 Include guard
**********************************************************************/
#ifndef __DIFFERENTIAL_H__
#define __DIFFERENTIAL_H__

/**********************************************************************
 Includes
**********************************************************************/


/**********************************************************************
 Differential class
**********************************************************************/
class Differential{
	public:
		Differential(){}
		Differential(float t_const_sec, float ctrl_period_sec);
		
		void set(float t_const_sec, float ctrl_period_sec);	// 時定数とサンプル周期を設定する関数
		void setTimeConst(float t_const_sec);				// 時定数のみを再設定する関数．変数の初期化はしない
		void setInputPrev(float input_prev);				// 入力列の前回の値を（0以外にするの場合）設定する関数．
		void setOutputPrev(float output_prev);				// 微分の前回の値を（0以外にするの場合）設定する関数．
		void reset();
		float filter(float input);
		
	private:
		float t_const_sec_;
		float ctrl_period_sec_;	
		float output_;
		float input_prev_;
		
		float coeff_a_;			// フィルタをかけるときに使用する係数a
		float coeff_b_;			// フィルタをかけるときに使用する係数b
	
};

#endif

/**********************************************************************
 Usage example
**********************************************************************/



