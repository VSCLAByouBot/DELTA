#include "delta/Control.h"

// TOTEST: Simplify control params as consts
const double InertiaMoment[AXIS] = {IM_0, IM_1, IM_2, IM_3, IM_4, IM_5}; // 诀篶篋秖盽计
const double Kp[AXIS] = {W_0 / (2.0 * Z_0), W_1 / (2.0 * Z_1), W_2 / (2.0 * Z_2),
						 W_3 / (2.0 * Z_3), W_4 / (2.0 * Z_4), W_5 / (2.0 * Z_5)};
const double Kv[AXIS] = {2.0 * Z_0 * W_0, 2.0 * Z_1 *W_1, 2.0 * Z_2 *W_2,
						 2.0 * Z_3 *W_3, 2.0 * Z_4 *W_4, 2.0 * Z_5 *W_5};

//================ Feedback Control (PD-like) ================
void Control_Feedback(double Pos[AXIS], double Vel[AXIS], double PosCmd[AXIS], double (&TorCtrl)[AXIS])
{
	/*	double Kp[AXIS] = { 650.0 , 450.0 , 400.0 , 400.0 , 380.0 , 400.0 } ;
		double Kv[AXIS] = { 150.0 ,  80.0 ,  25.0 ,  20.0 ,  10.0 ,  15.0 } ;*/

	//-----------------------------------------------------------------------------------
	//double InertiaMoment[AXIS] = {4.1075, 4.0711, 0.676, 0.045156, 0.044573, 0.013443}; // 诀篶篋秖盽计
	//double w[AXIS] = {100.0, 80.0, 100.0, 300.0, 250.0, 550.0}; // 礛繵瞯
	//double z[AXIS] = {0.2, 0.1, 0.2, 0.5, 0.5, 1.0};			// ェ玒计

	for (int i = 0; i < AXIS; i++)
	{
		// TorCtrl[i] = Kv[i] * ( Kp[i] * ( PosCmd[i] - Pos[i] ) - Vel[i] ) ; // 锣痻北㏑(Nm)

		//-----------------------------------------------------------------------------------

		//double Kp = w[i] / (2 * z[i]);
		//double Kv = 2 * z[i] * w[i];
		TorCtrl[i] = InertiaMoment[i] * (Kv[i] * (Kp[i] * (PosCmd[i] - Pos[i]) - Vel[i])); // 锣痻北㏑(Nm)
	}
}