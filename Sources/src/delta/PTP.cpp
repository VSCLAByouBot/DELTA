#include "delta/PTP.h"

double jerk[AXIS];
double Time = 0.0;
double Displacement[AXIS];
double max = 0.0;
double Ta, Tb, Tc, Ts, t1, t2, t3, t4, t5, t6, t7;
double DisplacementFunction;
int NumberMax; // The Axis number of the max displacement
constexpr double a_avg = 0.75;	// 範圍: [0.5, 1]
static_assert(a_avg >= 0.5 && a_avg <= 1.0, "PTP.cpp(11): average of acceleration must within [0.5, 1]");

void PTP_Scurve(double InitialPos[AXIS], double FinalPos[AXIS], double (&PosCmd)[AXIS], double (&VelCmd)[AXIS], double (&AccCmd)[AXIS], int(&EndFlag))
{
	if (Time == 0.0)
	{
		double Vmax = 1.0;	// rad/s
		double Amax = 1.5;	// rad/s^2
		double Aavg = a_avg * Amax;

		// 找出最大位移之軸編號
		for (int i = 0; i < AXIS; i++)
		{
			Displacement[i] = FinalPos[i] - InitialPos[i];

			if (abs(Displacement[i]) > max)
			{
				max = abs(Displacement[i]);
				NumberMax = i;
			}
		}

		if (Displacement[NumberMax] < 0.0)
		{
			Vmax = -Vmax;
			Amax = -Amax;
			Aavg = -Aavg;
		}

		Ta = Vmax / Aavg;
		Tb = 2 * Vmax / Amax - Ta;
		Tc = (Ta - Tb) / 2;
		Ts = (Displacement[NumberMax] - Vmax * Ta) / Vmax;
		if (Ts < 0.0)
		{
			Ts = 0.0;
		}

		t1 = Tc;
		t2 = Tc + Tb;
		t3 = Ta;
		t4 = Ta + Ts;
		t5 = Ta + Ts + Tc;
		t6 = Ta + Ts + Tc + Tb;
		t7 = Ta + Ts + Ta;

		DisplacementFunction = pow(t1, 3) / 6 - (pow(t1, 2) * t7) / 2 - (t1 * pow(t3, 2)) / 2 +
							   t1 * t3 * t7 + pow(t2, 3) / 6 - (pow(t2, 2) * t7) / 2 -
							   (t2 * pow(t3, 2)) / 2 + t2 * t3 * t7 + pow(t3, 3) / 3 -
							   (pow(t3, 2) * t7) / 2 + pow(t4, 3) / 6 - (pow(t4, 2) * t7) / 2 +
							   (t4 * pow(t7, 2)) / 2 - pow(t5, 3) / 6 + (pow(t5, 2) * t7) / 2 -
							   (t5 * pow(t7, 2)) / 2 - pow(t6, 3) / 6 + (pow(t6, 2) * t7) / 2 -
							   (t6 * pow(t7, 2)) / 2 + pow(t7, 3) / 6;

		for (int i = 0; i < AXIS; i++)
		{
			jerk[i] = Displacement[i] / DisplacementFunction;
		}
	}

	if (Time <= t7)
	{
		for (int i = 0; i < AXIS; i++)
		{
			if (Time <= t1)
			{
				AccCmd[i] = jerk[i] * Time;
				VelCmd[i] = jerk[i] * pow(Time, 2) / 2;
				PosCmd[i] = jerk[i] * pow(Time, 3) / 6 + InitialPos[i];
			}
			else if (Time > t1 && Time <= t2)
			{
				AccCmd[i] = jerk[i] * t1;
				VelCmd[i] = jerk[i] * Time * t1 - jerk[i] * pow(t1, 2) / 2;
				PosCmd[i] = (jerk[i] * pow(Time, 2) * t1) / 2 - (jerk[i] * Time * pow(t1, 2)) / 2 +
							(jerk[i] * pow(t1, 3)) / 6 + InitialPos[i];
			}
			else if (Time > t2 && Time <= t3)
			{
				AccCmd[i] = jerk[i] * t1 - jerk[i] * Time + jerk[i] * t2;
				VelCmd[i] = jerk[i] * Time * t1 - (jerk[i] * pow(t1, 2)) / 2 - (jerk[i] * pow(t2, 2)) / 2 -
							(jerk[i] * pow(Time, 2)) / 2 + jerk[i] * Time * t2;
				PosCmd[i] = jerk[i] * (-pow(Time, 3) / 6 + (pow(Time, 2) * t1) / 2 + (pow(Time, 2) * t2) / 2 -
									   (Time * pow(t1, 2)) / 2 - (Time * pow(t2, 2)) / 2 + pow(t1, 3) / 6 + pow(t2, 3) / 6) + InitialPos[i];
			}
			else if (Time > t3 && Time <= t4)
			{
				AccCmd[i] = 0.0;
				VelCmd[i] = jerk[i] * (t1 * t3 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t1, 2) / 2 + t2 * t3);
				PosCmd[i] = jerk[i] * (pow(t1, 3) / 6 - (Time * pow(t1, 2)) / 2 - (t1 * pow(t3, 2)) / 2 +
									   Time * t1 * t3 + pow(t2, 3) / 6 - (Time * pow(t2, 2)) / 2 - (t2 * pow(t3, 2)) / 2 + Time * t2 * t3 +
									   pow(t3, 3) / 3 - (Time * pow(t3, 2)) / 2) + InitialPos[i];
			}
			else if (Time > t4 && Time <= t5)
			{
				AccCmd[i] = jerk[i] * t4 - jerk[i] * Time;
				VelCmd[i] = jerk[i] * (Time * t4 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 -
									   pow(t4, 2) / 2 - pow(Time, 2) / 2 + t1 * t3 + t2 * t3);
				PosCmd[i] = jerk[i] * (-pow(Time, 3) / 6 + (pow(Time, 2) * t4) / 2 - (Time * pow(t1, 2)) / 2 +
									   Time * t1 * t3 - (Time * pow(t2, 2)) / 2 + Time * t2 * t3 - (Time * pow(t3, 2)) / 2 -
									   (Time * pow(t4, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 -
									   (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6) + InitialPos[i];
			}
			else if (Time > t5 && Time <= t6)
			{
				AccCmd[i] = jerk[i] * t4 - jerk[i] * t5;
				VelCmd[i] = jerk[i] * (pow(t5, 2) / 2 - Time * t5 - pow(t1, 2) / 2 - pow(t2, 2) / 2 -
									   pow(t3, 2) / 2 - pow(t4, 2) / 2 + Time * t4 + t1 * t3 + t2 * t3);
				PosCmd[i] = jerk[i] * ((pow(Time, 2) * t4) / 2 - (pow(Time, 2) * t5) / 2 - (Time * pow(t1, 2)) / 2 +
									   Time * t1 * t3 - (Time * pow(t2, 2)) / 2 + Time * t2 * t3 - (Time * pow(t3, 2)) / 2 -
									   (Time * pow(t4, 2)) / 2 + (Time * pow(t5, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 +
									   pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6) + InitialPos[i];
			}
			else if (Time > t6 && Time <= t7)
			{
				AccCmd[i] = jerk[i] * Time + jerk[i] * t4 - jerk[i] * t5 - jerk[i] * t6;
				VelCmd[i] = jerk[i] * (pow(Time, 2) / 2 - Time * t5 - Time * t6 + Time * t4 + pow(t5, 2) / 2 +
									   pow(t6, 2) / 2 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + t1 * t3 + t2 * t3);
				PosCmd[i] = jerk[i] * (pow(Time, 3) / 6 + (pow(Time, 2) * t4) / 2 - (pow(Time, 2) * t5) / 2 -
									   (pow(Time, 2) * t6) / 2 - (Time * pow(t1, 2)) / 2 + Time * t1 * t3 - (Time * pow(t2, 2)) / 2 +
									   Time * t2 * t3 - (Time * pow(t3, 2)) / 2 - (Time * pow(t4, 2)) / 2 + (Time * pow(t5, 2)) / 2 +
									   (Time * pow(t6, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 +
									   pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6 - pow(t6, 3) / 6) + InitialPos[i];
			}

		} // end for axis = 1:6

		Time = Time + SamplingTime;
		EndFlag = 0; // 判別PTP命令是否結束(未結束)

	} // end if Time <=t7
	else
	{
		Time = 0.0; // 歸零
		max = 0.0;
		for (int i = 0; i < AXIS; i++) // 可忽略 ?
		{
			PosCmd[i] = FinalPos[i];
			VelCmd[i] = 0.0;
			AccCmd[i] = 0.0;
		}
		EndFlag = 1; // 判別PTP命令是否結束(結束)
	}
}
