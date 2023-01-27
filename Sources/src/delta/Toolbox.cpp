#include "delta/Toolbox.h"

//================= LSF 1/4 =================
void Toolbox_LSF(double x[AXIS], double (&y)[AXIS])
{
	static double xTemp[AXIS][4] = {0.0};

	for (int i = 0; i < AXIS; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			xTemp[i][j] = xTemp[i][j + 1];
		}

		xTemp[i][3] = x[i];

		y[i] = (0.3 * xTemp[i][3] + 0.1 * xTemp[i][2] - 0.1 * xTemp[i][1] - 0.3 * xTemp[i][0]) / SamplingTime;
	}
}

//=============== IIR filter ===============
void Toolbox_Filter(double x[AXIS], double (&y)[AXIS])
{
	// Matlab "fdatool" : [b,a] , Lowpass , IIR-Butterworth , Fs = 1000(Hz) , Fc = 10(Hz)

	// Denominator:
	double a1 = 1;
	double a2 = -1.9111970674260732;
	double a3 = 0.91497583480143374;

	// Numerator * Gain:
	double b1 = 1 * 0.00094469184384015075;
	double b2 = 2 * 0.00094469184384015075;
	double b3 = 1 * 0.00094469184384015075;

	static double xTemp[AXIS][3] = {0.0};
	static double yTemp[AXIS][3] = {0.0};

	for (int i = 0; i < AXIS; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			xTemp[i][j] = xTemp[i][j + 1];
			yTemp[i][j] = yTemp[i][j + 1];
		}

		xTemp[i][2] = x[i];
		y[i] = b1 * xTemp[i][2] + b2 * xTemp[i][1] + b3 * xTemp[i][0] - a2 * yTemp[i][1] - a3 * yTemp[i][0];
		yTemp[i][2] = y[i];
	}
}

//================== Sign ==================
double Toolbox_sign(double x)
{
	if (x < 0.0)
		return -1.0;
	else if (x == 0.0)
		return 0.0;
	else
		return 1.0;
}

// Recommended implementation of sign function
// See https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
// template <typename T>
// double sgn(T val)
// {
//     return (T(0) < val) - (val < T(0));
// }
