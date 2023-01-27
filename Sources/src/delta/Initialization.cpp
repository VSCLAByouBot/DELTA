#include "delta/Initialization.h"

void Init_PosCmd_LSF(double (&PosCmd)[AXIS])
{
	printf("Initializing PosCmd & LSF ... ");
	double Vel[AXIS] = {0.0};

	for (int i = 0; i < 4; i++) // LSF��s�ܤֻ�4��
	{
		MotionCard_Encoder(PosCmd); // ��s���c���� [rad]
		Toolbox_LSF(PosCmd, Vel);	// ��s���c�t�� [rad/s]
	}
	puts("OK !");
}
