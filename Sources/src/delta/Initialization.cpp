#include "delta/Initialization.h"

void Init_PosCmd_LSF(double (&PosCmd)[AXIS])
{
	printf("Initializing PosCmd & LSF ... ");
	double Vel[AXIS] = {0.0};

	for (int i = 0; i < 4; i++) // LSF更新至少需4次
	{
		MotionCard_Encoder(PosCmd); // 更新機構角度 [rad]
		Toolbox_LSF(PosCmd, Vel);	// 更新機構速度 [rad/s]
	}
	puts("OK !");
}
