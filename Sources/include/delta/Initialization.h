#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include "delta/MotionCard.h"
#include "delta/Toolbox.h"

/**
 * @brief 讀取機構角度作為初始角度命令(緩衝1的角度命令)並且更新LSF
 * @param PosCmd: Joint position command [rad]
 */
void Init_PosCmd_LSF(double (&PosCmd)[AXIS]);

#endif // INITIALIZATION_H
