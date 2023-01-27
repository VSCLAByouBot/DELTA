#ifndef PTP_H
#define PTP_H

#include <cmath>
#include "delta/Setting.h"

/**
 * @brief 產生點對點 S-curve 命令
 * @param InitialPos: Current joint position [rad]
 * @param FinalPos: Current joint velocity [rad/s]
 * @param PosCmd: Joint position command [rad]
 * @param VelCmd: Joint velocity command [rad/s]
 * @param AccCmd: Joint acceleration command [rad/s^2]
 * @param EndFlag: 1 if end of commands, 0 if not.
 */
void PTP_Scurve(double InitialPos[AXIS],
                double FinalPos[AXIS],
                double (&PosCmd)[AXIS],
                double (&VelCmd)[AXIS],
                double (&AccCmd)[AXIS],
                int(&EndFlag));

#endif // PTP_H
