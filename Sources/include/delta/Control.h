#ifndef CONTROL_H
#define CONTROL_H

#include "delta/Setting.h"

/**
 * @brief ¦^±Â±±¨î(PD-like)
 * @param Pos: Current joint position [rad]
 * @param Vel: Current joint velocity [rad/s]
 * @param PosCmd: Joint position command [rad]
 * @param TorCtrl: Output joint torque command [Nm]
 */
void Control_Feedback(double Pos[AXIS], double Vel[AXIS],
                      double PosCmd[AXIS], double (&TorCtrl)[AXIS]);

#endif // CONTROL_H
