#ifndef TOOLBOX_H
#define TOOLBOX_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "delta/Setting.h"

/**
 * @brief Least-Square Fit 1/4 Velocity Estimator
 * @param x: Current joint position [rad]
 * @param y: Current joint velocity [rad/s]
 */
void Toolbox_LSF(double x[AXIS], double (&y)[AXIS]);

/**
 * @brief IIR Filter
 * @param x: (unknown)
 * @param y: (unknown)
 */
void Toolbox_Filter(double x[AXIS], double (&y)[AXIS]);

/**
 * @brief Sign Function
 * @param x: input
 * @return -1, 0, +1
 */
double Toolbox_sign(double x);

#endif // TOOLBOX_H
