#ifndef PROTECTION_H
#define PROTECTION_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "delta/Setting.h"
#include <string>
#include <array>
#include <random>

// �ˬd�U�b���w���Y�ƭȰ�,�Φb�D�{���� static_assert ��
#define CHK_SF(I) Pos_SF[(I)] < 0.0 || Pos_SF[(I)] > 1.0 || Vel_SF[(I)] < 0.0 || Vel_SF[(I)] > 1.0

enum class Unit
{
    rad,
    deg
};

/**
 * @brief ��l�ƦU�b���A���W�U��
 * @param Pos_SF: Safety factors for each joint position
 * @param Vel_SF: Safety factors for each joint velocity
 * @param checkPos: Position protection switch
 * @param checkVel: Velocity protection switch
 * @param unit: Display unit
 */
void Init_Joint_Bound(const double Pos_SF[AXIS], const double Vel_SF[AXIS],
					  const bool checkPos, const bool checkVel,
                      const enum class Unit unit = Unit::rad);

/**
 * @brief �ˬd�b�t�׬O�_���`(�ª���)
 * @param Vel: Joint velocity [rad/s]
 * @param SafetyFlag: Record safety state. 0 if safe, 1 if danger.
 */
void Protection(const double Vel[AXIS], int &SafetyFlag);

/**
 * @brief �ˬd�b���A�O�_���`
 * @param Pos: Joint position [rad]
 * @param Vel: Joint velocity [rad/s]
 * @param err: Record the value of the error state
 * @param checkPos: Check position if true, skip if false
 * @param checkVel: Check velocity if true, skip if false
 * @return 0 if safe,
 * @return non-zero error code if wrong
 */
int Check_Joint_State(const double Pos[AXIS], const double Vel[AXIS], double &err,
                      const bool checkPos = true, const bool checkVel = true);

/**
 * @brief Get joint state error messages string by error code.
 * @param e: Error code for joint state
 * @param err: Value of the error state
 * @param unit: Display unit
 * @return String of the error message
 */
std::string get_joint_errmsg(int e, double err,
                             const enum class Unit unit = Unit::rad);

/**
 * @brief ��ܩҦ����`���b���A
 * @param Pos: Joint position [rad]
 * @param Vel: Joint velocity [rad/s]
 * @param unit: Display unit
 */
void print_joint_errmsg(const double Pos[AXIS], const double Vel[AXIS],
                        const enum class Unit unit = Unit::rad);

/**
 * @brief �����ܶb���A
 * @param Pos: Joint position [rad]
 * @param Vel: Joint velocity [rad/s]
 * @param unit: Display unit
 * @return The string of the joint state
 */
std::string get_joint_state_one_line(const double Pos[AXIS], const double Vel[AXIS],
                                     const enum class Unit unit = Unit::rad);

/**
 * @brief �N��l�ƶüƲ��;����W�U���]���U�b�w���W�U��
 * @param pos_rnd_gen: �b��m�üƲ��;�
 * @param vel_rnd_gen: �b�t�׶üƲ��;�
 */
void init_rnd_gen(std::uniform_real_distribution<double> pos_rnd_gen[AXIS],
                  std::uniform_real_distribution<double> vel_rnd_gen[AXIS]);

#endif // PROTECTION_H
