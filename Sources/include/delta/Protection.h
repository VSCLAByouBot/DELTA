#ifndef PROTECTION_H
#define PROTECTION_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "delta/Setting.h"
#include <string>
#include <array>
#include <random>

// 檢查各軸之安全係數值域,用在主程式之 static_assert 中
#define CHK_SF(I) Pos_SF[(I)] < 0.0 || Pos_SF[(I)] > 1.0 || Vel_SF[(I)] < 0.0 || Vel_SF[(I)] > 1.0

enum class Unit
{
    rad,
    deg
};

/**
 * @brief 初始化各軸狀態之上下限
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
 * @brief 檢查軸速度是否正常(舊版本)
 * @param Vel: Joint velocity [rad/s]
 * @param SafetyFlag: Record safety state. 0 if safe, 1 if danger.
 */
void Protection(const double Vel[AXIS], int &SafetyFlag);

/**
 * @brief 檢查軸狀態是否正常
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
 * @brief 顯示所有異常的軸狀態
 * @param Pos: Joint position [rad]
 * @param Vel: Joint velocity [rad/s]
 * @param unit: Display unit
 */
void print_joint_errmsg(const double Pos[AXIS], const double Vel[AXIS],
                        const enum class Unit unit = Unit::rad);

/**
 * @brief 單行顯示軸狀態
 * @param Pos: Joint position [rad]
 * @param Vel: Joint velocity [rad/s]
 * @param unit: Display unit
 * @return The string of the joint state
 */
std::string get_joint_state_one_line(const double Pos[AXIS], const double Vel[AXIS],
                                     const enum class Unit unit = Unit::rad);

/**
 * @brief 將初始化亂數產生器之上下限設為各軸安全上下限
 * @param pos_rnd_gen: 軸位置亂數產生器
 * @param vel_rnd_gen: 軸速度亂數產生器
 */
void init_rnd_gen(std::uniform_real_distribution<double> pos_rnd_gen[AXIS],
                  std::uniform_real_distribution<double> vel_rnd_gen[AXIS]);

#endif // PROTECTION_H
