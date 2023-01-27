#ifndef MOTIONCARD_H
#define MOTIONCARD_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "IMCDriver.h"
#include "IMCDefine.h"
#include <windows.h> // for Sleep()
#include <iostream>
#include "delta/Setting.h"

#define CLEAR_CMD false // 清軸卡時套用此參數至MotionCard_OpenCard(),不需讀取絕對行編碼器
#define DEFAULT_MC_ABS_MAX_CNT 5

// Error Code
const int IMC_OPENDEV_ERR = 1;
const int IMC_RD_ABS_ENC_ERR = 2;

/**
 * @brief 開啟軸卡
 * @param read_abs_enc: 是否要讀取絕對行編碼器
 * @param rd_abs_enc_max: 讀取絕對行編碼器之最大嘗試次數
 * @return 0 if success, nonzero if failed
 */
int MotionCard_OpenCard(bool read_abs_enc = true,
                        const int rd_abs_enc_max = DEFAULT_MC_ABS_MAX_CNT);
/** @brief 關閉軸卡 */
void MotionCard_CloseCard();

/**
 * @brief 開啟伺服馬達(鬆煞車)
 * @brief 若輸出扭矩命令為零,手臂將受重力影響滑落,請特別注意! */
void MotionCard_ServoOn();

/** @brief 關閉伺服馬達(拉煞車) */
void MotionCard_ServoOff();

/**
 * @brief 讀取絕對型編碼器數值
 * @param JointABSRad: To record the absolute Joint encoder value [rad]
 * @return true if success, false if failed
 */
bool MotionCard_ABS(double *JointABSRad);

/**
 * @brief 讀取機構角度
 * @param JointEncRad: To record the Joint encoder value [rad]
 */
void MotionCard_Encoder(double (&JointEncRad)[AXIS]);

/**
 * @brief 送出各軸之控制命令至軸卡
 * @param JointTorCtrl: Joint Torque Command [rad]
 */
void MotionCard_DAC(double (&JointTorCtrl)[AXIS]);

/**
 * @brief 切換軸卡之中斷副程式
 * @param Timer: Function pointer
 */
void MotionCard_ChangeTimer(TMRISR Timer);

#endif // MOTIONCARD_H
