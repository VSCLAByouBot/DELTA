#ifndef MOTIONCARD_H
#define MOTIONCARD_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "IMCDriver.h"
#include "IMCDefine.h"
#include <windows.h> // for Sleep()
#include <iostream>
#include "delta/Setting.h"

#define CLEAR_CMD false // �M�b�d�ɮM�Φ��ѼƦ�MotionCard_OpenCard(),����Ū�������s�X��
#define DEFAULT_MC_ABS_MAX_CNT 5

// Error Code
const int IMC_OPENDEV_ERR = 1;
const int IMC_RD_ABS_ENC_ERR = 2;

/**
 * @brief �}�Ҷb�d
 * @param read_abs_enc: �O�_�nŪ�������s�X��
 * @param rd_abs_enc_max: Ū�������s�X�����̤j���զ���
 * @return 0 if success, nonzero if failed
 */
int MotionCard_OpenCard(bool read_abs_enc = true,
                        const int rd_abs_enc_max = DEFAULT_MC_ABS_MAX_CNT);
/** @brief �����b�d */
void MotionCard_CloseCard();

/**
 * @brief �}�Ҧ��A���F(�P�٨�)
 * @brief �Y��X��x�R�O���s,���u�N�����O�v�T�Ƹ�,�ЯS�O�`�N! */
void MotionCard_ServoOn();

/** @brief �������A���F(�Է٨�) */
void MotionCard_ServoOff();

/**
 * @brief Ū�����﫬�s�X���ƭ�
 * @param JointABSRad: To record the absolute Joint encoder value [rad]
 * @return true if success, false if failed
 */
bool MotionCard_ABS(double *JointABSRad);

/**
 * @brief Ū�����c����
 * @param JointEncRad: To record the Joint encoder value [rad]
 */
void MotionCard_Encoder(double (&JointEncRad)[AXIS]);

/**
 * @brief �e�X�U�b������R�O�ܶb�d
 * @param JointTorCtrl: Joint Torque Command [rad]
 */
void MotionCard_DAC(double (&JointTorCtrl)[AXIS]);

/**
 * @brief �����b�d�����_�Ƶ{��
 * @param Timer: Function pointer
 */
void MotionCard_ChangeTimer(TMRISR Timer);

#endif // MOTIONCARD_H
