#ifndef SETTING_H
#define SETTING_H

#define _USE_MATH_DEFINES // For M_PI, M_1_PI
#include <math.h>

//========== constexpr: Calculate at compile stage ==========
// Convert radian to degree
constexpr double rad2deg(double x)
{
    return x * M_1_PI * 180.0; // M_1_PI := 1/PI
}

// Convert degree to radian
constexpr double deg2rad(double x)
{
    return x * M_PI / 180.0;
}

// Convert rpm to radian per sec
constexpr double rpm2rad_s(double x)
{
    return x * M_PI / 30.0;
}

//========== �x�F������u & IMP3�b�d�ѼƳ]�w ==========
#define AXIS 6               // ������u�b��
#define K_56 (53.0 / 1836.0) // ��5,6���`���X��Pitch���v����

//---------- ��t�� ----------
#define GR_0 (2835.0 / 32.0) // ��1�b��t��
#define GR_1 121.0           // ��2�b��t��
#define GR_2 81.0            // ��3�b��t��
#define GR_3 (2091.0 / 31.0) // ��4�b��t��
#define GR_4 (2244.0 / 30.0) // ��5�b��t��
#define GR_5 (2244.0 / 53.0) // ��6�b��t��

//---------- �̤j���F��x = n * �B�w��x [Nm] ----------
//---------- �̤j���F��x�]�w,�X�ʾ��Ѽ�P1-41 ----------
#define MAX_TQ_0 (0.8 * 3.0)   // ��1�b���F�̤j��x [Nm]
#define MAX_TQ_1 (0.8 * 2.0)   // ��2�b���F�̤j��x [Nm]
#define MAX_TQ_2 (0.557 * 2.0) // ��3�b���F�̤j��x [Nm]
#define MAX_TQ_3 (0.32 * 2.0)  // ��4�b���F�̤j��x [Nm]
#define MAX_TQ_4 (0.32 * 2.0)  // ��5�b���F�̤j��x [Nm]
#define MAX_TQ_5 (0.32 * 2.0)  // ��6�b���F�̤j��x [Nm]

#define MAX_VOLTAGE 10.0 // �̤j�q�� [V]

//---------- �b�Ŷ��d�򭭨� [rad] ----------
#define MIN_POS_0 deg2rad(-170.0) // ��1�b�̤p���� [rad]
#define MAX_POS_0 deg2rad(170.0)  // ��1�b�̤j���� [rad]
#define MIN_POS_1 deg2rad(-105.0) // ��2�b�̤p���� [rad]
#define MAX_POS_1 deg2rad(133.0)  // ��2�b�̤j���� [rad]
#define MIN_POS_2 deg2rad(-205.0) // ��3�b�̤p���� [rad]
#define MAX_POS_2 deg2rad(65.0)   // ��3�b�̤j���� [rad]
#define MIN_POS_3 deg2rad(-190.0) // ��4�b�̤p���� [rad]
#define MAX_POS_3 deg2rad(190.0)  // ��4�b�̤j���� [rad]
#define MIN_POS_4 deg2rad(-120.0) // ��5�b�̤p���� [rad]
#define MAX_POS_4 deg2rad(120.0)  // ��5�b�̤j���� [rad]
#define MIN_POS_5 deg2rad(-360.0) // ��6�b�̤p���� [rad]
#define MAX_POS_5 deg2rad(360.0)  // ��6�b�̤j���� [rad]

//---------- �b�t�׽d�򭭨� [rad/s] ----------
#define RATED_RPM 3000.0                        // �X�ʾ�����F�̰���t 3000 [rpm]
#define MAX_VEL_0 (rpm2rad_s(RATED_RPM) / GR_0) // ��1�b�̤j���t�� [rad/s]
#define MAX_VEL_1 (rpm2rad_s(RATED_RPM) / GR_1) // ��2�b�̤j���t�� [rad/s]
#define MAX_VEL_2 (rpm2rad_s(RATED_RPM) / GR_2) // ��3�b�̤j���t�� [rad/s]
#define MAX_VEL_3 (rpm2rad_s(RATED_RPM) / GR_3) // ��4�b�̤j���t�� [rad/s]
#define MAX_VEL_4 (rpm2rad_s(RATED_RPM) / GR_4) // ��5�b�̤j���t�� [rad/s]
#define MAX_VEL_5 (rpm2rad_s(RATED_RPM) / GR_5) // ��6�b�̤j���t�� [rad/s]

//========== ����ѼƳ]�w ==========
#define SamplingTime 0.001 // [sec]

// ���c�D�q�`��
#define IM_0 4.1075   // ��1�b���c�D�q�`��
#define IM_1 4.0711   // ��2�b���c�D�q�`��
#define IM_2 0.676    // ��3�b���c�D�q�`��
#define IM_3 0.045156 // ��4�b���c�D�q�`��
#define IM_4 0.044573 // ��5�b���c�D�q�`��
#define IM_5 0.013443 // ��6�b���c�D�q�`��

//---------- PD-like ----------
#define W_0 100.0 // ��1�b�۵M�W�v
#define W_1 80.0  // ��2�b�۵M�W�v
#define W_2 100.0 // ��3�b�۵M�W�v
#define W_3 300.0 // ��4�b�۵M�W�v
#define W_4 250.0 // ��5�b�۵M�W�v
#define W_5 550.0 // ��6�b�۵M�W�v
#define Z_0 0.2   // ��1�b�����Y��
#define Z_1 0.1   // ��2�b�����Y��
#define Z_2 0.2   // ��3�b�����Y��
#define Z_3 0.5   // ��4�b�����Y��
#define Z_4 0.5   // ��5�b�����Y��
#define Z_5 1.0   // ��6�b�����Y��

#endif // SETTING_H
