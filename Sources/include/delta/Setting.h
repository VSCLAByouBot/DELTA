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

//========== 台達機械手臂 & IMP3軸卡參數設定 ==========
#define AXIS 6               // 機械手臂軸數
#define K_56 (53.0 / 1836.0) // 第5,6關節耦合的Pitch補償倍數

//---------- 減速比 ----------
#define GR_0 (2835.0 / 32.0) // 第1軸減速比
#define GR_1 121.0           // 第2軸減速比
#define GR_2 81.0            // 第3軸減速比
#define GR_3 (2091.0 / 31.0) // 第4軸減速比
#define GR_4 (2244.0 / 30.0) // 第5軸減速比
#define GR_5 (2244.0 / 53.0) // 第6軸減速比

//---------- 最大馬達轉矩 = n * 額定轉矩 [Nm] ----------
//---------- 最大馬達轉矩設定,驅動器參數P1-41 ----------
#define MAX_TQ_0 (0.8 * 3.0)   // 第1軸馬達最大轉矩 [Nm]
#define MAX_TQ_1 (0.8 * 2.0)   // 第2軸馬達最大轉矩 [Nm]
#define MAX_TQ_2 (0.557 * 2.0) // 第3軸馬達最大轉矩 [Nm]
#define MAX_TQ_3 (0.32 * 2.0)  // 第4軸馬達最大轉矩 [Nm]
#define MAX_TQ_4 (0.32 * 2.0)  // 第5軸馬達最大轉矩 [Nm]
#define MAX_TQ_5 (0.32 * 2.0)  // 第6軸馬達最大轉矩 [Nm]

#define MAX_VOLTAGE 10.0 // 最大電壓 [V]

//---------- 軸空間範圍限制 [rad] ----------
#define MIN_POS_0 deg2rad(-170.0) // 第1軸最小角度 [rad]
#define MAX_POS_0 deg2rad(170.0)  // 第1軸最大角度 [rad]
#define MIN_POS_1 deg2rad(-105.0) // 第2軸最小角度 [rad]
#define MAX_POS_1 deg2rad(133.0)  // 第2軸最大角度 [rad]
#define MIN_POS_2 deg2rad(-205.0) // 第3軸最小角度 [rad]
#define MAX_POS_2 deg2rad(65.0)   // 第3軸最大角度 [rad]
#define MIN_POS_3 deg2rad(-190.0) // 第4軸最小角度 [rad]
#define MAX_POS_3 deg2rad(190.0)  // 第4軸最大角度 [rad]
#define MIN_POS_4 deg2rad(-120.0) // 第5軸最小角度 [rad]
#define MAX_POS_4 deg2rad(120.0)  // 第5軸最大角度 [rad]
#define MIN_POS_5 deg2rad(-360.0) // 第6軸最小角度 [rad]
#define MAX_POS_5 deg2rad(360.0)  // 第6軸最大角度 [rad]

//---------- 軸速度範圍限制 [rad/s] ----------
#define RATED_RPM 3000.0                        // 驅動器限制馬達最高轉速 3000 [rpm]
#define MAX_VEL_0 (rpm2rad_s(RATED_RPM) / GR_0) // 第1軸最大角速度 [rad/s]
#define MAX_VEL_1 (rpm2rad_s(RATED_RPM) / GR_1) // 第2軸最大角速度 [rad/s]
#define MAX_VEL_2 (rpm2rad_s(RATED_RPM) / GR_2) // 第3軸最大角速度 [rad/s]
#define MAX_VEL_3 (rpm2rad_s(RATED_RPM) / GR_3) // 第4軸最大角速度 [rad/s]
#define MAX_VEL_4 (rpm2rad_s(RATED_RPM) / GR_4) // 第5軸最大角速度 [rad/s]
#define MAX_VEL_5 (rpm2rad_s(RATED_RPM) / GR_5) // 第6軸最大角速度 [rad/s]

//========== 控制器參數設定 ==========
#define SamplingTime 0.001 // [sec]

// 機構慣量常數
#define IM_0 4.1075   // 第1軸機構慣量常數
#define IM_1 4.0711   // 第2軸機構慣量常數
#define IM_2 0.676    // 第3軸機構慣量常數
#define IM_3 0.045156 // 第4軸機構慣量常數
#define IM_4 0.044573 // 第5軸機構慣量常數
#define IM_5 0.013443 // 第6軸機構慣量常數

//---------- PD-like ----------
#define W_0 100.0 // 第1軸自然頻率
#define W_1 80.0  // 第2軸自然頻率
#define W_2 100.0 // 第3軸自然頻率
#define W_3 300.0 // 第4軸自然頻率
#define W_4 250.0 // 第5軸自然頻率
#define W_5 550.0 // 第6軸自然頻率
#define Z_0 0.2   // 第1軸阻尼係數
#define Z_1 0.1   // 第2軸阻尼係數
#define Z_2 0.2   // 第3軸阻尼係數
#define Z_3 0.5   // 第4軸阻尼係數
#define Z_4 0.5   // 第5軸阻尼係數
#define Z_5 1.0   // 第6軸阻尼係數

#endif // SETTING_H
