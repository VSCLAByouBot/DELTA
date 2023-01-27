#ifndef ERROR_HANDLE_H
#define ERROR_HANDLE_H

// Terminate Program if Error Occurs
#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#define _ROS2_SHUTDOWN rclcpp::shutdown();
#else
#define _ROS2_SHUTDOWN ;
#endif // ROS2

#include <cstdlib>  // For EXIT_SUCCESS, EXIT_FAILURE

// Error Exit Number
#define JOINT_STATE_ERR 2

// Failure Messages
#define FILE_OPEN_ERR_MESSAGE	    " Failed to Create Save Data "
#define TRCK_FILE_INIT_ERR_MESSAGE  " Failed to Init Tracking File "
#define MC_ERR_MESSAGE			    " Motion Card Failed to Init "
#define DO_NOTHING_MESSAGE		    " Nothing has done! Program Terminated "
#define IMC_ABORT_MESSAGE           " UNSAFETY OPERATIONS DETECTED ! "

#define _CHECK(_ERR_COND, _ERR_STR, _RET) \
    {                                     \
        if (_ERR_COND)                    \
        {                                 \
            put_line((_ERR_STR));         \
            _ROS2_SHUTDOWN                \
            return _RET;                  \
        }                                 \
    }

#define CHECK_SUCCESS(ERR_COND, ERR_STR) _CHECK(ERR_COND, ERR_STR, EXIT_SUCCESS)
#define CHECK_FAIL(ERR_COND, ERR_STR)	 _CHECK(ERR_COND, ERR_STR, EXIT_FAILURE)

// #define IMC_ABORT(_ERR_COND, _ERR_STR, _RET)                    \
//     {                                                           \
//         if (_ERR_COND)                                          \
//         {                                                       \
//             MotionCard_ServoOff();  /* Servo Off */             \
//             MotionCard_CloseCard(); /* 關閉軸卡 */          \
//             SaveData_CloseFile();   /* 關閉實驗資料檔 */ \
//             put_line(" All Functions Closed Successfully ");    \
//             _ROS2_SHUTDOWN                                      \
//             put_line((_ERR_STR));                               \
//             return _RET;                                        \
//         }                                                       \
//     }

#endif // ERROR_HANDLE_H
