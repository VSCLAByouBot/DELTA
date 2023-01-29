//==================== 定義輸入/輸出檔案名稱 ====================
// 輸出檔名方法一: 自定義完整檔案路徑及名稱
// #define MY_OUTPUT_FILENAME "src\\delta\\data\\MyData_0125.txt" // 欲使用方法二,請將此行註解

// 輸出檔名方法二: 使用當前系統時間戳記作為檔案名稱後綴(請將方法一註解以啟用方法二)
#define OUTPUT_PATH 	"data\\"			// 定義路徑
#define OUTPUT_PREFIX	"ReadEncMeasured_"	// 定義前綴

//==================== Integrated Functions ====================
#include "delta/Initialization.h"
#include "delta/Protection.h"
#include "SaveData.h"
#include "error_handle.h"
#include "keyboard.h"
#include "box.h"
#include "clock.h"

//==================== 定義各軸位置與速度之安全係數(0 ~ 1),並且在編譯階段檢查 ====================
constexpr double Pos_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
constexpr double Vel_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
// 在編譯期間檢查安全係數的值域,若超過則編譯失敗,不會進到執行階段才檢查
static_assert(!(CHK_SF(0) || CHK_SF(1) || CHK_SF(2) || CHK_SF(3) || CHK_SF(4) || CHK_SF(5)),
			  "Safety Factors (Pos_SF, Vel_SF) should all be 0 ~ 1.");
const bool CHECKPOS = true; // 選擇是否開啟位置保護功能
const bool CHECKVEL = true; // 選擇是否開啟速度保護功能

//======================= Counter & Flag =======================
int SafetyFlag = 0; // 軸狀態保護旗標 (0:安全, 其他:軸狀態錯誤代碼)
int TimerFlag = 0;	// 中斷切換旗標
int EndFlag1 = 0;	// 命令結束旗標1
int EndFlag2 = 0;	// 命令結束旗標2
int StayCount = 0;	// 緩衝用計數

//===============================================================
double InitialPos[AXIS];	  // 初始角度 [rad]
double Pos[AXIS] = {0.0};	  // 編碼器角度 [rad]
double Vel[AXIS] = {0.0};	  // LSF速度 [rad/s]
double Acc[AXIS] = {0.0};	  // TODO: 待加速度估測完成 [rad/s^2]
double PosCmd[AXIS] = {0.0};  // 角度命令 [rad]
double VelCmd[AXIS] = {0.0};  // 速度命令 [rad/s]
double AccCmd[AXIS] = {0.0};  // 加速度命令 [rad/s^2]
double TorCtrl[AXIS] = {0.0}; // 軸轉矩控制訊號(減速前) [Nm]

// double TorM[AXIS] = {0.0};	 // 前饋慣性力
// double TorN[AXIS] = {0.0};	 // 前饋科氏力&重力
// double TorF[AXIS] = {0.0};	 // 前饋摩擦力
// double TorF_f[AXIS] = {0.0}; // 濾波後前饋摩擦力
// double TorFF[AXIS] = {0.0};	 // 總前饋轉矩

//================== 宣告軸卡之中斷副程式 ==================
void _stdcall Timer_ReadEncoder(TMRINT *pstINTSource);

//========================== 建立讀寫軌跡之實例 ==========================
SaveData record;	// 儲存軌跡資訊檔

int main(int argc, char *argv[])
{
	//-------------------- 顯示程式概述與操作提示(預備階段) --------------------
	put_hint("(DELTA) READ ENCODER\n"
			 "Show All Axes Position, Velocity, Acceleration(TODO: derivatives)");

	//-------------------- 初始化各軸位置與速度之上下限 --------------------
	Init_Joint_Bound(Pos_SF, Vel_SF, CHECKPOS, CHECKVEL, Unit::rad);

	while (!_kbhit()) // 按任意鍵開始程式
		;
	int sw = _getch(); // Get ASCII of the pressed key

	// 在預備階段按下ESC,則立即關閉程式,返回成功狀態
	CHECK_SUCCESS(sw == ESC_KEY, DO_NOTHING_MESSAGE);

	//---------------------- 讀取資料 ----------------------
	CHECK_FAIL(record.CreateSaveData(__OUTPUT, __TIMESTAMP) != 0,
		FILE_OPEN_ERR_MESSAGE); // 建立實驗資料檔

	//-------------------- 開啟各項功能 --------------------
	CHECK_FAIL(MotionCard_OpenCard() != 0, MC_ERR_MESSAGE); // 開啟軸卡,若失敗則結束程式
	Init_PosCmd_LSF(PosCmd);								// 給初始角度命令 & 更新LSF

	//---------------------- 工作流程 ----------------------
	Clock clk;				   // 計時器,用來控制數據顯示在螢幕上的頻率
	double _safety_prob = 0.0; // 返回錯誤數值
	bool no_wait = true;	   // 一進入工作流程,就直接顯示位置與速度值

	put_line(" Launching Workflow ... ");
	clk.start();
	while (sw != ESC_KEY)
	{
		//-------------------- 手動結束程式 --------------------
		if (_kbhit())
			sw = _getch(); // 按ESC結束程式

		//-------------------- 位置速度保護程式 --------------------
		SafetyFlag = Check_Joint_State(Pos, Vel, _safety_prob, CHECKPOS, CHECKVEL);
		if (SafetyFlag != 0)
			break;

		//---------------------- 進入中斷 ----------------------
		MotionCard_ChangeTimer(Timer_ReadEncoder);

		// 每過1秒,紀錄於終端機上
		if (no_wait || clk.peekElapsedTime() > 1000000LL)
		{
			puts(get_joint_state_one_line(Pos, Vel, Unit::deg).c_str());
			no_wait = false;
			clk.start(); // Reset timer
		}
	}

	//-------------------- 關閉各項功能 --------------------
	MotionCard_ServoOff();	// Servo Off (拉煞車)
	MotionCard_CloseCard(); // 關閉軸卡
	record.CloseFile();		// 關閉實驗資料檔
	put_line(" All Functions Closed Successfully ");

	//-------------------- 顯示各軸狀態之錯誤訊息 --------------------
	if (SafetyFlag != 0)
	{
		put_line(IMC_ABORT_MESSAGE);
		print_joint_errmsg(Pos, Vel);
	}
	return 0;
}

void _stdcall Timer_ReadEncoder(TMRINT *pstINTSource)
{
	MotionCard_Encoder(Pos); 		// 更新機構角度 [rad]
	Toolbox_LSF(Pos, Vel);	 		// 更新機構速度 [rad/s]
	record.save(Pos, Vel, Acc); 	// 儲存實驗資料
}
