//==================== 定義輸入/輸出檔案名稱 ====================
// 輸出檔名方法一: 自定義完整檔案路徑及名稱
// #define MY_OUTPUT_FILENAME "src\\delta\\data\\MyData_0125.txt" // 欲使用方法二,請將此行註解

// 輸出檔名方法二: 使用當前系統時間戳記作為檔案名稱後綴(請將方法一註解以啟用方法二)
#define OUTPUT_PATH "data\\"		// 定義路徑
#define OUTPUT_PREFIX "PTPCmdGen_"	// 定義前綴

// 定義輸入軌跡檔案路徑及名稱
#define INPUT_FILENAME "src\\delta\\data\\Trajectory.txt"

//==================== Integrated Functions ====================
#include "delta/Protection.h"
#include "delta/PTP.h"
#include "SaveData.h"
#include "error_handle.h"
#include "keyboard.h"
#include "box.h"

//==================== 定義各軸位置與速度之安全係數(0 ~ 1),並且在編譯階段檢查 ====================
constexpr double Pos_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
constexpr double Vel_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
// 在編譯期間檢查安全係數的值域,若超過則編譯失敗,不會進到執行階段才檢查
static_assert(!(CHK_SF(0) || CHK_SF(1) || CHK_SF(2) || CHK_SF(3) || CHK_SF(4) || CHK_SF(5)),
			  "Safety Factors (Pos_SF, Vel_SF) should all be 0 ~ 1.");
const bool CHECKPOS = true; // 選擇是否開啟位置保護功能
const bool CHECKVEL = true; // 選擇是否開啟速度保護功能

//======================= State & Counter & Flag =======================
int SafetyFlag = 0; // 軸狀態保護旗標 (0:安全, 其他:軸狀態錯誤代碼)
int EndFlag1 = 0;	// 命令結束旗標1

//===============================================================
double InitialPos[AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 初始角度 [rad]
double TargetPos[AXIS] = {1.0, 1.0, -1.0, -1.0, 1.0, 1.0}; // 目標角度 [rad]

double Pos[AXIS] = {0.0};	 // 編碼器角度 [rad]
double Vel[AXIS] = {0.0};	 // LSF速度 [rad/s]
double PosCmd[AXIS] = {0.0}; // 角度命令 [rad]
double VelCmd[AXIS] = {0.0}; // 速度命令 [rad/s]
double AccCmd[AXIS] = {0.0}; // 加速度命令 [rad/s^2]

//========================== 建立讀寫軌跡之實例 ==========================
SaveData record; // 儲存軌跡資訊

int main(int argc, char *argv[])
{
	//-------------------- 顯示程式概述與操作提示(預備階段) --------------------
	put_hint("(DELTA) PTP TRAJECTORY GENERATION\n \n"
			 "Generate a trajectory file from point to point.");

	//-------------------- 初始化各軸位置與速度之上下限 --------------------
	Init_Joint_Bound(Pos_SF, Vel_SF, Unit::rad);

	//-------------------- 顯示點對點運動之初始點與目標點 --------------------
	printf("InitialPos: %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f\n",
				InitialPos[0], InitialPos[1], InitialPos[2],
				InitialPos[3], InitialPos[4], InitialPos[5]);
	printf("TargetPos : %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f\n",
				TargetPos[0], TargetPos[1], TargetPos[2],
				TargetPos[3], TargetPos[4], TargetPos[5]);

	while (!_kbhit()) // 按任意鍵開始程式
		;
	int sw = _getch(); // Get ASCII of the pressed key

	// 在預備階段按下ESC,則立即關閉程式,返回成功狀態
	CHECK_SUCCESS(sw == ESC_KEY, DO_NOTHING_MESSAGE);

	//---------------------- 建立軌跡紀錄檔 ----------------------
	CHECK_FAIL(record.CreateSaveData(__OUTPUT, __TIMESTAMP) != 0,
			   FILE_OPEN_ERR_MESSAGE);

	//---------------------- 工作流程 ----------------------
	double _safety_prob = 0.0; // 返回錯誤數值
	put_line(" Launching Workflow ... ");
	while (sw != ESC_KEY)
	{
		//-------------------- 手動結束程式 --------------------
		if (_kbhit())
			sw = _getch(); // 按ESC結束程式

	//------------------------ 產生點對點命令 ------------------------
		PTP_Scurve(InitialPos, TargetPos, PosCmd, VelCmd, AccCmd, EndFlag1);

		//-------------------- 位置速度保護程式 --------------------
		SafetyFlag = Check_Joint_State(PosCmd, VelCmd, _safety_prob, CHECKPOS, CHECKVEL);
		record.save(PosCmd, VelCmd, AccCmd); // 儲存實驗資料

		if (SafetyFlag != 0 || EndFlag1 != 0)
			break;
	}

	//-------------------- 顯示各軸狀態之錯誤訊息 --------------------
	if (SafetyFlag != 0)
	{
		put_line(IMC_ABORT_MESSAGE);
		print_joint_errmsg(Pos, Vel);
	}

	record.CloseFile(); // 關閉實驗資料檔
	put_line("PTP Generated Successfully!");
	return 0;
}
