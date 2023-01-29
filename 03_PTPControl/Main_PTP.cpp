//==================== 定義輸入/輸出檔案名稱 ====================
// 輸出檔名方法一: 自定義完整檔案路徑及名稱
// #define MY_OUTPUT_FILENAME "src\\delta\\data\\MyData_0125.txt" // 欲使用方法二,請將此行註解

// 輸出檔名方法二: 使用當前系統時間戳記作為檔案名稱後綴(請將方法一註解以啟用方法二)
#define OUTPUT_PATH "data\\" 			// 定義路徑
#define OUTPUT_PREFIX "PTPMeasured_"	// 定義前綴

//==================== Integrated Functions ====================
#include "delta/Initialization.h"
#include "delta/Control.h"
#include "delta/Protection.h"
#include "delta/PTP.h"
// #include "delta/RobotModel.h"
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
const int STATE_BUFFER1 = 0;
const int STATE_PTPtoTargetPos = 1;
const int STATE_BUFFER2 = 2;
const int STATE_PTPtoInitialPos = 3;
const int STATE_BUFFER3 = 4;
const int STATE_END = 5;

int SafetyFlag = 0;			   // 軸狀態保護旗標 (0:安全, 其他:軸狀態錯誤代碼)
int TimerFlag = STATE_BUFFER1; // 中斷切換旗標
int EndFlag1 = 0;			   // 命令結束旗標1
int EndFlag2 = 0;			   // 命令結束旗標2
int StayCount = 0;			   // 緩衝用計數

//===============================================================
double HomePos[AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};	   // HOME點 [rad]
double InitialPos[AXIS];								   // 初始角度 [rad]
double TargetPos[AXIS] = {1.0, 1.0, -1.0, -1.0, 1.0, 1.0}; // 目標角度 [rad]
 // double TargetPos[AXIS] = {1.0, 0.5, -0.5, 0.0, 0.0, 0.0}; // 目標角度 [rad]

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
void _stdcall Timer_PTPtoTargetPos(TMRINT *pstINTSource);
void _stdcall Timer_PTPtoInitialPos(TMRINT *pstINTSource);
void _stdcall Timer_Stay(TMRINT *pstINTSource);

//========================== 建立讀寫軌跡之實例 ==========================
SaveData record; // 儲存軌跡資訊檔

int main(int argc, char *argv[])
{
	//-------------------- 顯示程式概述與操作提示(預備階段) --------------------
	put_hint("(DELTA) POINT TO POINT\n \n"
			 "Specify a target EEF point to reach the goal and go back to HOME.");

	//-------------------- 初始化各軸位置與速度之上下限 --------------------
	Init_Joint_Bound(Pos_SF, Vel_SF, CHECKPOS, CHECKVEL, Unit::rad);

	//-------------------- 顯示點對點運動之初始點與目標點 --------------------
	printf("InitialPos: [%+8.5f, %+8.5f, %+8.5f, %+8.5f, %+8.5f, %+8.5f]\n",
		   InitialPos[0], InitialPos[1], InitialPos[2],
		   InitialPos[3], InitialPos[4], InitialPos[5]);
	printf("TargetPos : [%+8.5f, %+8.5f, %+8.5f, %+8.5f, %+8.5f, %+8.5f]\n",
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
	// RobotModel_Beta( "Beta.txt" ) ; // 載入系統參數

	//-------------------- 開啟各項功能 --------------------
	CHECK_FAIL(MotionCard_OpenCard() != 0, MC_ERR_MESSAGE); // 開啟軸卡,若失敗則結束程式
	Init_PosCmd_LSF(PosCmd);								// 給初始角度命令 & 更新LSF

	//---------------------- 工作流程 ----------------------
	double _safety_prob = 0.0; // 返回錯誤數值
	put_line(" Launching Workflow ... ");
	MotionCard_ServoOn(); // Servo On
	while (sw != ESC_KEY)
	{
		//-------------------- 手動結束程式 --------------------
		if (_kbhit())
			sw = _getch(); // 按ESC結束程式

		//-------------------- 位置速度保護程式 --------------------
		SafetyFlag = Check_Joint_State(Pos, Vel, _safety_prob, CHECKPOS, CHECKVEL);
		if (SafetyFlag != 0)
			break; // 結束程式

		//---------------------- 切換中斷 ----------------------
		if (TimerFlag == STATE_BUFFER1)
			MotionCard_ChangeTimer(Timer_Stay); // 緩衝1
		else if (TimerFlag == STATE_PTPtoTargetPos)
			MotionCard_ChangeTimer(Timer_PTPtoTargetPos); // Step1. PTP到目標點
		else if (TimerFlag == STATE_BUFFER2)
			MotionCard_ChangeTimer(Timer_Stay); // 緩衝2
		else if (TimerFlag == STATE_PTPtoInitialPos)
			MotionCard_ChangeTimer(Timer_PTPtoInitialPos); // Step2. PTP回初始點
		else if (TimerFlag == STATE_BUFFER3)
			MotionCard_ChangeTimer(Timer_Stay); // 緩衝3
		else if (TimerFlag == STATE_END)
			break; // 結束程式
	}

	//-------------------- 關閉各項功能 --------------------
	MotionCard_ServoOff();	// Servo Off (拉煞車)
	MotionCard_CloseCard(); // 關閉軸卡

	//-------------------- 顯示各軸狀態之錯誤訊息 --------------------
	if (SafetyFlag != 0)
	{
		record.save(Pos, Vel, PosCmd, VelCmd, TorCtrl); // 錯誤狀態也要記錄
		put_line(IMC_ABORT_MESSAGE);
		print_joint_errmsg(Pos, Vel);
	}
	record.CloseFile(); // 關閉實驗資料檔
	put_line(" All Functions Closed Successfully ");
	return 0;
}

//====================== Step1. PTP到目標點 ======================
void _stdcall Timer_PTPtoTargetPos(TMRINT *pstINTSource)
{
	//---------------------- 讀取角度、速度 ----------------------
	MotionCard_Encoder(Pos); // 更新機構角度 [rad]
	Toolbox_LSF(Pos, Vel);	 // 更新機構速度 [rad/s]

	// Protection( Vel , SafetyFlag ) ; // 速度保護

	//------------------------ 產生點對點命令 ------------------------
	PTP_Scurve(InitialPos, TargetPos, PosCmd, VelCmd, AccCmd, EndFlag1);

	Control_Feedback(Pos, Vel, PosCmd, TorCtrl); // 回授控制

	MotionCard_DAC(TorCtrl); // 送出控制命令

	// RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	// Toolbox_Filter( TorF , TorF_f ) ;

	//------------------------- 切換中斷 -------------------------
	if (EndFlag1 == 1)
	{
		TimerFlag = STATE_BUFFER2;
	}

	//----------------------- 儲存實驗資料 -----------------------
	if (TimerFlag == STATE_PTPtoTargetPos)
	{
		record.save(Pos, Vel, PosCmd, VelCmd, TorCtrl);
	}
}

//======================= Step2. PTP回初始點 =======================
void _stdcall Timer_PTPtoInitialPos(TMRINT *pstINTSource)
{
	//---------------------- 讀取角度、速度 ----------------------
	MotionCard_Encoder(Pos); // 更新機構角度 [rad]
	Toolbox_LSF(Pos, Vel);	 // 更新機構速度 [rad/s]

	// Protection( Vel , SafetyFlag ) ; // 速度保護

	//------------------------ 產生點對點命令 ------------------------
	PTP_Scurve(TargetPos, HomePos, PosCmd, VelCmd, AccCmd, EndFlag2);

	Control_Feedback(Pos, Vel, PosCmd, TorCtrl); // 回授控制

	MotionCard_DAC(TorCtrl); // 送出控制命令

	// RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	// Toolbox_Filter( TorF , TorF_f ) ;

	//----------------------- 儲存實驗資料 -----------------------
	if (TimerFlag == STATE_PTPtoInitialPos)
	{
		record.save(Pos, Vel, PosCmd, VelCmd, TorCtrl);
	}

	//------------------------- 切換中斷 -------------------------
	if (EndFlag2 == 1)
	{
		TimerFlag = STATE_BUFFER3;
	}
}

//=========================== 緩衝程式 ===========================
void _stdcall Timer_Stay(TMRINT *pstINTSource)
{
	//---------------------- 讀取角度、速度 ----------------------
	MotionCard_Encoder(Pos); // 更新機構角度 [rad]
	Toolbox_LSF(Pos, Vel);	 // 更新機構速度 [rad/s]

	// Protection( Vel , SafetyFlag ) ; // 速度保護

	//--------------------------- 控制 ---------------------------
	Control_Feedback(Pos, Vel, PosCmd, TorCtrl); // 回授控制
	
	MotionCard_DAC(TorCtrl); // 送出控制命令

	//------------------------- 切換中斷 -------------------------
	if (TimerFlag == STATE_BUFFER1 && StayCount >= 1000) // 緩衝1
	{
		/* 此時的初始 PosCmd 為 Servo ON 前的值,
		 * 若無重力補償,則第三軸可能受到重力影響,
		 * 造成 InitialPos 與初始 PosCmd 有偏差
		 */
		TimerFlag = STATE_PTPtoTargetPos;
		for (int i = 0; i < AXIS; i++)
		{
			InitialPos[i] = Pos[i]; // 初始角度(Step1.PTP)
		}
		puts("---------- 緩衝1 結束 ----------");
	}
	else if (TimerFlag == STATE_BUFFER2 && StayCount >= 2000) // 緩衝2
	{
		TimerFlag = STATE_PTPtoInitialPos;
		puts("---------- 緩衝2 結束 ----------");
	}
	else if (TimerFlag == STATE_BUFFER3 && StayCount >= 3000) // 緩衝3
	{
		TimerFlag = STATE_END;
		puts("---------- 緩衝3 結束 ----------");
	}

	record.save(Pos, Vel, PosCmd, VelCmd, TorCtrl);
	StayCount++; // 軸卡切換中斷之副程式執行週期為 1 ms
}
