//==================== Integrated Functions ====================
#include "delta/MotionCard.h"
#include "error_handle.h"
#include "keyboard.h"
#include "box.h"
#include "clock.h"

int main(int argc, char *argv[])
{
	//-------------------- 顯示程式概述與操作提示(預備階段) --------------------
	put_hint("(DELTA) MOTION CARD CLEAR COMMAND");

	while (!_kbhit()) // 按任意鍵開始程式
		;
	int sw = _getch(); // Get ASCII of the pressed key

	// 在預備階段按下ESC,則立即關閉程式,返回成功狀態
	CHECK_SUCCESS(sw == ESC_KEY, DO_NOTHING_MESSAGE);

	//-------------------- 開啟各項功能 --------------------
	CHECK_FAIL(MotionCard_OpenCard(CLEAR_CMD) != 0, MC_ERR_MESSAGE); // 開啟軸卡,若失敗則結束程式

	//---------------------- 工作流程 ----------------------
	put_line(" Launching Workflow ... ");
	put_line(" Waiting for 1 sec ... ");
	Clock clk;
	clk.start();
	while (sw != ESC_KEY)
	{
		//-------------------- 手動結束程式 --------------------
		if (_kbhit())
			sw = _getch(); // 按ESC結束程式

		//----------------- 等待超過1秒直接關閉 -----------------
		if (clk.peekElapsedTime() > 1000000LL)
			break;
	}

	//-------------------- 關閉各項功能 --------------------
	MotionCard_ServoOff();	// Servo Off (拉煞車)
	MotionCard_CloseCard(); // 關閉軸卡
	put_line(" Successfully Clear Command ");
	return 0;
}
