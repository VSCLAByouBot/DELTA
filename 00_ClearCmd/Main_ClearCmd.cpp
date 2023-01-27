//==================== Integrated Functions ====================
#include "delta/MotionCard.h"
#include "error_handle.h"
#include "keyboard.h"
#include "box.h"
#include "clock.h"

int main(int argc, char *argv[])
{
	//-------------------- ��ܵ{�����z�P�ާ@����(�w�ƶ��q) --------------------
	put_hint("(DELTA) MOTION CARD CLEAR COMMAND");

	while (!_kbhit()) // �����N��}�l�{��
		;
	int sw = _getch(); // Get ASCII of the pressed key

	// �b�w�ƶ��q���UESC,�h�ߧY�����{��,��^���\���A
	CHECK_SUCCESS(sw == ESC_KEY, DO_NOTHING_MESSAGE);

	//-------------------- �}�ҦU���\�� --------------------
	CHECK_FAIL(MotionCard_OpenCard(CLEAR_CMD) != 0, MC_ERR_MESSAGE); // �}�Ҷb�d,�Y���ѫh�����{��

	//---------------------- �u�@�y�{ ----------------------
	put_line(" Launching Workflow ... ");
	put_line(" Waiting for 1 sec ... ");
	Clock clk;
	clk.start();
	while (sw != ESC_KEY)
	{
		//-------------------- ��ʵ����{�� --------------------
		if (_kbhit())
			sw = _getch(); // ��ESC�����{��

		//----------------- ���ݶW�L1�������� -----------------
		if (clk.peekElapsedTime() > 1000000LL)
			break;
	}

	//-------------------- �����U���\�� --------------------
	MotionCard_ServoOff();	// Servo Off (�Է٨�)
	MotionCard_CloseCard(); // �����b�d
	put_line(" Successfully Clear Command ");
	return 0;
}
