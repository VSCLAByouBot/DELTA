//==================== �w�q��J/��X�ɮצW�� ====================
// ��X�ɦW��k�@: �۩w�q�����ɮ׸��|�ΦW��
// #define MY_OUTPUT_FILENAME "data\\MyData_0125.txt" // ���ϥΤ�k�G,�бN�������

// ��X�ɦW��k�G: �ϥη�e�t�ήɶ��W�O�@���ɮצW�٫��(�бN��k�@���ѥH�ҥΤ�k�G)
#define OUTPUT_PATH "data\\" 			// �w�q���|
#define OUTPUT_PREFIX "TrackMeasured_"	// �w�q�e��

// �w�q��J�y���ɮ׸��|�ΦW��
#define INPUT_FILENAME "data\\PTP_test_0.5_J1.txt" // testing sequence 1
// #define INPUT_FILENAME "data\\PTP_test_0.5_all.txt" // testing sequence 2
// #define INPUT_FILENAME "data\\PTP_test_1.0_J1.txt"	// testing sequence 3
// #define INPUT_FILENAME "data\\PTP_test_1.0_all.txt" // testing sequence 4
// #define INPUT_FILENAME "data\\PTP_test_2.9_J1.txt"	// testing sequence 5
// #define INPUT_FILENAME "data\\Trajectory.txt"		// testing sequence 6

//==================== Integrated Functions ====================
#include "delta/Initialization.h"
#include "delta/Control.h"
#include "delta/Protection.h"
#include "delta/PTP.h"
// #include "delta/RobotModel.h"
#include "delta/Tracking.h"
#include "SaveData.h"
#include "error_handle.h"
#include "keyboard.h"
#include "box.h"

//==================== �w�q�U�b��m�P�t�פ��w���Y��(0 ~ 1),�åB�b�sĶ���q�ˬd ====================
constexpr double Pos_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
constexpr double Vel_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
// �b�sĶ�����ˬd�w���Y�ƪ��Ȱ�,�Y�W�L�h�sĶ����,���|�i����涥�q�~�ˬd
static_assert(!(CHK_SF(0) || CHK_SF(1) || CHK_SF(2) || CHK_SF(3) || CHK_SF(4) || CHK_SF(5)),
			  "Safety Factors (Pos_SF, Vel_SF) should all be 0 ~ 1.");
const bool CHECKPOS = true;	 // ��ܬO�_�}�Ҧ�m�O�@�\��
const bool CHECKVEL = false; // ��ܬO�_�}�ҳt�׫O�@�\��

//======================= State & Counter & Flag =======================
const int STATE_BUFFER1 = 0;
const int STATE_PTPtoFirstPos = 1;
const int STATE_BUFFER2 = 2;
const int STATE_TRACKING = 3;
const int STATE_BUFFER3 = 4;
const int STATE_PTPtoHomePos = 5;
const int STATE_BUFFER4 = 6;
const int STATE_END = 7;

int SafetyFlag = 0;			   // �b���A�O�@�X�� (0:�w��, ��L:�b���A���~�N�X)
int TimerFlag = STATE_BUFFER1; // ���_�����X��
int EndFlag1 = 0;			   // �R�O�����X��1
int EndFlag2 = 0;			   // �R�O�����X��2
int EndFlag3 = 0;			   // �R�O�����X��3
int StayCount = 0;			   // �w�ĥέp��

//===============================================================
double HomePos[AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Home�I [rad]
double InitialPos[AXIS];							   // ��l���� [rad]
double FirstPos[AXIS];								   // �Ĥ@���y��R�O [rad]
double LastPos[AXIS];								   // �̥����y��R�O [rad]

double Pos[AXIS] = {0.0};	  // �s�X������ [rad]
double Vel[AXIS] = {0.0};	  // LSF�t�� [rad/s]
double Acc[AXIS] = {0.0};	  // TODO: �ݥ[�t�צ������� [rad/s^2]
double PosCmd[AXIS] = {0.0};  // ���שR�O [rad]
double VelCmd[AXIS] = {0.0};  // �t�שR�O [rad/s]
double AccCmd[AXIS] = {0.0};  // �[�t�שR�O [rad/s^2]
double TorCtrl[AXIS] = {0.0}; // �b��x����T��(��t�e) [Nm]

// double TorM[AXIS] = {0.0};	 // �e�X�D�ʤO
// double TorN[AXIS] = {0.0};	 // �e�X���O&���O
// double TorF[AXIS] = {0.0};	 // �e�X�����O
// double TorF_f[AXIS] = {0.0}; // �o�i��e�X�����O
// double TorFF[AXIS] = {0.0};	 // �`�e�X��x

//================== �ŧi�b�d�����_�Ƶ{�� ==================
void _stdcall Timer_PTPtoFirstPos(TMRINT *pstINTSource);
void _stdcall Timer_Tracking(TMRINT *pstINTSource);
void _stdcall Timer_PTPtoHomePos(TMRINT *pstINTSource);
void _stdcall Timer_Stay(TMRINT *pstINTSource);

//========================== �إ�Ū�g�y�񤧹�� ==========================
SaveData record; // �x�s�y���T��
Track track;	 // Ū����J�y����

int main(int argc, char *argv[])
{
	//-------------------- ��ܵ{�����z�P�ާ@����(�w�ƶ��q) --------------------
	put_hint("(DELTA) TRACKING\n \n"
			 "Follow the trajectory defined in the input file and go back to HOME.");

	//-------------------- ��l�ƦU�b��m�P�t�פ��W�U�� --------------------
	Init_Joint_Bound(Pos_SF, Vel_SF, CHECKPOS, CHECKVEL, Unit::rad);
	puts("Input Trajectory File: \"" INPUT_FILENAME "\"");

	while (!_kbhit()) // �����N��}�l�{��
		;
	int sw = _getch(); // Get ASCII of the pressed key

	// �b�w�ƶ��q���UESC,�h�ߧY�����{��,��^���\���A
	CHECK_SUCCESS(sw == ESC_KEY, DO_NOTHING_MESSAGE);

	//---------------------- Ū���y������ ----------------------
	// �}�ҭy��R�O��,�ˬd����y����->Ū���Ĥ@���y��R�O(for Step1.PTP) ~ �ݦb�e!
	CHECK_FAIL(track.TrackingFile_Init(INPUT_FILENAME, FirstPos) != 0,
			   TRCK_FILE_INIT_ERR_MESSAGE);

	//---------------------- �إ߭y������� ----------------------
	CHECK_FAIL(record.CreateSaveData(__OUTPUT, __TIMESTAMP) != 0,
			   FILE_OPEN_ERR_MESSAGE);
	// RobotModel_Beta("Beta.txt") ; // ���J�t�ΰѼ�

	//-------------------- �}�ҦU���\�� --------------------
	CHECK_FAIL(MotionCard_OpenCard() != 0, MC_ERR_MESSAGE); // �}�Ҷb�d,�Y���ѫh�����{��
	Init_PosCmd_LSF(PosCmd);								// ����l���שR�O & ��sLSF

	//---------------------- �u�@�y�{ ----------------------
	double _safety_prob = 0.0; // ��^���~�ƭ�
	put_line(" Launching Workflow ... ");
	MotionCard_ServoOn(); // Servo On
	while (sw != ESC_KEY)
	{
		//-------------------- ��ʵ����{�� --------------------
		if (_kbhit())
			sw = _getch(); // ��ESC�����{��

		//-------------------- ��m�t�׫O�@�{�� --------------------
		SafetyFlag = Check_Joint_State(Pos, Vel, _safety_prob, CHECKPOS, CHECKVEL);
		if (SafetyFlag != 0)
			break; // �����{��

		//---------------------- �������_ ----------------------
		if (TimerFlag == STATE_BUFFER1)
			MotionCard_ChangeTimer(Timer_Stay); // �w��1
		else if (TimerFlag == STATE_PTPtoFirstPos)
			MotionCard_ChangeTimer(Timer_PTPtoFirstPos); // Step1. PTP���ܭy���l����
		else if (TimerFlag == STATE_BUFFER2)
			MotionCard_ChangeTimer(Timer_Stay); // �w��2
		else if (TimerFlag == STATE_TRACKING)
			MotionCard_ChangeTimer(Timer_Tracking); // Step2. �`�񱱨�
		else if (TimerFlag == STATE_BUFFER3)
			MotionCard_ChangeTimer(Timer_Stay); // �w��3
		else if (TimerFlag == STATE_PTPtoHomePos)
			MotionCard_ChangeTimer(Timer_PTPtoHomePos); // Step3. PTP���^Home�I
		else if (TimerFlag == STATE_BUFFER4)
			MotionCard_ChangeTimer(Timer_Stay); // �w��4
		else if (TimerFlag == STATE_END)
			break; // �����{��
	}

	//-------------------- �����U���\�� --------------------
	MotionCard_ServoOff();	// Servo Off (�Է٨�)
	MotionCard_CloseCard(); // �����b�d

	//-------------------- ��ܦU�b���A�����~�T�� --------------------
	if (SafetyFlag != 0)
	{
		record.save(Pos, Vel, PosCmd, VelCmd, TorCtrl); // ���~���A�]�n�O��
		put_line(IMC_ABORT_MESSAGE);
		print_joint_errmsg(Pos, Vel);
	}
	record.CloseFile(); // ������������
	track.CloseFile();	// �����y��R�O��
	put_line(" All Functions Closed Successfully ");
	return 0;
}

//====================== Step1. PTP��Ĥ@���y���I ======================
void _stdcall Timer_PTPtoFirstPos(TMRINT *pstINTSource)
{
	//---------------------- Ū�����סB�t�� ----------------------
	MotionCard_Encoder(Pos); // ��s���c���� [rad]
	Toolbox_LSF(Pos, Vel);	 // ��s���c�t�� [rad/s]

	//------------------------ �����I���I�R�O ------------------------
	PTP_Scurve(InitialPos, FirstPos, PosCmd, VelCmd, AccCmd, EndFlag1);

	Control_Feedback(Pos, Vel, PosCmd, TorCtrl); // �^�±���

	MotionCard_DAC(TorCtrl); // �e�X����R�O

	//------------------------- �������_ -------------------------
	if (EndFlag1 == 1)
	{
		TimerFlag = STATE_BUFFER2;
	}
}

//====================== Step 2. �`�񱱨�  ======================
void _stdcall Timer_Tracking(TMRINT *pstINTSource)
{
	//---------------------- Ū�����סB�t�� ----------------------
	MotionCard_Encoder(Pos); // ��s���c���� [rad]
	Toolbox_LSF(Pos, Vel);	 // ��s���c�t�� [rad/s]

	//------------------------- �`�񱱨� -------------------------
	track.Get_Cmd(PosCmd, VelCmd, AccCmd, EndFlag2); // �`��R�O

	Control_Feedback(Pos, Vel, PosCmd, TorCtrl); // �^�±���

	MotionCard_DAC(TorCtrl); // �e�X����R�O

	//------------------------- �������_ -------------------------
	if (EndFlag2 == 1)
	{
		TimerFlag = STATE_BUFFER3;
	}

	//------------------------- �x�s������ -------------------------
	if (TimerFlag == STATE_TRACKING) // �h�X�Ӫ���Ƥ��ά���
	{
		record.save(Pos, Vel, TorCtrl); // �x�s������
	}
}

//==================== Step 3. PTP���^Home�I ====================
void _stdcall Timer_PTPtoHomePos(TMRINT *pstINTSource)
{
	//---------------------- Ū�����סB�t�� ----------------------
	MotionCard_Encoder(Pos); // ��s���c���� [rad]
	Toolbox_LSF(Pos, Vel);	 // ��s���c�t�� [rad/s]

	//------------------------ �����I���I�R�O ------------------------
	PTP_Scurve(LastPos, HomePos, PosCmd, VelCmd, AccCmd, EndFlag3);

	Control_Feedback(Pos, Vel, PosCmd, TorCtrl); // �^�±���

	MotionCard_DAC(TorCtrl); // �e�X����R�O

	//------------------------- �������_ -------------------------
	if (EndFlag3 == 1)
	{
		TimerFlag = STATE_BUFFER4;
	}
}

//=========================== �w�ĵ{�� ===========================
void _stdcall Timer_Stay(TMRINT *pstINTSource)
{
	//---------------------- Ū�����סB�t�� ----------------------
	MotionCard_Encoder(Pos); // ��s���c���� [rad]
	Toolbox_LSF(Pos, Vel);	 // ��s���c�t�� [rad/s]

	//--------------------------- ���� ---------------------------
	Control_Feedback(Pos, Vel, PosCmd, TorCtrl); // �^�±���

	MotionCard_DAC(TorCtrl); // �e�X����R�O

	//------------------------- �������_ -------------------------
	if (TimerFlag == STATE_BUFFER1 && StayCount >= 1000) // �w��1
	{
		/* ���ɪ���l PosCmd �� Servo ON �e����,
		 * �Y�L���O���v,�h�ĤT�b�i����쭫�O�v�T,
		 * �y�� InitialPos �P��l PosCmd �����t
		 */
		TimerFlag = STATE_PTPtoFirstPos;
		for (int i = 0; i < AXIS; i++)
		{
			//InitialPos[i] = Pos[i]; // ��l����(Step1.PTP) (2023/01/29�H�e)

			/* 2023/01/29 ���յ��G
			 * �ѩ�Pos�PPosCmd�������M�s�bí�A�~�t,
			 * �Y�������ܩR�O(PosCmd -> Pos),�i��|�y�����u�����_��
			 */
			InitialPos[i] = PosCmd[i]; // ��l����(Step1.PTP)
		}
		puts("---------- �w��1 ���� ----------");
	}
	else if (TimerFlag == STATE_BUFFER2 && StayCount >= 2000) // �w��2
	{
		TimerFlag = STATE_TRACKING;
		puts("---------- �w��2 ���� ----------");
	}
	else if (TimerFlag == STATE_BUFFER3 && StayCount >= 3000) // �w��3
	{
		TimerFlag = STATE_PTPtoHomePos;
		for (int i = 0; i < AXIS; i++)
		{
			//LastPos[i] = Pos[i]; // �̥����y��R�O(Step3.PTP) (2023/01/29�H�e)

			/* 2023/01/29 ���յ��G
			 * �ѩ�Pos�PPosCmd�������M�s�bí�A�~�t,
			 * �Y�������ܩR�O(PosCmd -> Pos),�i��|�y�����u�����_��
			 */
			LastPos[i] = PosCmd[i]; // �̥����y��R�O(Step3.PTP)
		}
		puts("---------- �w��3 ���� ----------");
	}
	else if (TimerFlag == STATE_BUFFER4 && StayCount >= 4000) // �w��4
	{
		TimerFlag = STATE_END;
		puts("---------- �w��4 ���� ----------");
	}
	StayCount++; // �b�d�������_���Ƶ{������g���� 1 ms
}
