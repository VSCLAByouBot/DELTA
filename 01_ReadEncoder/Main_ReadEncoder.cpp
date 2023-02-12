//==================== �w�q��J/��X�ɮצW�� ====================
// ��X�ɦW��k�@: �۩w�q�����ɮ׸��|�ΦW��
// #define MY_OUTPUT_FILENAME "src\\delta\\data\\MyData_0125.txt" // ���ϥΤ�k�G,�бN�������

// ��X�ɦW��k�G: �ϥη�e�t�ήɶ��W�O�@���ɮצW�٫��(�бN��k�@���ѥH�ҥΤ�k�G)
#define OUTPUT_PATH 	"data\\"			// �w�q���|
#define OUTPUT_PREFIX	"ReadEncMeasured_"	// �w�q�e��

//==================== Integrated Functions ====================
#include "delta/Initialization.h"
#include "delta/Protection.h"
#include "SaveData.h"
#include "error_handle.h"
#include "keyboard.h"
#include "box.h"
#include "clock.h"

//==================== �w�q�U�b��m�P�t�פ��w���Y��(0 ~ 1),�åB�b�sĶ���q�ˬd ====================
constexpr double Pos_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
constexpr double Vel_SF[AXIS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
// �b�sĶ�����ˬd�w���Y�ƪ��Ȱ�,�Y�W�L�h�sĶ����,���|�i����涥�q�~�ˬd
static_assert(!(CHK_SF(0) || CHK_SF(1) || CHK_SF(2) || CHK_SF(3) || CHK_SF(4) || CHK_SF(5)),
			  "Safety Factors (Pos_SF, Vel_SF) should all be 0 ~ 1.");
const bool CHECKPOS = true; // ��ܬO�_�}�Ҧ�m�O�@�\��
const bool CHECKVEL = true; // ��ܬO�_�}�ҳt�׫O�@�\��

//======================= Counter & Flag =======================
int SafetyFlag = 0; // �b���A�O�@�X�� (0:�w��, ��L:�b���A���~�N�X)
int TimerFlag = 0;	// ���_�����X��
int EndFlag1 = 0;	// �R�O�����X��1
int EndFlag2 = 0;	// �R�O�����X��2
int StayCount = 0;	// �w�ĥέp��

//===============================================================
double InitialPos[AXIS];	  // ��l���� [rad]
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
void _stdcall Timer_ReadEncoder(TMRINT *pstINTSource);

//========================== �إ�Ū�g�y�񤧹�� ==========================
SaveData record;	// �x�s�y���T��

int main(int argc, char *argv[])
{
	//-------------------- ��ܵ{�����z�P�ާ@����(�w�ƶ��q) --------------------
	put_hint("(DELTA) READ ENCODER\n"
			 "Show All Axes Position, Velocity, Acceleration(TODO: derivatives)");

	//-------------------- ��l�ƦU�b��m�P�t�פ��W�U�� --------------------
	Init_Joint_Bound(Pos_SF, Vel_SF, CHECKPOS, CHECKVEL, Unit::rad);

	while (!_kbhit()) // �����N��}�l�{��
		;
	int sw = _getch(); // Get ASCII of the pressed key

	// �b�w�ƶ��q���UESC,�h�ߧY�����{��,��^���\���A
	CHECK_SUCCESS(sw == ESC_KEY, DO_NOTHING_MESSAGE);

	//---------------------- Ū����� ----------------------
	CHECK_FAIL(record.CreateSaveData(__OUTPUT, __TIMESTAMP) != 0,
		FILE_OPEN_ERR_MESSAGE); // �إ߹�������

	//-------------------- �}�ҦU���\�� --------------------
	CHECK_FAIL(MotionCard_OpenCard() != 0, MC_ERR_MESSAGE); // �}�Ҷb�d,�Y���ѫh�����{��
	Init_PosCmd_LSF(PosCmd);								// ����l���שR�O & ��sLSF

	//---------------------- �u�@�y�{ ----------------------
	Clock clk;				   // �p�ɾ�,�Ψӱ���ƾ���ܦb�ù��W���W�v
	double _safety_prob = 0.0; // ��^���~�ƭ�
	bool no_wait = true;	   // �@�i�J�u�@�y�{,�N������ܦ�m�P�t�׭�

	put_line(" Launching Workflow ... ");
	clk.start();
	while (sw != ESC_KEY)
	{
		//-------------------- ��ʵ����{�� --------------------
		if (_kbhit())
			sw = _getch(); // ��ESC�����{��

		//-------------------- ��m�t�׫O�@�{�� --------------------
		SafetyFlag = Check_Joint_State(Pos, Vel, _safety_prob, CHECKPOS, CHECKVEL);
		if (SafetyFlag != 0)
			break;

		//---------------------- �i�J���_ ----------------------
		MotionCard_ChangeTimer(Timer_ReadEncoder);

		// �C�L1��,������׺ݾ��W
		if (no_wait || clk.peekElapsedTime() > 1000000LL)
		{
			puts(get_joint_state_one_line(Pos, Vel, Unit::deg).c_str());
			no_wait = false;
			clk.start(); // Reset timer
		}
	}

	//-------------------- �����U���\�� --------------------
	MotionCard_ServoOff();	// Servo Off (�Է٨�)
	MotionCard_CloseCard(); // �����b�d
	record.CloseFile();		// ������������
	put_line(" All Functions Closed Successfully ");

	//-------------------- ��ܦU�b���A�����~�T�� --------------------
	if (SafetyFlag != 0)
	{
		put_line(IMC_ABORT_MESSAGE);
		print_joint_errmsg(Pos, Vel);
	}
	return 0;
}

void _stdcall Timer_ReadEncoder(TMRINT *pstINTSource)
{
	MotionCard_Encoder(Pos); 		// ��s���c���� [rad]
	Toolbox_LSF(Pos, Vel);	 		// ��s���c�t�� [rad/s]
	record.save(Pos, Vel, Acc); 	// �x�s������
}
