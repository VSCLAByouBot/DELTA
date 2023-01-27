//==================== �w�q��J/��X�ɮצW�� ====================
// ��X�ɦW��k�@: �۩w�q�����ɮ׸��|�ΦW��
// #define MY_OUTPUT_FILENAME "src\\delta\\data\\MyData_0125.txt" // ���ϥΤ�k�G,�бN�������

// ��X�ɦW��k�G: �ϥη�e�t�ήɶ��W�O�@���ɮצW�٫��(�бN��k�@���ѥH�ҥΤ�k�G)
#define OUTPUT_PATH "data\\"		// �w�q���|
#define OUTPUT_PREFIX "PTPCmdGen_"	// �w�q�e��

// �w�q��J�y���ɮ׸��|�ΦW��
#define INPUT_FILENAME "src\\delta\\data\\Trajectory.txt"

//==================== Integrated Functions ====================
#include "delta/Protection.h"
#include "delta/PTP.h"
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
const bool CHECKPOS = true; // ��ܬO�_�}�Ҧ�m�O�@�\��
const bool CHECKVEL = true; // ��ܬO�_�}�ҳt�׫O�@�\��

//======================= State & Counter & Flag =======================
int SafetyFlag = 0; // �b���A�O�@�X�� (0:�w��, ��L:�b���A���~�N�X)
int EndFlag1 = 0;	// �R�O�����X��1

//===============================================================
double InitialPos[AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // ��l���� [rad]
double TargetPos[AXIS] = {1.0, 1.0, -1.0, -1.0, 1.0, 1.0}; // �ؼШ��� [rad]

double Pos[AXIS] = {0.0};	 // �s�X������ [rad]
double Vel[AXIS] = {0.0};	 // LSF�t�� [rad/s]
double PosCmd[AXIS] = {0.0}; // ���שR�O [rad]
double VelCmd[AXIS] = {0.0}; // �t�שR�O [rad/s]
double AccCmd[AXIS] = {0.0}; // �[�t�שR�O [rad/s^2]

//========================== �إ�Ū�g�y�񤧹�� ==========================
SaveData record; // �x�s�y���T

int main(int argc, char *argv[])
{
	//-------------------- ��ܵ{�����z�P�ާ@����(�w�ƶ��q) --------------------
	put_hint("(DELTA) PTP TRAJECTORY GENERATION\n \n"
			 "Generate a trajectory file from point to point.");

	//-------------------- ��l�ƦU�b��m�P�t�פ��W�U�� --------------------
	Init_Joint_Bound(Pos_SF, Vel_SF, Unit::rad);

	//-------------------- ����I���I�B�ʤ���l�I�P�ؼ��I --------------------
	printf("InitialPos: %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f\n",
				InitialPos[0], InitialPos[1], InitialPos[2],
				InitialPos[3], InitialPos[4], InitialPos[5]);
	printf("TargetPos : %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f\n",
				TargetPos[0], TargetPos[1], TargetPos[2],
				TargetPos[3], TargetPos[4], TargetPos[5]);

	while (!_kbhit()) // �����N��}�l�{��
		;
	int sw = _getch(); // Get ASCII of the pressed key

	// �b�w�ƶ��q���UESC,�h�ߧY�����{��,��^���\���A
	CHECK_SUCCESS(sw == ESC_KEY, DO_NOTHING_MESSAGE);

	//---------------------- �إ߭y������� ----------------------
	CHECK_FAIL(record.CreateSaveData(__OUTPUT, __TIMESTAMP) != 0,
			   FILE_OPEN_ERR_MESSAGE);

	//---------------------- �u�@�y�{ ----------------------
	double _safety_prob = 0.0; // ��^���~�ƭ�
	put_line(" Launching Workflow ... ");
	while (sw != ESC_KEY)
	{
		//-------------------- ��ʵ����{�� --------------------
		if (_kbhit())
			sw = _getch(); // ��ESC�����{��

	//------------------------ �����I���I�R�O ------------------------
		PTP_Scurve(InitialPos, TargetPos, PosCmd, VelCmd, AccCmd, EndFlag1);

		//-------------------- ��m�t�׫O�@�{�� --------------------
		SafetyFlag = Check_Joint_State(PosCmd, VelCmd, _safety_prob, CHECKPOS, CHECKVEL);
		record.save(PosCmd, VelCmd, AccCmd); // �x�s������

		if (SafetyFlag != 0 || EndFlag1 != 0)
			break;
	}

	//-------------------- ��ܦU�b���A�����~�T�� --------------------
	if (SafetyFlag != 0)
	{
		put_line(IMC_ABORT_MESSAGE);
		print_joint_errmsg(Pos, Vel);
	}

	record.CloseFile(); // ������������
	put_line("PTP Generated Successfully!");
	return 0;
}
