#ifndef TRACKING_H
#define TRACKING_H

#include <iostream>
#include <vector>
#include "Safety_IO.h"
#include "delta/Setting.h"
#include "delta/Protection.h"

#define FMT_NUM_ERROR -1
#define TRACK_FILE_OPEN_ERROR 1

class Track
{
public:
    Track();

    /**
	 * @brief �}�ҭy��R�O��,�ˬd����y��R�O��,��Ū���Ĥ@���y��R�O(for Step1.PTP)
     * @param path: �y��R�O�ɮצW��
     * @param FirstPos: �O���Ĥ@���y���m�R�O
     * @return ���\�}�Ҧ^��0,���ѫh�^�ǫD0
     */
    int TrackingFile_Init(const char *path, double (&FirstPos)[AXIS]);

    /**
	 * @brief �}�ҭy��R�O��
     * @param path: �y��R�O�ɮצW��
     * @return ���\�}�Ҧ^��0,���ѫh�^�ǫD0
     */
    int Tracking_OpenFile(const char *path);

    /**
	 * @brief Ū���y��R�O��(slow, file pointer method)
     * @param PosCmd: �O���y���m�R�O
     * @param VelCmd: �O���y��t�שR�O
     * @param AccCmd: �O���y��[�t�שR�O
     * @param EndFlag: �y��R�O�����X��
     */
    void Get_Cmd_old(double (&PosCmd)[AXIS], double (&VelCmd)[AXIS],
                     double (&AccCmd)[AXIS], int(&EndFlag));

    /**
	 * @brief Ū���y��R�O��(fast, memory method)
     * @param PosCmd: �O���y���m�R�O
     * @param VelCmd: �O���y��t�שR�O
     * @param AccCmd: �O���y��[�t�שR�O
     * @param EndFlag: �y��R�O�����X��
     */
    void Get_Cmd(double (&PosCmd)[AXIS], double (&VelCmd)[AXIS],
                 double (&AccCmd)[AXIS], int(&EndFlag));

    /** @brief �����y��R�O�� */
    void CloseFile();

private:
    FILE *fp;                                      // �y��R�O�ɮ׫���
    size_t line_cnt;                               // �y��R�O�ɮצ��
    std::string filename;                          // �y��R�O�ɦW
    std::vector<std::vector<double>> Track_PosCmd; // �x�s�Ҧ��y���m�R�O [line_cnt AXIS]
    std::vector<std::vector<double>> Track_VelCmd; // �x�s�Ҧ��y��t�שR�O [line_cnt AXIS]
    std::vector<std::vector<double>> Track_AccCmd; // �x�s�Ҧ��y��[�t�שR�O [line_cnt AXIS]
    size_t iter_exec;
};

#endif // TRACKING_H
