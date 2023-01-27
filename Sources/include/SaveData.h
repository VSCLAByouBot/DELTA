#ifndef SAVEDATA_H
#define SAVEDATA_H

#include <iostream>
#include <string>
#include <ctime>
#include "Safety_IO.h"
#include "delta/Setting.h"

//=============== ��X�ɮצW�٫e�B�z ===============
#ifdef MY_OUTPUT_FILENAME
// ��X�ɦW��k�@: �۩w�q�����ɮ׸��|�ΦW��
#define __TIMESTAMP false
#define __OUTPUT MY_OUTPUT_FILENAME
#else
// ��X�ɦW��k�G: �ϥη�e�t�ήɶ��W�O�@���ɮצW�٫��
#define __TIMESTAMP true
#ifndef OUTPUT_PATH
#define OUTPUT_PATH "data\\" // �Y�L�w�q�h�ϥιw�]���|
#endif                                   // OUTPUT_PATH

#ifndef OUTPUT_PREFIX
#define OUTPUT_PREFIX "MeasuredData" // �Y�L�w�q�h�ϥιw�]�e��
#endif                                // OUTPUT_PREFIX
#define __OUTPUT (OUTPUT_PATH OUTPUT_PREFIX)
#endif // MY_OUTPUT_FILENAME
//=============== ��X�ɮצW�٫e�B�z(END) ===============

class SaveData
{
public:
    SaveData();

    /**
	 * @brief �إ߹�������
     * @param path_prefix: �������ɫe��(�Y�n�[�ɶ�����)
     * @param path_prefix: �������ɧ�����|�W��(���[�ɶ�����)
     * @param add_timestamp: ��ܬO�_�[�W�ɶ�����
     * @return ���\�}�Ҧ^��0,���ѫh�^��1
     */
    int CreateSaveData(std::string path_prefix, bool add_timestamp);

    /**
	 * @brief �g�J��������(1)
     * @param a: �����ư}�C [1 by AXIS]
     */
    void save(double a[]);

    /**
	 * @brief �g�J��������(3)
     * @param a: �����ư}�C [1 by AXIS]
     * @param b: �����ư}�C [1 by AXIS]
     */
    void save(double a[], double b[]);

    /**
	 * @brief �g�J��������(3)
     * @param a: �����ư}�C [1 by AXIS]
     * @param b: �����ư}�C [1 by AXIS]
     * @param c: �����ư}�C [1 by AXIS]
     */
    void save(double a[], double b[], double c[]);

    /**
	 * @brief �g�J��������(5)
     * @param a: �����ư}�C [1 by AXIS]
     * @param b: �����ư}�C [1 by AXIS]
     * @param c: �����ư}�C [1 by AXIS]
     * @param d: �����ư}�C [1 by AXIS]
     * @param e: �����ư}�C [1 by AXIS]
     */
    void save(double a[], double b[], double c[],
              double d[], double e[]);

    /**
	 * @brief �g�J��������(6)
     * @param a: �����ư}�C [1 by AXIS]
     * @param b: �����ư}�C [1 by AXIS]
     * @param c: �����ư}�C [1 by AXIS]
     * @param d: �����ư}�C [1 by AXIS]
     * @param e: �����ư}�C [1 by AXIS]
     * @param f: �����ư}�C [1 by AXIS]
     */
    void save(double a[], double b[], double c[],
              double d[], double e[], double f[]);

    /** @brief ���������� */
    void CloseFile();

private:
    FILE *fp;             // �������ɮ׫���
    size_t line_cnt;      // �������ɦ��
    std::string filename; // �������ɦW
};

/** @brief ���o�t�ήɶ��W�O */
std::string get_current_sys_time();

#endif // SAVEDATA_H
