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
	 * @brief 開啟軌跡命令檔,檢查完整軌跡命令檔,並讀取第一項軌跡命令(for Step1.PTP)
     * @param path: 軌跡命令檔案名稱
     * @param FirstPos: 記錄第一項軌跡位置命令
     * @return 成功開啟回傳0,失敗則回傳非0
     */
    int TrackingFile_Init(const char *path, double (&FirstPos)[AXIS]);

    /**
	 * @brief 開啟軌跡命令檔
     * @param path: 軌跡命令檔案名稱
     * @return 成功開啟回傳0,失敗則回傳非0
     */
    int Tracking_OpenFile(const char *path);

    /**
	 * @brief 讀取軌跡命令檔(slow, file pointer method)
     * @param PosCmd: 記錄軌跡位置命令
     * @param VelCmd: 記錄軌跡速度命令
     * @param AccCmd: 記錄軌跡加速度命令
     * @param EndFlag: 軌跡命令結束旗標
     */
    void Get_Cmd_old(double (&PosCmd)[AXIS], double (&VelCmd)[AXIS],
                     double (&AccCmd)[AXIS], int(&EndFlag));

    /**
	 * @brief 讀取軌跡命令檔(fast, memory method)
     * @param PosCmd: 記錄軌跡位置命令
     * @param VelCmd: 記錄軌跡速度命令
     * @param AccCmd: 記錄軌跡加速度命令
     * @param EndFlag: 軌跡命令結束旗標
     */
    void Get_Cmd(double (&PosCmd)[AXIS], double (&VelCmd)[AXIS],
                 double (&AccCmd)[AXIS], int(&EndFlag));

    /** @brief 關閉軌跡命令檔 */
    void CloseFile();

private:
    FILE *fp;                                      // 軌跡命令檔案指標
    size_t line_cnt;                               // 軌跡命令檔案行數
    std::string filename;                          // 軌跡命令檔名
    std::vector<std::vector<double>> Track_PosCmd; // 儲存所有軌跡位置命令 [line_cnt AXIS]
    std::vector<std::vector<double>> Track_VelCmd; // 儲存所有軌跡速度命令 [line_cnt AXIS]
    std::vector<std::vector<double>> Track_AccCmd; // 儲存所有軌跡加速度命令 [line_cnt AXIS]
    size_t iter_exec;
};

#endif // TRACKING_H
