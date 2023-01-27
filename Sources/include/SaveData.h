#ifndef SAVEDATA_H
#define SAVEDATA_H

#include <iostream>
#include <string>
#include <ctime>
#include "Safety_IO.h"
#include "delta/Setting.h"

//=============== 輸出檔案名稱前處理 ===============
#ifdef MY_OUTPUT_FILENAME
// 輸出檔名方法一: 自定義完整檔案路徑及名稱
#define __TIMESTAMP false
#define __OUTPUT MY_OUTPUT_FILENAME
#else
// 輸出檔名方法二: 使用當前系統時間戳記作為檔案名稱後綴
#define __TIMESTAMP true
#ifndef OUTPUT_PATH
#define OUTPUT_PATH "data\\" // 若無定義則使用預設路徑
#endif                                   // OUTPUT_PATH

#ifndef OUTPUT_PREFIX
#define OUTPUT_PREFIX "MeasuredData" // 若無定義則使用預設前綴
#endif                                // OUTPUT_PREFIX
#define __OUTPUT (OUTPUT_PATH OUTPUT_PREFIX)
#endif // MY_OUTPUT_FILENAME
//=============== 輸出檔案名稱前處理(END) ===============

class SaveData
{
public:
    SaveData();

    /**
	 * @brief 建立實驗資料檔
     * @param path_prefix: 實驗資料檔前綴(若要加時間標籤)
     * @param path_prefix: 實驗資料檔完整路徑名稱(不加時間標籤)
     * @param add_timestamp: 選擇是否加上時間標籤
     * @return 成功開啟回傳0,失敗則回傳1
     */
    int CreateSaveData(std::string path_prefix, bool add_timestamp);

    /**
	 * @brief 寫入實驗資料檔(1)
     * @param a: 實驗資料陣列 [1 by AXIS]
     */
    void save(double a[]);

    /**
	 * @brief 寫入實驗資料檔(3)
     * @param a: 實驗資料陣列 [1 by AXIS]
     * @param b: 實驗資料陣列 [1 by AXIS]
     */
    void save(double a[], double b[]);

    /**
	 * @brief 寫入實驗資料檔(3)
     * @param a: 實驗資料陣列 [1 by AXIS]
     * @param b: 實驗資料陣列 [1 by AXIS]
     * @param c: 實驗資料陣列 [1 by AXIS]
     */
    void save(double a[], double b[], double c[]);

    /**
	 * @brief 寫入實驗資料檔(5)
     * @param a: 實驗資料陣列 [1 by AXIS]
     * @param b: 實驗資料陣列 [1 by AXIS]
     * @param c: 實驗資料陣列 [1 by AXIS]
     * @param d: 實驗資料陣列 [1 by AXIS]
     * @param e: 實驗資料陣列 [1 by AXIS]
     */
    void save(double a[], double b[], double c[],
              double d[], double e[]);

    /**
	 * @brief 寫入實驗資料檔(6)
     * @param a: 實驗資料陣列 [1 by AXIS]
     * @param b: 實驗資料陣列 [1 by AXIS]
     * @param c: 實驗資料陣列 [1 by AXIS]
     * @param d: 實驗資料陣列 [1 by AXIS]
     * @param e: 實驗資料陣列 [1 by AXIS]
     * @param f: 實驗資料陣列 [1 by AXIS]
     */
    void save(double a[], double b[], double c[],
              double d[], double e[], double f[]);

    /** @brief 關閉實驗資料 */
    void CloseFile();

private:
    FILE *fp;             // 實驗資料檔案指標
    size_t line_cnt;      // 實驗資料檔行數
    std::string filename; // 實驗資料檔名
};

/** @brief 取得系統時間戳記 */
std::string get_current_sys_time();

#endif // SAVEDATA_H
