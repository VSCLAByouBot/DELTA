#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>

class Clock
{
public:
    Clock();
    Clock(const Clock &);
    ~Clock();

    /** @brief 開始計時 */
    void start();

    /** @brief 停止計時並記錄經過時間 */
    void stop();

    /** @brief 查看經過時間 [us] (繼續計時) */
    int64_t peekElapsedTime() const;

    /** @brief 取得經過時間 [us] (需先停止計時) */
    int64_t getElapsedTime() const;

    /** @brief 取得計時器數量 */
    static int getNum();

    /** @brief 取得所有計時器經過時間之加總 [us] */
    static int64_t getTotal();

private:
    std::chrono::high_resolution_clock::time_point start_ts; // 開始時間 [us]
    int64_t elapsed_time;                                    // 經過時間 [us]
    static int numClock;                                     // 計時器數量
    static int64_t totalClock;                               // 所有計時器經過時間之加總 [us]
};

#endif
