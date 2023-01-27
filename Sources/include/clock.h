#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>

class Clock
{
public:
    Clock();
    Clock(const Clock &);
    ~Clock();

    /** @brief �}�l�p�� */
    void start();

    /** @brief ����p�ɨðO���g�L�ɶ� */
    void stop();

    /** @brief �d�ݸg�L�ɶ� [us] (�~��p��) */
    int64_t peekElapsedTime() const;

    /** @brief ���o�g�L�ɶ� [us] (�ݥ�����p��) */
    int64_t getElapsedTime() const;

    /** @brief ���o�p�ɾ��ƶq */
    static int getNum();

    /** @brief ���o�Ҧ��p�ɾ��g�L�ɶ����[�` [us] */
    static int64_t getTotal();

private:
    std::chrono::high_resolution_clock::time_point start_ts; // �}�l�ɶ� [us]
    int64_t elapsed_time;                                    // �g�L�ɶ� [us]
    static int numClock;                                     // �p�ɾ��ƶq
    static int64_t totalClock;                               // �Ҧ��p�ɾ��g�L�ɶ����[�` [us]
};

#endif
