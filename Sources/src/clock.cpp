#include "Clock.h"

int Clock::numClock = 0;       // 定義static變數(.寫在cpp)
int64_t Clock::totalClock = 0; // 定義static變數(.寫在cpp)

Clock::Clock()
{
    ++numClock;
}

Clock::Clock(const Clock &)
{
    ++numClock;
}

Clock::~Clock()
{
    --numClock;
}

void Clock::start()
{
    start_ts = std::chrono::high_resolution_clock::now();
}

void Clock::stop()
{
    elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_ts).count();
    totalClock += elapsed_time;
}

int64_t Clock::peekElapsedTime() const
{
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_ts).count();
}

int64_t Clock::getElapsedTime() const
{
    return elapsed_time;
}

int Clock::getNum()
{
    return numClock;
}

int64_t Clock::getTotal()
{
    return totalClock;
}
