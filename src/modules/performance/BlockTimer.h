#pragma once

#include <iostream>

#include <chrono>

namespace performance
{

class BlockTimer
{
public:
    BlockTimer()
    {
        this->startTime = std::chrono::high_resolution_clock::now();
    }

    ~BlockTimer()
    {
        const auto endTime = std::chrono::high_resolution_clock::now();

        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - this->startTime).count();

        std::cout << "duration = " << duration << std::endl;
    }

private:
    std::chrono::high_resolution_clock::time_point startTime;

};

} // namespace performance

