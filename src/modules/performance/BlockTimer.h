#pragma once

#include <iostream>

#include <chrono>

namespace eacham
{

class BlockTimer
{
public:
    BlockTimer(const std::string &caption)
        : caption(caption)
    {
        this->startTime = std::chrono::high_resolution_clock::now();
    }

    ~BlockTimer()
    {
        const auto endTime = std::chrono::high_resolution_clock::now();

        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - this->startTime).count();

        std::cout << "[" << caption << "] time: " << duration << std::endl;
    }

private:
    const std::string caption;

    std::chrono::high_resolution_clock::time_point startTime;

};

} // namespace eacham

