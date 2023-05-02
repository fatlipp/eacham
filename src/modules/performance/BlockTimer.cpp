#include "performance/BlockTimer.h"

#include <iostream>

namespace eacham
{

std::unordered_map<std::string, std::pair<unsigned, double>> BlockTimer::timeMap;

BlockTimer::BlockTimer(const std::string &caption, const bool saveStat)
    : caption(caption)
    , saveStat(saveStat)
{
    this->startTime = std::chrono::high_resolution_clock::now();
}

BlockTimer::~BlockTimer()
{
    const auto endTime = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - this->startTime).count();

    std::cout << "[" << caption << "] time: " << duration << std::endl;

    if (saveStat)
    {
        if (BlockTimer::timeMap.contains(this->caption))
        {
            BlockTimer::timeMap[this->caption].first++;
            BlockTimer::timeMap[this->caption].second += duration;
        }
        else
        {
            BlockTimer::timeMap.insert({this->caption, std::make_pair(1, duration)});
        }
    }
}

void BlockTimer::PrintStat()
{
    std::cout << "[===============Time report===============]" << std::endl;

    for (const auto& [caption, value] : timeMap)
    {
        std::cout << "[" << caption << "] count: " << value.first;
        std::cout << ", meanTime: " << (value.second / value.first) << std::endl;
    }
}

} // namespace eacham

