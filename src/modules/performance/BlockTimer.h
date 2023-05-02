#pragma once

#include <string>
#include <chrono>
#include <unordered_map>

namespace eacham
{

class BlockTimer
{

public:
    BlockTimer(const std::string &caption, const bool saveStat = false);

    ~BlockTimer();

    static void PrintStat();

protected:
    static std::unordered_map<std::string, std::pair<unsigned, double>> timeMap;

private:
    const std::string caption;
    const bool saveStat;

    std::chrono::high_resolution_clock::time_point startTime;


};

} // namespace eacham

