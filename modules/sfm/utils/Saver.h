#pragma once

#include <Eigen/Geometry>

#include <fstream>
#include <string>
#include <map>

namespace eacham
{

void SavePositions(const std::string& path, 
    const std::map<unsigned, std::pair<std::string, Eigen::Matrix4d>>& positions)
{
    std::ofstream file(path, std::ios_base::out);
    if (file.is_open())
    {
        for (const auto& [id, pos] : positions)
        {
            // file << "frame: " << id << '\n';
            file << pos.first << '\n';
            file << pos.second << '\n';
        }

        file.close();
    }
}

} // namespace eacham
