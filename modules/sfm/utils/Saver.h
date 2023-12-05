#pragma once

#include <Eigen/Geometry>

#include <fstream>

namespace eacham
{

void SavePositions(const std::string& path, const std::map<unsigned, Eigen::Matrix4d>& positions)
{
    std::ofstream file(path, std::ios_base::out);
    if (file.is_open())
    {
        for (const auto& [id, pos] : positions)
        {
            file << "frame: " << id << '\n';
            file << pos << '\n';
        }

        file.close();
    }
}

} // namespace eacham
