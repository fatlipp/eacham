#pragma once

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <fstream>
#include <string>
#include <map>

namespace eacham
{

void SavePositions(const std::string& path, 
    const std::map<unsigned, std::pair<std::string, Eigen::Matrix4d>>& positions,
    const float w, const float h,
    const float cx, const float cy,
    const float fx, const float fy
    )
{
    nlohmann::json frames;

    frames["version"] = 0;
    frames["w"] = w;
    frames["h"] = h;
    frames["cx"] = cx;
    frames["cy"] = cy;
    frames["fl_x"] = fx;
    frames["fl_y"] = fy;
    frames["k1"] = 0;
    frames["k2"] = 0;
    frames["k3"] = 0;
    frames["k4"] = 0;
    frames["p1"] = 0;
    frames["p2"] = 0;
    frames["is_fisheye"] = false;

    const float angleX = std::atan(w / (fx * 2.0)) * 2.0;
    const float angleY = std::atan(h / (fy * 2.0)) * 2.0;
    frames["camera_angle_x"] = angleX;
    frames["camera_angle_y"] = angleY;
    frames["fovx"] = angleX * 180.0 / 3.141592;
    frames["fovy"] = angleY * 180.0 / 3.141592;

    frames["frames"] = {  };

    for (const auto& [id, posTuple] : positions)
    {
        auto [p, pos] = posTuple;
        // pos = pos.inverse();
        // Eigen::Matrix4d matr = Eigen::Matrix4d::Identity();
        // matr(1, 1) = -1;
        // matr(2, 2) = -1;
        // pos = pos * matr;
        
        nlohmann::json frame;
        frame["file_path"] = p;
        frame["transform_matrix"] = {
            { pos(0, 0), pos(0, 1), pos(0, 2), pos(0, 3) },
            { pos(1, 0), pos(1, 1), pos(1, 2), pos(1, 3) },
            { pos(2, 0), pos(2, 1), pos(2, 2), pos(2, 3) },
            { pos(3, 0), pos(3, 1), pos(3, 2), pos(3, 3) }
        };
        frames["frames"].push_back(frame);
    }

    std::ofstream file(path, std::ios_base::out);
    if (file.is_open())
    {
        file << std::setw(4) << frames << std::endl;

        file.close();
    }
}

} // namespace eacham
