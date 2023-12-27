#pragma once

#include "data/Types.h"
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <Eigen/Core>

#include <map>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

namespace eacham::parser
{

template<typename T>
T Parse(const std::string& path)
{
    T result;

    nlohmann::json config;
    // read a JSON file
    std::ifstream inpStream(path);
    inpStream >> config;
    inpStream.close();

    return T::Parse(config);
}

}