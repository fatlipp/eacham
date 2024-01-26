#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

namespace config
{

template<typename T>
T Parse(const std::string& path)
{
    nlohmann::json config;
    std::ifstream inpStream(path);
    if (!inpStream.is_open())
    {
        std::cerr << "file not found: " << path << std::endl;
        return {};
    }

    inpStream >> config;
    inpStream.close();

    return T::Parse(config);
}
    
} // namespace config