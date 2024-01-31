#pragma once

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