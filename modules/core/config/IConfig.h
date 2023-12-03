#pragma once

#include <json.hpp>

namespace eacham
{

class IConfig
{
public:
    IConfig(const std::string& nameInp)
        : name(nameInp)
    {
    }

protected:
    bool Check(const nlohmann::json& data)
    {
        if (!data.contains(this->name))
        {
            return false;
        }

        return true;
    }

protected:
    const std::string name;
};

}