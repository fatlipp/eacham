#pragma once

#include <string>
#include <tuple>
#include <istream>
#include <fstream>

#include <eigen3/Eigen/Core>

namespace eacham
{

class IDataset
{
public:
    IDataset(const std::string& gtPosePath)
        : gtPosePath(gtPosePath)
    {
        std::cout << "gtPosePath: " << gtPosePath << std::endl;

        gtFileStream.open(gtPosePath, std::ios::in);
    }

    virtual ~IDataset()
    {
        if (gtFileStream.is_open())
        {
            gtFileStream.close();
        }
    }

    virtual void ReadNext() = 0;

    virtual Eigen::Matrix4f GetGtPose() const
    {
        return currentPose;
    }

protected:
    const std::string gtPosePath;
    mutable std::ifstream gtFileStream;

    mutable Eigen::Matrix4f currentPose;
};

}