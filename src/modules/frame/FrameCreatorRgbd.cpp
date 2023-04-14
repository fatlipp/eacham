#include "FrameCreatorRgbd.h"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

#include "tools/Tools3d.h"

namespace eacham
{

Frame FrameCreatorRgbd::Create(const stereodata_t& data)
{
    const auto [features, descriptors] = extractor->GetFeatures(std::get<1>(data));

    std::cout << "features.size() = " << features.size() << std::endl;

    if (features.size() < 10)
    {
        return {};
    }

    Frame frame {std::get<0>(data)};

    int pointId = 0;

    for (const auto& feature : features)
    {
        const cv::Point3f pos3d = tools::Get3dPointByDepthMap(feature.pt, std::get<2>(data), this->cameraData);

        if (pos3d.z > 0.10f && pos3d.z < 70.0f)
        {
            frame.AddPoint(pointId, pos3d, feature, descriptors.row(pointId));
        }

        ++pointId;
    }

    return frame;
}

}