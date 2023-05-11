#include "FrameCreatorRgbd.h"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

#include "tools/Tools3d.h"

#include <ranges>

namespace eacham
{

IFrame FrameCreatorRgbd::Create(const stereodata_t& data)
{
    const auto [features, descriptors] = extractor->GetFeatures(std::get<1>(data));

    std::cout << "RGBD features.size() = " << features.size() << std::endl;

    if (features.size() < 10)
    {
        return {};
    }

    IFrame frame;
    frame.SetId(GetId());

    for (unsigned pointId = 0; const auto& feature : features)
    {
        const cv::Point3f pos3d = tools::Get3dPointByDepthMap(feature.pt, std::get<2>(data), this->cameraData);

        if (pos3d.z > 0.10f && pos3d.z < 70.0f)
        {   
            frame.AddPoint({ .id = pointId, .keypoint = feature.pt, .position3d = pos3d, .descriptor = descriptors.row(pointId).clone() });
        }

        ++pointId;
    }

    return frame;
}

}