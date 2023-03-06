#include <iostream>
#include "modules/data_source/DataInputSourceFactory.h"
#include "modules/odometry/VisualOdometry.h"

#include "modules/performance/BlockTimer.h"
#include "visualization/Render.h"

int main(int argc, char* argv[])
{
    if (argc > 1)
    {
    }

    render::Render renderer;

    // TODO: Config
    const auto sourceType = data_source::DataSourceType::DATASET;
    const std::string folder = "/home/blackdyce/Datasets/kitty/data_odometry_gray";
    
    auto dataSource = data_source::CreateStereo<stereodata_t>(sourceType, folder);
    auto visualOdometry = std::make_unique<odometry::VisualOdometry<stereodata_t>>(dataSource.get());

    for (int i = 0; i < 1000; ++i)
    {
        const auto images = dataSource->GetNext();

        {
            performance::BlockTimer timer;
            const auto odom = visualOdometry->GetOdometry(images);
            renderer.AddFramePoint(odom);
        }

        cv::waitKey(1);
    }

    renderer.Stop();

    return 0;
}