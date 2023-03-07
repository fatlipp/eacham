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
    const std::string folder = "/home/blackdyce/Datasets/KITTI";
    
    auto dataSource = data_source::CreateStereo<stereodata_t>(sourceType, folder);
    auto dataset = dynamic_cast<data_source::IDataset<stereodata_t>*>(dataSource.get());
    auto visualOdometry = std::make_unique<odometry::VisualOdometry<stereodata_t>>(dataSource.get());

    if (dataset == nullptr)
    {
        return -1;
    }

    for (int i = 0; i < 1000; ++i)
    {
        // dataset (camera should use concurrent thread to get the next frame)
        dataset->ReadNext();
        const auto gtPose = dataset->GetGtPose();

        const auto images = dataSource->Get();

        {
            performance::BlockTimer timer;
            const auto odom = visualOdometry->GetOdometry(images);
            renderer.AddFramePoint(odom);


            std::cout << "ODOM:\n" << odom << std::endl;
            std::cout << "GT:\n" << gtPose << std::endl;
            std::cout << "DIFF:\n" << (odom - gtPose) << std::endl;
        }

        cv::waitKey(1);
    }

    renderer.Stop();

    return 0;
}