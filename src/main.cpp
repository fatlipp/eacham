#include <iostream>
#include "modules/data_source/DataInputSourceFactory.h"
#include "modules/odometry/VisualOdometry.h"

#include "modules/performance/BlockTimer.h"
#include "visualization/Render.h"

using namespace eacham;

int main(int argc, char* argv[])
{
    FeatureExtractorType extractorType = FeatureExtractorType::ORB;
    MotionEstimatorType motionEstimatorType = MotionEstimatorType::OPT;

    if (argc > 1)
    {
        extractorType = (std::string(argv[1]) == "ORB" ? FeatureExtractorType::ORB : 
                        (std::string(argv[1]) == "SIFT" ? FeatureExtractorType::SIFT : FeatureExtractorType::SURF));
    }

    if (argc > 2)
    {
        motionEstimatorType = (std::string(argv[2]) == "OPT" ? MotionEstimatorType::OPT : MotionEstimatorType::PNP);
    }

    std::cout << "extractorType: " << static_cast<int>(extractorType) << std::endl; 
    std::cout << "motionEstimatorType: " << static_cast<int>(motionEstimatorType) << std::endl; 

    cv::waitKey(1000);

    // render::Render renderer;

    // TODO: Config
    const auto sourceType = DataSourceType::DATASET;
    const std::string folder = "/home/blackdyce/Datasets/KITTI";
    
    auto dataSource = CreateStereo<stereodata_t>(sourceType, folder);
    auto dataset = dynamic_cast<IDataset<stereodata_t>*>(dataSource.get());
    auto visualOdometry = std::make_unique<VisualOdometry<stereodata_t>>(extractorType, motionEstimatorType, dataSource.get());

    if (dataset == nullptr)
    {
        return -1;
    }

    for (int i = 0; i < 5; ++i)
    {
        // dataset (camera should use concurrent thread to get the next frame)
        dataset->ReadNext();
        const auto gtPose = dataset->GetGtPose();

        const auto images = dataSource->Get();

        {
            BlockTimer timer;
            if (visualOdometry->Proceed(images))
            {
                const auto odom = visualOdometry->GetOdometry();
                // renderer.AddFramePoint(odom);
                // renderer.DrawMap(visualOdometry->GetLocalMapPoints());

                std::cout << "ODOM:\n" << odom << std::endl;
                std::cout << "GT:\n" << gtPose << std::endl;
                std::cout << "DIFF:\n" << (odom - gtPose) << std::endl;
            }
        }

        cv::waitKey(1);
    }

    // cv::waitKey(1000000);

    // renderer.Stop();

    return 0;
}