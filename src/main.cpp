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
    bool localOptimizerState = false;

    if (argc > 1)
    {
        extractorType = (std::string(argv[1]) == "ORB" ? FeatureExtractorType::ORB : 
                        (std::string(argv[1]) == "SIFT" ? FeatureExtractorType::SIFT : FeatureExtractorType::SURF));
    }

    if (argc > 2)
    {
        motionEstimatorType = (std::string(argv[2]) == "OPT" ? MotionEstimatorType::OPT : MotionEstimatorType::PNP);
    }

    if (argc > 3)
    {
        localOptimizerState = std::stoi(std::string(argv[3])) == 1 ? true : false;
    }

    std::cout << "extractorType: " << static_cast<int>(extractorType) << std::endl; 
    std::cout << "motionEstimatorType: " << static_cast<int>(motionEstimatorType) << std::endl; 

    bool play = false;
    bool nextStep = true;
    bool close = false;

    Render renderer;
    renderer.SetOnPlayClick([&]()
        {
            play = true;
        });
    renderer.SetOnStepClick([&]()
        {
            nextStep = !play;
            play = false;
        });
    renderer.SetOnCloseClick([&]()
        {
            close = true;
        });

    // TODO: Config
    const auto sourceType = DataSourceType::DATASET;
    const std::string folder = "/home/blackdyce/Datasets/KITTI";
    
    auto dataSource = CreateStereo<stereodata_t>(sourceType, folder);
    auto dataset = dynamic_cast<IDataset<stereodata_t>*>(dataSource.get());
    auto visualOdometry = std::make_unique<VisualOdometry<stereodata_t>>(extractorType, motionEstimatorType, dataSource.get());

    visualOdometry->SetLocalOptimizerState(localOptimizerState);

    if (dataset == nullptr)
    {
        return -1;
    }

    // for (int i = 0; i < 17; ++i)
    //     dataset->ReadNext();

    int frameId = 0;

    while (!close)
    {
        if (nextStep || play)
        {
            nextStep = false;

            // dataset (camera should use concurrent thread to get the next frame)
            dataset->ReadNext();
            const auto gtPose = dataset->GetGtPose();

            const auto images = dataSource->Get();

            {
                std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+++++++++++++++++++++++++++++[" << frameId << "]+++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

                BlockTimer timer;
                if (visualOdometry->Proceed(images))
                {
                    const auto odom = visualOdometry->GetOdometry();
                    // renderer.AddFramePoint(odom);
                    renderer.AddFGTPoint(gtPose);
                    renderer.DrawMapFrames(visualOdometry->GetLocalMapFrames());

                    std::cout << "ODOM:\n" << odom << std::endl;
                    std::cout << "GT:\n" << gtPose << std::endl;
                    std::cout << "DIFF:\n" << (odom - gtPose) << std::endl;
                }
                else
                {
                    std::cout << "Odometry error." << std::endl;
                    break;
                }
            }

            ++frameId;
        }

        cv::waitKey(1);
    }

    renderer.Stop();

    return 0;
}