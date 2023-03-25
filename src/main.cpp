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

            renderer.Stop();
        });

    // TODO: Config
    const auto sourceType = DataSourceType::DATASET;
    const std::string folder = "/home/blackdyce/Datasets/KITTI";
    
    auto dataSourceLidar = CreateLidar<lidardata_t>(sourceType, folder);
    auto datasetLidar = dynamic_cast<IDataset<lidardata_t>*>(dataSourceLidar.get());

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
            datasetLidar->ReadNext();
            const auto gtPos = dataset->GetGtPose();

            const auto images = dataSource->Get();

            {
                std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+++++++++++++++++++++++++++++[" << frameId << "]+++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

                BlockTimer timer { "Overall Loop" };
                Eigen::Matrix4f currentPos;
                {
                    BlockTimer timer { "Odom" };
                    if (visualOdometry->Proceed(images))
                    {
                        currentPos = visualOdometry->GetOdometry();
                    }
                    else
                    {
                        std::cout << "Odometry error." << std::endl;
                        break;
                    }
                }

                const Eigen::Matrix4f diff = (currentPos - gtPos);
                const float diffLen = std::sqrt(diff(0, 3) * diff(0, 3) + 
                                                diff(1, 3) * diff(1, 3) + 
                                                diff(2, 3) * diff(2, 3));
                renderer.AddFGTPoint(gtPos);
                renderer.DrawMapFrames(visualOdometry->GetLocalMapFrames());

                auto lidarData = dataSourceLidar->Get();

                for (auto &point : lidarData)
                {
                    const Eigen::Vector4f pointTrans = currentPos * Eigen::Vector4f{point.x(), point.y(), point.z(), 1.0f };
                    point = Eigen::Vector3f {pointTrans.x(), pointTrans.y(), pointTrans.z() };
                }

                renderer.DrawLidarData(lidarData);
                renderer.DrawFrame(visualOdometry->GetCurrentFrame());
                // renderer.DrawMapPoints(visualOdometry->GetLocalMapPoints());

                std::cout << "Current pos:\n" << currentPos << std::endl;
                std::cout << "GT pos:\n" << gtPos << std::endl;
                std::cout << "Diff:\n" << diffLen << "\n" << diff << std::endl;
            }

            ++frameId;

            if (frameId > 300)
                break;
        }

        cv::waitKey(1);
    }

    while (renderer.IsActive())
    {
        cv::waitKey(10);
    }

    return 0;
}