#include <iostream>
#include "data_source/DataSourceDirector.h"
#include "data_source/dataset/IDataset.h"
#include "odometry/VisualOdometryDirector.h"
#include "performance/BlockTimer.h"
#include "visualization/Render.h"
#include "config/Config.h"

using namespace eacham;

int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        std::cout << "Usage: `dataset_reader 'dataset folder'`" << std::endl;
        return 1;
    }

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

    Config config { argv[1] };
    if (!config.Read())
    {
        return 2;
    }

    DataSourceDirector<stereodata_t> dataSourceDirector;
    auto dataSource = dataSourceDirector.Build(config.GetSource());
    auto dataset = dynamic_cast<IDataset<stereodata_t>*>(dataSource.get());

    if (dataset == nullptr)
    {
        return 3;
    }

    VisualOdometryDirector visualOdometryDirector;
    auto odometry = visualOdometryDirector.Build(dynamic_cast<IDataSourceCamera<stereodata_t>*>(dataSource.get()), config.GetOdometry());

    const auto maxFrames = config.GetGeneral().maxFrames;
    int frameId = 0;

    Eigen::Matrix4f prevPos = Eigen::Matrix4f::Identity();
    float drivenDistTotal = 0.0;

    Eigen::Matrix4f defaultPos = Eigen::Matrix4f::Identity();
    while (!close)
    {
        if (nextStep || play)
        {
            nextStep = false;

            // dataset (camera should use concurrent thread to get the next frame)
            {
                std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+++++++++++++++++++++++++++++[" << frameId << "]+++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

                BlockTimer timer { "Read dataset" };
                dataset->ReadNext();
                // datasetLidar->ReadNext();
            }
            auto gtPos = dataset->GetGtPose();

            if (frameId == 0)
            {
                defaultPos = gtPos.inverse();
            }

            gtPos = defaultPos * gtPos;

            const auto images = dataSource->Get();

            {
                BlockTimer timer { "Overall Loop" };
                Eigen::Matrix4f currentPos;
                {
                    BlockTimer timer { "Odom" };
                    if (odometry->Proceed(images))
                    {
                        currentPos = odometry->GetOdometry();
                    }
                    else
                    {
                        break;
                    }
                }

                const Eigen::Matrix4f diff = (currentPos - gtPos);
                const float diffLen = std::sqrt(diff(0, 3) * diff(0, 3) + 
                                                diff(1, 3) * diff(1, 3) + 
                                                diff(2, 3) * diff(2, 3));
                renderer.AddFGTPoint(gtPos);
                renderer.DrawMapFrames(odometry->GetLocalMap()->GetFrames());

                // auto lidarData = dataSourceLidar->Get();

                // for (auto &point : lidarData)
                // {
                //     const Eigen::Vector4f pointTrans = currentPos * Eigen::Vector4f{point.x(), point.y(), point.z(), 1.0f };
                //     point = Eigen::Vector3f {pointTrans.x(), pointTrans.y(), pointTrans.z() };
                // }

                // renderer.DrawLidarData(lidarData);
                // renderer.DrawMapPoints(odometry->GetLocalMapPoints());

                std::cout << "Current pos:\n" << currentPos << std::endl;
                std::cout << "GT pos:\n" << gtPos << std::endl;
                std::cout << "Diff:\n" << diffLen << "\n" << diff << std::endl;

                {
                    const Eigen::Matrix4f drivenDistMat = (currentPos - prevPos);
                    const float drivenDist =  std::sqrt(drivenDistMat(0, 3) * drivenDistMat(0, 3) + 
                                                        drivenDistMat(1, 3) * drivenDistMat(1, 3) + 
                                                        drivenDistMat(2, 3) * drivenDistMat(2, 3));
                    drivenDistTotal += drivenDist;
                    std::cout << "Driven distance: " << drivenDistTotal << std::endl;
                }

                prevPos = currentPos;
            }

            ++frameId;

            if (maxFrames > -1 && frameId > maxFrames)
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