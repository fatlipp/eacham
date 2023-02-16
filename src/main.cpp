#include <iostream>
#include "modules/data_source/DataInputSourceFactory.h"
#include "modules/odometry/VisualOdometry.h"

#include "modules/performance/BlockTimer.h"

#include <opencv4/opencv2/highgui.hpp>

int main()
{
    auto dataSource = data_source::CreateStereo(data_source::DataSourceType::DATASET, "/home/blackdyce/Datasets/kitty/data_odometry_gray");
    auto odometry = std::make_unique<odometry::VisualOdometry>();

    for (int i = 0; i < 2000; ++i)
    {
        performance::BlockTimer timer;
        const auto images = dataSource->GetNext();
        cv::imshow("im1", std::get<0>(images));
        cv::imshow("im2", std::get<1>(images));

        const auto odom = odometry->GetOdometry(images);
        std::cout << "->" << odom << std::endl;

        cv::waitKey(10);
    }

    return 0;
}