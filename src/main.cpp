#include <iostream>
#include "modules/data_source/DataInputSourceFactory.h"
#include "modules/odometry/VisualOdometry.h"

#include "modules/performance/BlockTimer.h"
#include "visualization/Render.h"


int main()
{    
    auto dataSource = data_source::CreateStereo<stereodata_t>(data_source::DataSourceType::DATASET, "/home/blackdyce/Datasets/kitty/data_odometry_gray");
    auto kittySOurce = dynamic_cast<data_source::DataSourceKittyStereo<stereodata_t>*>(dataSource.get());

    auto visualOdometry = std::make_unique<odometry::VisualOdometry<stereodata_t>>(dataSource.get());
    // visualOdometry->GetOdometry

    render::Render renderer;
    Eigen::Vector3f z(0.0f, 0.0f, 3.0f);
    Eigen::Vector3f y(0.0f, 3.0f, 0.0f);
    Eigen::Vector3f x(3.0f, 0.0f, 0.0f);
    renderer.AddPoint(x);
    renderer.AddPoint(y);
    renderer.AddPoint(z);
    const auto images1 = dataSource->GetNext();
    visualOdometry->GetOdometry(images1, renderer);

    for (int i = 0; i < 1000; ++i)
    {
        const auto images = dataSource->GetNext();
        cv::imshow("im1", std::get<0>(images));

        {
            performance::BlockTimer timer;
            const auto odom = visualOdometry->GetOdometry(images, renderer);
            // renderer.SetCameraPosition(odom);
        }

        cv::waitKey(10);
    }

    std::this_thread::sleep_for(std::chrono::seconds(25));

    renderer.Stop();

    return 0;
}