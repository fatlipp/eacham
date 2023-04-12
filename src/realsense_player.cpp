#include <iostream>
#include "data_source/VisualDataSourceDirector.h"
#include "data_source/dataset/IDataset.h"
#include "odometry/VisualOdometryDirector.h"
#include "performance/BlockTimer.h"
#include "config/Config.h"
#include "pipeline/Pipeline.h"
#include "pipeline/PipelineDataset.h"
#include "visualization/Render.h"
#include "visualization/view/VisualOdometryView.h"
#include "visualization/view/DatasetView.h"

using namespace eacham;

int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        std::cout << "Usage: `realsense_player 'config folder'`" << std::endl;
        return 1;
    }

    Config config { argv[1] };
    if (!config.Read())
    {
        return 2;
    }

    using T = stereodata_t;

    // specific for visual sensor
    VisualDataSourceDirector<T> dataSourceDirector;
    auto dataSource = dataSourceDirector.Build(config.GetSource());

    VisualOdometryDirector<T> visualOdometryDirector;
    auto odometry = visualOdometryDirector.Build(dataSource.get(), config.GetOdometry());

    // general implementation
    Pipeline<T> pipeline { config.GetGeneral() };

    auto render = std::make_unique<Render>();
    render->SetOnPlayClick([&pipeline]() { pipeline.Play(); });
    render->SetOnStepClick([&pipeline]() { pipeline.Step(); });
    render->SetOnCloseClick([&pipeline]() { pipeline.Kill(); });

    auto vo = dynamic_cast<IFrameToMapOdometry<T>*>(odometry.get()) ;
    if (vo != nullptr)
    {
        render->Add(std::make_unique<VisualOdometryView<T>>(vo));
    }
    
    pipeline.SetDataSource(std::move(dataSource));
    pipeline.SetOdometry(std::move(odometry));
    pipeline.SetRender(std::move(render));
    pipeline.Start();

    while (pipeline.IsActive())
    {
    }

    return 0;
}