#include <iostream>
#include "data_source/VisualDataSourceDirector.h"
#include "data_source/dataset/IDataset.h"
#include "odometry/VisualOdometryDirector.h"
#include "performance/BlockTimer.h"
#include "visualization/Render.h"
#include "config/Config.h"
#include "pipeline/Pipeline.h"
#include "pipeline/PipelineDataset.h"

using namespace eacham;

int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        std::cout << "Usage: `dataset_reader 'dataset folder'`" << std::endl;
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
    PipelineDataset<T> pipeline { config.GetGeneral() };

    auto render = std::make_unique<Render>();
    render->SetOnPlayClick([&pipeline]() { pipeline.Play(); });
    render->SetOnStepClick([&pipeline]() { pipeline.Step(); });
    render->SetOnCloseClick([&pipeline]() { pipeline.Stop(); });
    
    pipeline.SetDataSource(std::move(dataSource));
    pipeline.SetOdometry(std::move(odometry));
    pipeline.SetRender(std::move(render));
    pipeline.Start();

    return 0;
}