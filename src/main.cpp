#include <iostream>
#include "data_source/VisualDataSourceDirector.h"
#include "data_source/dataset/IDataset.h"
#include "odometry/VisualOdometryDirector.h"
#include "performance/BlockTimer.h"
#include "config/Config.h"
#include "pipeline/SlamPipeline.h"
#include "optimizer/OptimizerFactory.h"
#include "visualization/Render.h"
#include "visualization/view/DatasetView.h"
#include "visualization/view/FrameView.h"
#include "visualization/view/MapView.h"
#include "frame/FrameCreatorFactory.h"
#include "map/Map.h"

using namespace eacham;

void MakeDataset()
{
}

int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        std::cout << "Usage: `main 'config folder'`" << std::endl;
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
    auto dataSource = dataSourceDirector.Build(config);

    if (dataSource == nullptr)
    {
        std::cerr << "dataSource is null" << std::endl;

        return 3;
    }

    std::unique_ptr<IFrameCreator> frameCreator = BuildFrameCreator(dataSource.get(), config);

    VisualOdometryDirector<T> visualOdometryDirector;
    auto odometry = visualOdometryDirector.Build(dataSource.get(), config);

    if (odometry == nullptr)
    {
        std::cerr << "odometry is null" << std::endl;

        return 3;
    }

    auto optimizer = OptimizerFactory::Build(config, dataSource->GetParameters());
    auto frameView = std::make_unique<FrameView>();
    auto dataSourcePtr = dataSource.get();

    auto map = std::make_unique<Map>();
    auto mapPtr = map.get();

    std::unique_ptr<SlamPipeline> pipeline = std::make_unique<SlamPipeline>();
    pipeline->SetMap(std::move(map));
    pipeline->SetDataSource(std::move(dataSource));
    pipeline->SetFrameCreator(std::move(frameCreator));
    pipeline->SetOdometry(std::move(odometry));
    pipeline->SetOptimizer(std::move(optimizer));
    pipeline->SetOnUpdate([view = frameView.get()](const auto& position) 
        {
            view->SetPosition(position); 
        });

    std::unique_ptr<Render> render = std::make_unique<Render>();
    render->SetOnPlayClick([&pipeline]() { pipeline->Play(); });
    render->SetOnStepClick([&pipeline]() { pipeline->Step(); });
    render->SetOnResetClick([&pipeline]() { pipeline->Reset(); });
    render->SetOnCloseClick([&pipeline, &dataSource]() { pipeline->Deactivate(); });
    render->Add(std::move(frameView));
    render->Add(std::make_unique<MapView>(mapPtr));

    if (dynamic_cast<IDataset*>(dataSourcePtr) != nullptr)
    {
        render->Add(std::make_unique<DatasetView<T>>(dynamic_cast<IDataset*>(dataSourcePtr)));
    }

    render->Activate();
    pipeline->Activate();

    while (pipeline->IsActive())
    {
    }

    BlockTimer::PrintStat();

    render->Stop();

    std::cout << "Program end." << std::endl;

    return 0;
}