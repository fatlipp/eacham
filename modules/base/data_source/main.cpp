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
    using T = stereodata_t;

    // specific for visual sensor
    VisualDataSourceDirector<T> dataSourceDirector;
    auto dataSource = dataSourceDirector.Build(config);

    if (dataSource == nullptr)
    {
        std::cerr << "dataSource is null" << std::endl;

        return 1;
    }

    while (true)
    {
        // const auto data = dataSource->Get();
    }

    return 0;
}