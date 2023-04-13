#pragma once

#include "data_source/dataset/IDataset.h"
#include "visualization/IDrawable.h"
#include "visualization/view/ViewTools.h"

namespace eacham
{

template<typename T>
class DatasetView : public IDrawable
{
public:
    DatasetView(IDataset* dataset)
        : dataset(dataset)
    {
    }

public:
    void Draw() override
    {   
        view_tools::DrawCamera(dataset->GetGtPose(), Eigen::Vector3f{0, 1, 0});
    }

private:
    IDataset* const dataset;
    
};

}