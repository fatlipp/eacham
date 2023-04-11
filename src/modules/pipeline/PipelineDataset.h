#pragma once

#include "pipeline/Pipeline.h"
#include "data_source/dataset/IDataset.h"

namespace eacham
{

template<typename T>
class PipelineDataset : public Pipeline<T>
{
public:
    PipelineDataset(const ConfigGeneral& config)
        : Pipeline<T>(config) 
        {}

public:
    void SetDataSource(std::unique_ptr<IDataSource<T>> dataSourceInp) override
    {
        Pipeline<T>::SetDataSource(std::move(dataSourceInp));
        
        this->dataset = dynamic_cast<IDataset*>(this->dataSource.get());
    }

protected:
    bool Process() override
    {
        PrepareNextFrame();
        return Pipeline<T>::Process();
    }

private:
    void PrepareNextFrame()
    {
        this->dataset->ReadNext();
    }

private:
    IDataset* dataset;
};
    
} // namespace eacham
