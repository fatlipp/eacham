#pragma once

#include "pipeline/Pipeline.h"
#include "data_source/dataset/IDataset.h"

#include "performance/BlockTimer.h"

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
        if (this->frameId == 0)
        {
            this->odometry->SetPosition(std::get<1>(this->dataset->GetGtPose()));
        }

        const bool isProcessed = Pipeline<T>::Process();

        if (isProcessed)
        {
            std::cout << std::endl << "[=== DATASET SUMMARY ===]" << std::endl;
            {   
                BlockTimer timer("Pipeline::Dataset()");
                const auto currentPos = this->odometry->GetPosition();
                const auto gtPos = this->dataset->GetGtPose();

                const Eigen::Matrix4f diff = (currentPos - std::get<1>(gtPos));
                const float diffLen = std::sqrt(diff(0, 3) * diff(0, 3) + 
                                                diff(1, 3) * diff(1, 3) + 
                                                diff(2, 3) * diff(2, 3));

                std::cout << "Current pos:\n" << currentPos << std::endl;
                std::cout << "GT pos:\n" << std::get<1>(gtPos) << std::endl;
                std::cout << "Diff:\n" << diffLen << "\n" << diff << std::endl;
            }
        }
        else
        {
            this->Pause();
        }

        return isProcessed;
    }

    void CheckReset() override
    {
        Pipeline<T>::CheckReset();

        // TODO
        // this->dataset->Reset();
    }

private:
    IDataset* dataset;
};
    
} // namespace eacham
