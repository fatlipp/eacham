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
        std::cout << "Process() frameId: " << this->frameId << std::endl;

        bool isProcessed = PrepareNextFrame();

        if (this->frameId == 0)
        {
            this->odometry->SetPosition(this->dataset->GetGtPose());
        }

        {   
            BlockTimer timer("Pipeline::Process()");
            isProcessed = Pipeline<T>::Process();
        }

        if (isProcessed)
        {
            std::cout << "[=== DATASET SUMMARY ===]" << std::endl;
            {   
                const auto currentPos = this->odometry->GetPosition();
                const auto gtPos = this->dataset->GetGtPose();

                const Eigen::Matrix4f diff = (currentPos - gtPos);
                const float diffLen = std::sqrt(diff(0, 3) * diff(0, 3) + 
                                                diff(1, 3) * diff(1, 3) + 
                                                diff(2, 3) * diff(2, 3));

                std::cout << "Current pos:\n" << currentPos << std::endl;
                std::cout << "GT pos:\n" << gtPos << std::endl;
                std::cout << "Diff:\n" << diffLen << "\n" << diff << std::endl;
            }
            std::cout << "[===+===============+===]" << std::endl;
        }

        return isProcessed;
    }

private:
    bool PrepareNextFrame()
    {
        if (this->dataset != nullptr)
        {
            this->dataset->ReadNext();
            
            return true;
        }

        return false;
    }

private:
    IDataset* dataset;
};
    
} // namespace eacham
