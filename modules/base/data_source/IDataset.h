#pragma once

#include <tuple>
#include <utility>

namespace eacham
{

template <typename CameraT, typename GtT>
class IDataset
{
public:
    IDataset(std::unique_ptr<CameraT> camera, std::unique_ptr<GtT> groundTruth)
        : camera(std::move(camera))
        , groundTruth(std::move(groundTruth))
        , isDataAvailable{false}
    {
        std::cout << "cam: " << typeid(CameraT).name() << "\n";
    }

public:
    std::pair<typename CameraT::ReturnType, typename GtT::ReturnType> Get() const
    {
        if (!isDataAvailable)
        {
            std::cout << "IDataset() Data is not available...\n";
        }

        return { currentCamData, gtData };
    }

    void PrepareNext()
    {
        camera->Read();

        currentCamData = camera->Get();
        gtData = groundTruth->Get();

        isDataAvailable = true;
    }

    bool HasNext() const
    {
        return camera->HasNext();
    }

protected:
    std::unique_ptr<CameraT> camera;
    std::unique_ptr<GtT> groundTruth;

    std::atomic<bool> isDataAvailable;

    typename CameraT::ReturnType currentCamData;
    typename GtT::ReturnType gtData;
};

}