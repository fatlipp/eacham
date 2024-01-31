#pragma once

#include "sfm/data/Frame.h"
#include "base/tools/Tools2d.h"
#include <string>

namespace eacham::dataset
{

template<typename SourceT>
class SfmInputSource
{
public:
    SfmInputSource(SourceT&& source)
        : source{ std::move(source) }
        {}

    std::vector<Frame> GetAll(const int maxSize)
    {
        std::vector<Frame> frames;
        unsigned frameId = 0;
        while (source.HasNext())
        {
            source.Read();

            auto [ts, image, path] = source.Get();

            while (image.rows > 1500.0F)
                ResizeImage(image, 0.95);

            frames.emplace_back(frameId++, image.clone(), path);

            if (maxSize > 0 && frameId >= maxSize)
            {
                break;
            }
        }

        return frames;
    }
    
private:
    SourceT source;

};

}