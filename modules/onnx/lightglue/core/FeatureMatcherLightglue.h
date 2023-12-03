#pragma once

#include "core/features/IFeatureMatcher.h"
#include "./Types.h"

namespace eacham
{

class FeatureMatcherLightglue : public IFeatureMatcher<matcher_t>
{
    
public:
    MatchType Match(const matcher_t& descriptor1, 
              const matcher_t& descriptor2) override;
};

}