#pragma once

#include <vector>

namespace eacham
{

template<typename T>
class IFeatureMatcher
{
public:
    using MatchType = std::vector<std::pair<unsigned, unsigned>>;

public:
    virtual ~IFeatureMatcher() = default;

public:
    virtual MatchType 
    Match(const T& descriptor1, const T& descriptor2) = 0;
};

}