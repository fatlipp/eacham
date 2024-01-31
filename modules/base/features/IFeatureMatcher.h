#pragma once

#include <unordered_map>

namespace eacham
{

template<typename T>
class IFeatureMatcher
{
public:
    using MatchType = std::unordered_map<unsigned, unsigned>;

public:
    virtual ~IFeatureMatcher() = default;

public:
    virtual MatchType 
    Match(const T& descriptor1, const T& descriptor2) = 0;
};

}