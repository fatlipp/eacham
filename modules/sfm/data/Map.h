#pragma once

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <Eigen/Core>

#include <map>
#include <unordered_map>
#include <iostream>
#include <string>

namespace eacham
{

struct MapPointData
{
    unsigned id;
    Eigen::Vector3d point3d;
    Eigen::Vector3d color;
    bool isValid = false;

    std::unordered_map<unsigned, unsigned> observers;
};

class Map
{
public:
    Map()
        : mapPointId(0)
    { }

public:
    bool CheckExists(const unsigned id)
    {
        std::lock_guard<std::mutex> lock(mutex);
        return points.find(id) != points.end();
    }

    unsigned Add(const Eigen::Vector3d& point)
    {
        std::lock_guard<std::mutex> lock(mutex);
        ++mapPointId;

        points.insert({mapPointId, {mapPointId, point, {0, 255, 0}, false}});

        return mapPointId;
    }

    unsigned Add(const Eigen::Vector3d& point, const Eigen::Vector3d& color)
    {
        std::lock_guard<std::mutex> lock(mutex);

        ++mapPointId;

        points.insert({mapPointId, {mapPointId, point, color, false}});

        return mapPointId;
    }

    void UpdatePoint(const unsigned id, const Eigen::Vector3d& point)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(id);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::UpdatePoint() point %i is not found", id));
        }

        points[id].point3d = point;
    }

    void UpdateColor(const unsigned id, const Eigen::Vector3d& color)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(id);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::UpdateColor() point %i is not found", id));
        }

        points[id].color = color;
    }

    void UpdateStatus(const unsigned id, const bool isValid)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(id);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::UpdateStatus() point %i is not found", id));
        }

        points[id].isValid = isValid;
    }

    Eigen::Vector3d Get(const unsigned id) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(id);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::Get() point %i is not found", id));
        }
        
        return iter->second.point3d;
    }

    bool GetStatus(const unsigned id) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(id);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::GetStatus() point %i is not found", id));
        }
        
        return iter->second.isValid;
    }

    void AddObserver(const unsigned frame, const unsigned point2d,
        const unsigned point3d)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(point3d);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::AddObserver() point2d %i is not found", point2d));
        }

        iter->second.observers[frame] = point2d;

        const auto iter2 = points.find(point3d);

        if (iter2->second.observers[frame] != point2d)
        {
            throw std::runtime_error(
                cv::format("Map::AddObserver() point3d %i is not found", point3d));
        }
    }

    void RemoveObserver(const unsigned frame, const unsigned point2d,
        const unsigned point3d)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(point3d);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::RemoveObserver() point3d %i is not found", point3d));
        }

        if (iter->second.observers.find(frame) == iter->second.observers.end())
        {
            return;
        }

        iter->second.observers.erase(frame);
    }

    std::unordered_map<unsigned, unsigned> GetObservers(const unsigned id) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = points.find(id);

        if (iter == points.end())
        {
            throw std::runtime_error(
                cv::format("Map::GetObservers() point %i is not found", id));
        }
        
        return iter->second.observers;
    }

    const std::unordered_map<unsigned, MapPointData>& GetAll() const
    {
        return points;
    }

    void lock()
    {
        mutex.lock();
    }

    void unlock()
    {
        mutex.unlock();
    }

public:
    std::unordered_map<unsigned, MapPointData> points;

private:
    unsigned mapPointId;
    mutable std::mutex mutex;
};



}