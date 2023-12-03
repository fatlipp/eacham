#include "sfm/utils/Triangulator.h"
#include "sfm/utils/ProjectionHelper.h"

#include "base/tools/Tools3d.h"
#include "sfm/data/Map.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Eigenvalues> 

#include <iostream>
#include <random>

namespace eacham
{

Eigen::Vector3d TriangulatePoint(const Eigen::Matrix4d& cam1Pos,
                                 const Eigen::Matrix4d& cam2Pos,
                                 const Eigen::Vector2d& point1,
                                 const Eigen::Vector2d& point2)
{
  Eigen::Matrix4d A;
  A.row(1) = point1.x() * cam1Pos.row(2) - cam1Pos.row(0);
  A.row(0) = point1.y() * cam1Pos.row(2) - cam1Pos.row(1);
  A.row(3) = point2.x() * cam2Pos.row(2) - cam2Pos.row(0);
  A.row(2) = point2.y() * cam2Pos.row(2) - cam2Pos.row(1);

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);

  return svd.matrixV().col(3).hnormalized();
}

Eigen::Vector3d TriangulatePoint(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const cv::Mat& K,
    const Eigen::Matrix4d& transform1, const Eigen::Matrix4d& transform2)
{
    Eigen::Vector2d p1n; 
    p1n.x() = (p1.x() - K.at<double>(0, 2)) / K.at<double>(0, 0);
    p1n.y() = (p1.y() - K.at<double>(1, 2)) / K.at<double>(1, 1);

    Eigen::Vector2d p2n; 
    p2n.x() = (p2.x() - K.at<double>(0, 2)) / K.at<double>(0, 0);
    p2n.y() = (p2.y() - K.at<double>(1, 2)) / K.at<double>(1, 1);

    return TriangulatePoint(transform1, transform2, p1n, p2n);
}

Eigen::Vector3d TriangulatePoint(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const cv::Mat& K,
    const Eigen::Matrix4d& transform)
{
    return TriangulatePoint(p1, p2, K, Eigen::Matrix4d::Identity(), transform);
}

bool IsPositiveDepth(const Eigen::Matrix4d& transform,
                           const Eigen::Vector3d& point) {
  return transform.row(2).dot(point.homogeneous()) >=
         std::numeric_limits<double>::epsilon();
}

bool TriangulatePointRansac(const std::vector<EstimatorData>& data, 
    Eigen::Vector3d& point3d, std::vector<unsigned>& inliers, const float maxErr)
{
    const auto size = data.size();

    if (size < 2)
    {
        return false;
    }

    if (size < 3)
    {
        point3d = TriangulatePoint(data[0].point2d, data[1].point2d, data[0].K, data[0].transform, data[1].transform);

        for (const auto& pt : data)
        {
            const auto pos = tools::transformPoint3d(point3d, pt.transform);
            const auto err = CalcReprojectionError({pt.point2d.x(), pt.point2d.y()}, {pos.x(), pos.y(), pos.z()}, pt.K);

            if (err < maxErr && IsPositiveDepth(pt.transform, point3d))
            {
                inliers.push_back(1);
            }
            else
            {
                inliers.push_back(0);
            }
        }

        return point3d.z() > 0.0f;
    }

    // std::random_device dev;
    // std::mt19937 rng(dev());
    // std::uniform_int_distribution<std::mt19937::result_type> dist6(0, size - 1); // distribution in range [1, 6]
    // const auto r1 = static_cast<unsigned>(dist6(rng));

    unsigned bestInl = 0;
    std::vector<unsigned> bestInliers;

    std::vector<std::pair<unsigned, unsigned>> usedPairs;

    // TODO: RANSAC??
    unsigned r1 = 0;
    unsigned r2 = 1;

    while (true)
    {
        point3d = TriangulatePoint(data[r1].point2d, data[r2].point2d, data[0].K, 
            data[r1].transform, data[r2].transform);

        unsigned inl = 0;
        std::vector<unsigned> inliersLocal;

        for (const auto& pt : data)
        {
            const auto pos = tools::transformPoint3d(point3d, pt.transform);
            const auto err = CalcReprojectionError({pt.point2d.x(), pt.point2d.y()}, {pos.x(), pos.y(), pos.z()}, pt.K);

            if (err < maxErr && IsPositiveDepth(pt.transform, point3d))
            {
                inliersLocal.push_back(1);

                ++inl;
            }
            else
            {
                inliersLocal.push_back(0);
            }
        }
        
        if (inl > bestInl)
        {
            bestInl = inl;
            bestInliers = inliersLocal;
        }

        ++r2;

        if (r2 == data.size())
        {
            ++r1;
            r2 = r1 + 1;

            if (r2 >= data.size())
            {
                break;
            }
        }
    }

    inliers = bestInliers;

    return point3d.z() > 0.0f && bestInl > 2;// && bestProjErr < maxErr * data.size();
}

void TriangulateFrame(const unsigned frameId, std::shared_ptr<graph_t> graph, 
    std::shared_ptr<Map> map, const cv::Mat& K, const unsigned minObservers)
{
    static int seed = 12345;
    seed += 10;
    cv::RNG rng(seed);
    Eigen::Vector3d color = {rng.uniform(0, 255), 
                            rng.uniform(0, 255), 
                            rng.uniform(0, 255)};
    color /= 255.0;

    auto currentNode = graph->Get(frameId);

    std::map<unsigned, std::map<unsigned, unsigned>> observersFull;

    const auto& factors = currentNode->GetFactors();

    for (const auto& [id, factor] : factors)
    {
        if (!graph->Get(id)->IsValid())
        {
            continue;
        }

        unsigned addd = 0;
        
        for (const auto m : factor.matches)
        {
            if (graph->Get(id)->HasPoint3d(m.id2) 
                && 
                map->GetObservers(graph->Get(id)->GetPoint3d(m.id2)).size() > 2
                )
            {
                const auto pt2d = graph->Get(frameId)->GetKeyPoint(m.id1);
                const auto pt3d = tools::transformPoint3d(
                            map->Get(graph->Get(id)->GetPoint3d(m.id2)),
                            currentNode->GetTransform());

                const auto err = CalcReprojectionError(pt2d, {pt3d.x(), pt3d.y(), pt3d.z()}, K);

                if (err < 4.0f)
                {
                    graph->Get(frameId)->SetPoint3d(m.id1, graph->Get(id)->GetPoint3d(m.id2));
                    map->AddObserver(frameId, m.id1, graph->Get(id)->GetPoint3d(m.id2));
                    continue;
                }
            }

            observersFull[m.id1][frameId] = m.id1;
            observersFull[m.id1][id] = m.id2;
            ++addd;
        }
        std::cout << "--- " << id << ") factor added: " << addd << " of " << factor.matches.size() << std::endl;
    }
    std::cout << "observersFull: " << observersFull.size() << std::endl;

    unsigned total = 0;
    unsigned added = 0;

    for (const auto& [_, observers] : observersFull)
    {
        if (observers.size() < minObservers)
        {
            continue;
        }

        std::vector<EstimatorData> datas;

        for (const auto& [frame, pointId2] : observers)
        {
            EstimatorData pointData;
            pointData.point2d = graph->Get(frame)->GetKeyPointEigen(pointId2);
            pointData.transform = graph->Get(frame)->GetTransform();
            pointData.K = K;

            datas.push_back(pointData);
        }

        Eigen::Vector3d point3d;
        std::vector<unsigned> mask;
        if (TriangulatePointRansac(datas, point3d, mask, 4.0f) && mask.size() > 0)
        {
            if (mask.size() != datas.size())
            {
                std::bad_exception();
            }

            const int inliersCount = std::accumulate(mask.begin(), mask.end(), 0);

            if (inliersCount == mask.size())
            {
                const auto mapPointId = map->Add(point3d, color);

                for (const auto& [frame, pointId2] : observers)
                {
                    if (graph->Get(frame)->HasPoint3d(pointId2))
                    {
                        map->RemoveObserver(frame, pointId2, graph->Get(frame)->GetPoint3d(pointId2));
                        map->UpdateStatus(graph->Get(frame)->GetPoint3d(pointId2), false);
                    }

                    graph->Get(frame)->SetPoint3d(pointId2, mapPointId);
                    map->AddObserver(frame, pointId2, mapPointId);
                }
                map->UpdateStatus(mapPointId, true);

                ++added;
            }
        }

        ++total;
    }

    std::cout << "added: " << added << ", total: " << total << std::endl;
}

}