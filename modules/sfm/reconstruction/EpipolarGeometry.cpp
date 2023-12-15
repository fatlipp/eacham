#include "sfm/reconstruction/EpipolarGeometry.h"
#include "sfm/reconstruction/ProjectionHelper.h"
#include "sfm/reconstruction/Triangulator.h"

#include "sfm/data/Map.h"
#include "sfm/view/Gui.h"

#include "base/tools/Tools3d.h"
#include "base/tools/Tools2d.h"

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <unordered_map>

namespace eacham
{

std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> GetMatchedPoints(
    node_t* node1, node_t* node2)
{
    const auto& factor = node1->GetFactor(node2->GetId());

    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for (const auto& m : factor.matches)
    {
        points1.push_back(node1->GetKeyPoint(m.id1));
        points2.push_back(node2->GetKeyPoint(m.id2));
    }

    return {points1, points2};
}

void ApplyMask(
    const std::vector<cv::Point2f>& v1, const std::vector<cv::Point2f>& v2, 
    std::vector<cv::Point2f>& v1out, std::vector<cv::Point2f>& v2out, 
    const cv::Mat& mask)
{
    for (size_t i = 0; i < v1.size(); ++i)
    {
        if (mask.at<unsigned char>(i) == 1)
        {
            v1out.push_back(v1[i]);
            v2out.push_back(v2[i]);
        }
    }
}

std::vector<Match> CreateMatches(
    const std::vector<cv::Point2f>& v1, const std::vector<cv::Point2f>& v2, 
    const std::vector<std::pair<unsigned, unsigned>>& matches,
    const cv::Mat& mask,
    const Eigen::Matrix4d& transform,
    const cv::Mat& K,
    std::shared_ptr<Map> map)
{
    std::vector<Match> output;

    for (size_t i = 0; i < v1.size(); ++i)
    {
        if (mask.at<unsigned char>(i) == 1)
        {
            const Eigen::Vector2d p1 = Eigen::Vector2d{v1[i].x, v1[i].y};
            const Eigen::Vector2d p2 = Eigen::Vector2d{v2[i].x, v2[i].y};
            auto point3d = TriangulatePoint(p1, p2, K, transform);

            if (point3d.z() <= 0.0)
            {
                continue;
            }

            const auto manualProj = tools::project3dPoint({point3d.x(), point3d.y(), point3d.z()}, K);
            const float reprojectionError = std::sqrt(std::pow(v1[i].x - manualProj.x, 2) + 
                std::pow(v1[i].y - manualProj.y, 2));

            if (reprojectionError > 3.5F)
            {
                continue;
            }

            const auto mapPointId = map->Add(point3d, {1, 0.5, 0.4});

            output.push_back({
                    .id1 = matches[i].first, 
                    .id2 = matches[i].second,
                    .triangulatedPointId = mapPointId
                });
        }
    }

    return output;
}

void RecoverPoseTwoView(const unsigned id1, const unsigned id2, 
    std::shared_ptr<graph_t> graph, const cv::Mat& K,
    std::shared_ptr<Map> map, const float reprErrMax)
{
    auto node1 = graph->Get(id1);
    auto node2 = graph->Get(id2);
    auto& factor = node1->GetFactor(node2->GetId());

    // std::cout << "RecoverPoseTwoView: " << node1->GetId() << " -> " << node2->GetId() << 
    //     ", frameMatches: " << factor.matches.size() << ":\n";

    const auto& [pts1, pts2] = GetMatchedPoints(node1, node2);

    cv::Mat mask; // CV_8U
    auto E = cv::findEssentialMat(pts1, pts2, 
        K.at<double>(0, 0), 
        cv::Point2f{K.at<double>(0, 2), K.at<double>(1, 2)}, cv::LMEDS, 0.99, 4.0, 1000, mask);

    float E_Inliers = 0.0;

    {
        std::vector<cv::Point2f> points1_H;//(vec.size());
        std::vector<cv::Point2f> points2_H;//(vec.size());
        ApplyMask(pts1, pts2, points1_H, points2_H, mask);

        E_Inliers = points1_H.size();

        // std::cout << ", points1_E: " << E_Inliers;
        // std::cout << ", E_inliers_ratio: " << E_Inliers / static_cast<float>(pts1.size());
    }

    cv::Mat mask2;
    const auto H = cv::findHomography(pts1, pts2, cv::LMEDS, 4.0, mask2, 100, 0.999);
    
    float H_Inliers = 0.0;
    
    {
        std::vector<cv::Point2f> points1_H;//(vec.size());
        std::vector<cv::Point2f> points2_H;//(vec.size());
        ApplyMask(pts1, pts2, points1_H, points2_H, mask2);

        H_Inliers = points1_H.size();

        // std::cout << "; points1_H: " << H_Inliers;
        // std::cout << ", H_inliers_ratio: " << H_Inliers / static_cast<float>(pts1.size()) << std::endl;
    }

    const float H_E_ratio = (H_Inliers > 0.0) ? (H_Inliers / E_Inliers) : 0;
    // std::cout << "H/E ratio: " << H_E_ratio << std::endl;

    if (H_E_ratio > 0.9)
    {
        std::vector<cv::Mat> Rs_decomp, ts_decomp, normals_decomp;
        const int solutions = cv::decomposeHomographyMat(H, K, Rs_decomp, ts_decomp, normals_decomp);
        
        std::vector<std::pair<unsigned, Eigen::Vector3d>> bestMatches;
        Eigen::Matrix4d bestTransform;
        unsigned bestNum = 99;
        // std::cout << "recoverPose by H, solutions: " << solutions << std::endl;

        for (int i = 0; i < solutions; ++i)
        {
            std::vector<std::pair<unsigned, Eigen::Vector3d>> matches;

            cv::Mat R = Rs_decomp[i];
            cv::Mat t = ts_decomp[i];

            const auto transform = ConvertToTransformInv(R, t); 

            for (size_t j = 0; j < factor.matches.size(); ++j)
            {
                const auto& [m1, m2, triangulatedPointId] = factor.matches[j];

                const auto p1 = node1->GetKeyPointEigen(m1);
                const auto p2 = node2->GetKeyPointEigen(m2);

                auto point3d = TriangulatePoint(p1, p2, K, transform);

                if (point3d.z() <= 0.0F)
                {
                    continue;
                }

                const auto manualProj = tools::project3dPoint({point3d.x(), point3d.y(), point3d.z()}, K);
                float reprojectionError = std::sqrt(std::pow(p1.x() - manualProj.x, 2) + 
                    std::pow(p1.y() - manualProj.y, 2));


                if (CheckTriangulationAngle(Eigen::Matrix4d::Identity(), transform, 
                    point3d, 4.0) 
                && (reprojectionError < reprErrMax))
                {
                    matches.push_back({j, point3d});
                }
            }

            if (matches.size() > bestMatches.size())
            {
                bestMatches = matches;
                bestTransform = transform;
                bestNum = i;
            }
        }

        // std::cout << "Best solution = " << bestNum << ", bestMatches: " << bestMatches.size() << std::endl;

        if (bestMatches.size() > 20)
        {
            for (const auto& [matchId, p3d] : bestMatches)
            {
                factor.matches[matchId].triangulatedPointId = map->Add(p3d, {0.3, 0.3, 0.3});
            }

            factor.quality = factor.matches.size();
            factor.transform = bestTransform;
        }
        else
        {
            factor.quality = 0;
        }

    }
    else
    {
        // std::cout << "recoverPose by E()\n";

        cv::Mat R;
        cv::Mat t;
        const int goodPoints = cv::recoverPose(E, pts1, pts2, K, R, t, 50.0f, mask);

        const auto transform = ConvertToTransformInv(R, t);
        unsigned goodReprojections = 0;

        for (size_t i = 0; i < factor.matches.size(); ++i)
        {
            const auto& [m1, m2, triangulatedPointId] = factor.matches[i];

            const auto v1 = node1->GetKeyPoint(m1);
            const Eigen::Vector2d p1 = Eigen::Vector2d{v1.x, v1.y};

            const auto v2 = node2->GetKeyPoint(m2);
            const Eigen::Vector2d p2 = Eigen::Vector2d{v2.x, v2.y};

            auto point3d = TriangulatePoint(p1, p2, K, transform);

            if (point3d.z() <= 0.0F)
            {
                continue;
            }

            const auto manualProj = tools::project3dPoint({point3d.x(), point3d.y(), point3d.z()}, K);
            const float reprojectionError = std::sqrt(std::pow(p1.x() - manualProj.x, 2) + 
                std::pow(p1.y() - manualProj.y, 2));

            if (CheckTriangulationAngle(Eigen::Matrix4d::Identity(), transform, 
                point3d, 2.0) 
                && (reprojectionError < reprErrMax))
            {
                factor.matches[i].triangulatedPointId = map->Add(point3d, {0.3, 0, 0.3});

                ++goodReprojections;
            }
        }

        factor.transform = transform;
        factor.quality = goodReprojections;
    }

    // std::cout << "factor.qulity: " << factor.quality << std::endl;
}

bool RecoverPosePnP(
    const unsigned id1, const unsigned id2,
    std::shared_ptr<graph_t> graph, 
    std::shared_ptr<Map> map, const cv::Mat& K)
{
    auto node1 = graph->Get(id1);
    auto node2 = graph->Get(id2);
    auto& factor = node1->GetFactor(node2->GetId());

    std::cout << "RecoverPosePnP: " << node1->GetId() << " -> " << node2->GetId() << 
        ", frameMatches: " << factor.matches.size() << ":\n";

    std::vector<cv::Point3f> pts3d1;
    std::vector<cv::Point2f> pts2d2;

    unsigned counts = 0; 
    for (const auto& m : factor.matches)
    {
        if (node1->HasPoint3d(m.id1))
        {
            const auto point3d = map->Get(node1->GetPoint3d(m.id1));
            pts3d1.push_back({point3d.x(), point3d.y(), point3d.z()});

            pts2d2.push_back(node2->GetKeyPoint(m.id2));
        }

        ++counts;
    }
    std::cout << "Prepared for PnP matches count: " << pts2d2.size() 
              << " (of " <<  counts << ")" << std::endl;

    const int MIN_INLIERS = 10;
    
    if (pts2d2.size() < MIN_INLIERS)
    {
        return false;
    }

    std::vector<double> distCoeffs = {0, 0, 0, 0 };
    cv::Mat rvec = cv::Mat_<double>(3, 1);
    cv::Mat t = cv::Mat_<double>(3, 1);

    std::vector<int> inliersPnP;
    cv::solvePnPRansac(pts3d1, pts2d2, K, distCoeffs, rvec, t, false, 
        10000, 4.0f, 0.999f, inliersPnP, cv::SOLVEPNP_EPNP);
    std::cout << "inliers (pnp): " << inliersPnP.size() << " (" << 
        (inliersPnP.size() / static_cast<float>(pts2d2.size())) << ")" << std::endl;

    if (inliersPnP.size() < MIN_INLIERS)
    {
        return false;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    factor.transform = ConvertToTransformInv(R, t);
    node2->SetTransform(factor.transform);
    node2->SetValid(true);

    return true;
}

} // namespace eacham