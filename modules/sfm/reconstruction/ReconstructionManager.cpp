#include "sfm/reconstruction/ReconstructionManager.h"
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

MatchTwoView ReconstructionManager::RecoverPoseTwoView(const unsigned id1, const unsigned id2, 
    const cv::Mat& K) const
{
    MatchTwoView result;

    auto node1 = graph->Get(id1);
    auto node2 = graph->Get(id2);
    auto& factor = node1->GetFactor(node2->GetId());

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
    }

    cv::Mat mask2;
    const auto H = cv::findHomography(pts1, pts2, cv::LMEDS, 4.0, mask2, 100, 0.999);
    
    float H_Inliers = 0.0;
    
    {
        std::vector<cv::Point2f> points1_H;//(vec.size());
        std::vector<cv::Point2f> points2_H;//(vec.size());
        ApplyMask(pts1, pts2, points1_H, points2_H, mask2);

        H_Inliers = points1_H.size();
    }

    const float H_E_ratio = (H_Inliers > 0.0) ? (H_Inliers / E_Inliers) : 0;
    // std::cout << "H/E ratio: " << H_E_ratio << std::endl;

    if (H_E_ratio > 0.9)
    {
        std::vector<cv::Mat> Rs_decomp, ts_decomp, normals_decomp;
        const int solutions = cv::decomposeHomographyMat(H, K, Rs_decomp, ts_decomp, normals_decomp);
        
        std::vector<std::pair<unsigned, Eigen::Vector3d>> bestMatches;
        Eigen::Matrix4d bestTransform;
        unsigned bestNum = 0;

        for (int i = 0; i < solutions; ++i)
        {
            std::vector<std::pair<unsigned, Eigen::Vector3d>> matches;

            cv::Mat R = Rs_decomp[i];
            cv::Mat t = ts_decomp[i];

            const auto transform = ConvertToTransform(R, t); 

            for (size_t j = 0; j < factor.matches.size(); ++j)
            {
                const auto& [m1, m2] = factor.matches[j];

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


                if (reprojectionError < maxReprError &&
                    TriangulationAngle(Eigen::Matrix4d::Identity(), transform, point3d) > minTriAngle)
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

        if (bestMatches.size() > 20)
        {
            for (const auto& [matchId, p3d] : bestMatches)
            {
                result.matches.push_back({factor.matches[matchId].id1, factor.matches[matchId].id2, p3d});
            }

            result.transform = bestTransform;
        }
    }
    else
    {
        cv::Mat R;
        cv::Mat t;
        const int goodPoints = cv::recoverPose(E, pts1, pts2, K, R, t, 50.0f, mask);
        const auto transform = ConvertToTransform(R, t);

        for (size_t i = 0; i < factor.matches.size(); ++i)
        {
            const auto& [m1, m2] = factor.matches[i];

            const auto v1 = node1->GetKeyPoint(m1);
            const Eigen::Vector2d p1 = Eigen::Vector2d{v1.x, v1.y};

            const auto v2 = node2->GetKeyPoint(m2);
            const Eigen::Vector2d p2 = Eigen::Vector2d{v2.x, v2.y};

            const auto point3d = TriangulatePoint(p1, p2, K, transform);

            if (point3d.z() <= 0.0F || TriangulationAngle(Eigen::Matrix4d::Identity(), transform, 
                point3d) < minTriAngle)
            {
                continue;
            }

            const auto manualProj = tools::project3dPoint({point3d.x(), point3d.y(), point3d.z()}, K);
            const float reprojectionError = std::sqrt(std::pow(p1.x() - manualProj.x, 2) + 
                std::pow(p1.y() - manualProj.y, 2));

            if (reprojectionError < maxReprError)
            {
                result.matches.push_back({factor.matches[i].id1, factor.matches[i].id2, point3d});
            }
        }

        result.transform = transform;
    }

    return result;
}

bool ReconstructionManager::RecoverPosePnP(const unsigned id1, 
    const unsigned id2, const cv::Mat& K)
{
    auto node1 = graph->Get(id1);
    auto node2 = graph->Get(id2);
    auto& factor = node1->GetFactor(node2->GetId());

    std::cout << "RecoverPosePnP: " << node1->GetId() << " -> " << node2->GetId() << 
        ", frameMatches: " << factor.matches.size() << ":\n";

    std::vector<cv::Point3f> pts3d1;
    std::vector<cv::Point2f> pts2d1;
    std::vector<cv::Point2f> pts2d2;

    unsigned counts = 0; 
    for (const auto& m : factor.matches)
    {
        if (node1->HasPoint3d(m.id1))
        {
            const auto point3d = map->Get(node1->GetPoint3d(m.id1));
            pts3d1.push_back({point3d.x(), point3d.y(), point3d.z()});
            pts2d1.push_back(node1->GetKeyPoint(m.id1));
            pts2d2.push_back(node2->GetKeyPoint(m.id2));
        }

        ++counts;
    }

    // DrawMatches("HH", node1->GetImage(), node2->GetImage(), pts2d1, pts2d2, 0, {});

    std::cout << "Prepared for PnP matches count: " << pts2d2.size() 
              << " (of " <<  counts << ")\n";

    if (pts2d2.size() < minPnpInliers)
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
        (inliersPnP.size() / static_cast<float>(pts2d2.size())) << ")\n";

    if (inliersPnP.size() < minPnpInliers)
    {
        return false;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    factor.transform = ConvertToTransform(R, t);
    node2->SetTransform(factor.transform);
    node2->SetValid(true);

    return true;
}

} // namespace eacham