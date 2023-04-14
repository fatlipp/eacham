#include "MotionEstimatorPnP.h"
#include "tools/Tools3d.h"

#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Geometry>

#include "motion_estimator/MotionEstimatorTools.h"

namespace eacham
{

MotionEstimatorPnP::MotionEstimatorPnP(const cv::Mat &cameraMatInp, const cv::Mat &distCoeffsInp)
    : MotionEstimatorBase()
{
    this->distCoeffs = distCoeffsInp;

    if (cameraMatInp.rows == 1)
    {
        cameraMat = cv::Mat::eye(3, 3, CV_32FC1);
        cameraMat.at<float>(0, 0) = cameraMatInp.at<float>(0, 0);
        cameraMat.at<float>(1, 1) = cameraMatInp.at<float>(0, 1);
        cameraMat.at<float>(0, 2) = cameraMatInp.at<float>(0, 2);
        cameraMat.at<float>(1, 2) = cameraMatInp.at<float>(0, 3);
        cameraMat.at<float>(2, 2) = 1.0f;
    }
    else
    {
        cameraMat = cameraMatInp;

        std::cerr << "NEED CHECK VALUES!!!!!!!!!\n";
    }
}

std::tuple<Eigen::Matrix4f, unsigned> MotionEstimatorPnP::Estimate(const Frame& frame1, Frame& frame2)
{
    const auto [pts1, pts2] = FindMatches(frame1, frame2);
    const auto matches = pts1.size();

    std::vector<cv::Point3f> pts3d1;
    std::vector<cv::Point2f> pts2d2; 

    //draw
    // std::vector<cv::KeyPoint> kp1;
    // std::vector<cv::KeyPoint> kp2;
    // std::vector<std::vector<cv::DMatch>> matchesGood;
    // int pointId = 0;

    std::vector<std::pair<int, int>> normIds;

    for (size_t i = 0; i < matches; ++i)
    {
        pts3d1.push_back(frame1.GetPointData(pts1[i]).position3d);
        pts2d2.push_back(frame2.GetPointData(pts2[i]).keypoint.pt);
        frame2.GetPointData(pts2[i]).associatedMapPointId = frame1.GetPointData(pts1[i]).associatedMapPointId;

        normIds.push_back(std::make_pair(pts1[i], pts2[i]));

        //draw
        // kp1.push_back(frame1.GetPointData(pts1[i]).keypoint);
        // kp2.push_back(frame2.GetPointData(pts2[i]).keypoint);
        // cv::DMatch mm2 = cv::DMatch(pointId, pointId, 0);
        // matchesGood.push_back({mm2});
        // ++pointId;
    }

    //draw
    // if (kp1.size() > 0)
    // {
    //     cv::Mat img_match;
    //     cv::drawMatches(frame1.GetImage(), kp1, frame2.GetImage(), kp2, matchesGood, img_match);
    //     cv::resize(img_match, img_match, {img_match.cols * 0.7f, img_match.rows * 0.7f});
    //     cv::imshow("ME: Matches", img_match);
    // }

    std::cout << "matches: " << matches << std::endl;

    Eigen::Affine3f motion = Eigen::Affine3f::Identity();
    unsigned inliersCount = 0;

    const int MIN_INLIERS = 5;

    if (matches > MIN_INLIERS)
    {
		cv::Mat rvec = cv::Mat_<double>(3, 1);
		cv::Mat tvec = cv::Mat_<double>(3, 1);

        std::vector<int> inliersPnP;
        cv::solvePnPRansac(pts3d1, pts2d2, cameraMat, distCoeffs, rvec, tvec, false, 100, 4.0f, 0.99f, inliersPnP, cv::SOLVEPNP_EPNP);
    
        std::cout << "inliers (pnp): " << inliersPnP.size() << " (" << (inliersPnP.size() / static_cast<float>(matches)) << ")" << std::endl;

        if (inliersPnP.size() >= MIN_INLIERS)
		{
            inliersCount = inliersPnP.size();

            cv::Mat Rmat1;
			cv::Rodrigues(rvec, Rmat1);
            // std::vector<int> reprojectedInliers;
            // const auto [errMean1, errVar1] = CalcReprojectionError(frame2.GetImage(), pts3d1, pts2d2, cameraMat, distCoeffs, Rmat1, tvec, 2.0f, reprojectedInliers);
            // std::cout << "inliers (reprojected): " << reprojectedInliers.size() << " (" << (reprojectedInliers.size() / static_cast<float>(matches)) << ")" << std::endl;
            // inliersCount = reprojectedInliers.size();
            

            for(unsigned int i = 0; i < inliersPnP.size(); ++i)
            {
                const auto pair = normIds[inliersPnP[i]];
                const int id1 = std::get<0>(pair);
                const int id2 = std::get<1>(pair);
            }

            const bool needRefine = false;

            if (needRefine)
            {
                cv::Mat refinedRvec = rvec;
                cv::Mat refinedTvec = tvec;

                std::vector<cv::Point3f> points3d1Inliers(inliersPnP.size());
                std::vector<cv::Point2f> points2d2Inliers(inliersPnP.size());
                for(unsigned int i=0; i<inliersPnP.size(); ++i)
                {
                    points3d1Inliers[i] = pts3d1[inliersPnP[i]];
                    points2d2Inliers[i] = pts2d2[inliersPnP[i]];
                }

                cv::solvePnP(points3d1Inliers, points2d2Inliers, cameraMat, distCoeffs, refinedRvec, refinedTvec, true, cv::SOLVEPNP_EPNP);

                cv::Mat RmatNew;
                cv::Rodrigues(refinedRvec, RmatNew);

                std::vector<int> refinedInliers;
                // const auto [errMean, errVar] = CalcReprojectionError(frame2.GetImage(), points3d1Inliers, points2d2Inliers, cameraMat, distCoeffs, RmatNew, refinedTvec, 2.0f, refinedInliers);
                // std::cout << "inliers (refined): " << refinedInliers.size() << " (" << (refinedInliers.size() / static_cast<float>(matches)) << ")" << std::endl;

                // if (refinedInliers.size() >= reprojectedInliers.size())
                // {
                //     rvec = refinedRvec;
                //     tvec = refinedTvec;
                //     inliersCount = refinedInliers.size();
                // }
            }


            cv::Mat Rmat;
			cv::Rodrigues(rvec, Rmat);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> R_Eigen(Rmat.ptr<double>(), Rmat.rows, Rmat.cols);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> T_Eigen(tvec.ptr<double>(), tvec.rows, tvec.cols);

            motion.linear() = R_Eigen.cast<float>();
            motion.translation() = T_Eigen.cast<float>();
            motion = motion.inverse();
		}

    }

    return {motion.matrix(), inliersCount};
}

}