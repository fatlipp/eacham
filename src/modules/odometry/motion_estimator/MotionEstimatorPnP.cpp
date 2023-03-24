#include "MotionEstimatorPnP.h"
#include "tools/Tools3d.h"
#include "odometry/features/FeatureExtractor.h"

#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Geometry>

namespace eacham
{

MotionEstimatorPnP::MotionEstimatorPnP(const FeatureExtractorType &featureExtractor, const cv::Mat &cameraMatInp, const cv::Mat &distCoeffsInp)
    : MotionEstimatorBase(featureExtractor)
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

        this->cameraMatOneDim = cameraMatInp.clone();
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
    std::vector<int> iddds; 

    //draw
    std::vector<cv::KeyPoint> kp1;
    std::vector<cv::KeyPoint> kp2;
    std::vector<std::vector<cv::DMatch>> matchesGood;
    int pointId = 0;

    for (size_t i = 0; i < matches; ++i)
    {
        pts3d1.push_back(frame1.GetPointData(pts1[i]).position3d);
        pts2d2.push_back(frame2.GetPointData(pts2[i]).keypoint.pt);
        frame2.GetPointData(pts2[i]).associatedMapPointId = frame1.GetPointData(pts1[i]).associatedMapPointId;

        iddds.push_back(pts2[i]);

        //draw
        kp1.push_back(frame1.GetPointData(pts1[i]).keypoint);
        kp2.push_back(frame2.GetPointData(pts2[i]).keypoint);
        cv::DMatch mm2 = cv::DMatch(pointId, pointId, 0);
        matchesGood.push_back({mm2});
        ++pointId;
    }

    //draw
    if (kp1.size() > 0)
    {
        cv::Mat img_match;
        cv::drawMatches(frame1.GetImage(), kp1, frame2.GetImage(), kp2, matchesGood, img_match);
        cv::resize(img_match, img_match, {img_match.cols * 0.7f, img_match.rows * 0.7f});
        cv::imshow("ME: Matches", img_match);
    }

    std::cout << "matches: " << matches << std::endl;

    Eigen::Affine3f motion = Eigen::Affine3f::Identity();
    unsigned inliersCount = 0;

    const int MIN_INLIERS = 20;

    if (matches> MIN_INLIERS)
    {
		cv::Mat rvec = cv::Mat_<double>(3, 1);
		cv::Mat tvec = cv::Mat_<double>(3, 1);

        std::vector<int> inliers;
        cv::solvePnPRansac(pts3d1, pts2d2, cameraMat, distCoeffs, rvec, tvec, false, 1000, 4.0f, 0.99f, inliers, cv::SOLVEPNP_EPNP);
    
        std::cout << "inliers: " << inliers.size() << " (" << (inliers.size() / static_cast<float>(matches)) << ")" << std::endl;

        if (inliers.size() >= MIN_INLIERS)
		{
            inliersCount = inliers.size();

            cv::Mat Rmat;
			cv::Rodrigues(rvec, Rmat);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> R_Eigen(Rmat.ptr<double>(), Rmat.rows, Rmat.cols);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> T_Eigen(tvec.ptr<double>(), tvec.rows, tvec.cols);

            motion.linear() = R_Eigen.cast<float>();
            motion.translation() = T_Eigen.cast<float>();
            motion = motion.inverse();

            CalcReprojectionError(frame2.GetImage(), pts3d1, pts2d2, Rmat, tvec);

            for (int i = 0; i < inliers.size(); ++i)
            {
                frame2.GetPointData(iddds[inliers[i]]).uncertatinty = 1.0f;
            }
		}

    }

    return {motion.matrix(), inliersCount};
}

}