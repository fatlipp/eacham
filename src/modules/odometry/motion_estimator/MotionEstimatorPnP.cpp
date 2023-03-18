#include "MotionEstimatorPnP.h"
#include "tools/Tools3d.h"
#include "odometry/features/FeatureExtractor.h"

#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Geometry>

namespace eacham
{

MotionEstimatorPnP::MotionEstimatorPnP()
{
}

MotionEstimatorPnP::MotionEstimatorPnP(const cv::Mat &cameraMatInp, const cv::Mat &distCoeffs)
    : distCoeffs(distCoeffs)
{
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
    const cv::Mat descriptor1 = frame1.GetDescriptors().clone();
    const cv::Mat descriptor2 = frame2.GetDescriptors().clone();

    const static matcher_t mather = FeatureExtractor::GetMatcher();

    std::vector<std::vector<cv::DMatch>> matches;
    mather->knnMatch(descriptor1, descriptor2, matches, 2);

    if (matches.size() < 1)
    {
        return {Eigen::Matrix4f::Identity(), 0};
    }


    std::vector<cv::Point3f> pts3d1; 
    std::vector<FramePoint3d> pts3d1_data; 
    std::vector<cv::Point2f> pts2d2; 
    std::vector<PointData> pts3d2; 

    std::vector<cv::KeyPoint> kp1;
    std::vector<cv::KeyPoint> kp2;
    std::vector<std::vector<cv::DMatch>> matchesGood;

    int pointId = 0;

    for (const auto& m : matches)
    {
        if (m[0].distance < 0.7f * m[1].distance)
        {
            pts3d1.push_back(frame1.GetPoint3d(m[0].queryIdx).position);
            pts3d1_data.push_back(frame1.GetPoint3d(m[0].queryIdx));
            pts2d2.push_back(frame2.GetFeature(m[0].trainIdx).pt);
            pts3d2.push_back(frame2.GetPointData(m[0].trainIdx));

            kp1.push_back(frame1.GetFeature(m[0].queryIdx));
            kp2.push_back(frame2.GetFeature(m[0].trainIdx));
            cv::DMatch mm2 = cv::DMatch(pointId, pointId, 0);
            matchesGood.push_back({mm2});
            ++pointId;
        }
    }

    if (kp1.size() > 0)
    {
        cv::Mat img_match;
        cv::drawMatches(frame1.GetImage(), kp1, frame2.GetImage(), kp2, matchesGood, img_match);
        cv::resize(img_match, img_match, {img_match.cols * 0.7f, img_match.rows * 0.7f});
        cv::imshow("motionEST", img_match);
    }

    std::cout << "matches: " << static_cast<float>(matches.size()) << ", good matches: " << pts3d1.size() << std::endl;

    Eigen::Affine3f motion = Eigen::Affine3f::Identity();
    unsigned inliersCount = 0;

    const int MIN_INLIERS = 20;

    if (pts3d1.size() > MIN_INLIERS)
    {
		cv::Mat rvec = cv::Mat_<double>(3, 1);
		cv::Mat tvec = cv::Mat_<double>(3, 1);

        std::vector<int> inliers;
        cv::solvePnPRansac(pts3d1, pts2d2, cameraMat, distCoeffs, rvec, tvec, false, 1000, 8.0f, 0.99f, inliers, cv::SOLVEPNP_EPNP);
    
        std::cout << "inliers: " << inliers.size() << " (" << (inliers.size() / static_cast<float>(pts2d2.size())) << ")" << std::endl;

        if (inliers.size() >= MIN_INLIERS)
		{
            inliersCount = inliers.size();

            cv::Mat R;
			cv::Rodrigues(rvec, R);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> R_Eigen(R.ptr<double>(), R.rows, R.cols);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> T_Eigen(tvec.ptr<double>(), tvec.rows, tvec.cols);

            motion.linear() = R_Eigen.cast<float>();
            motion.translation() = T_Eigen.cast<float>();
            motion = motion.inverse();

            {
                cv::Mat reprIm = frame2.GetImage();
                cv::Mat reprIm2;
                cv::cvtColor(reprIm, reprIm2, cv::COLOR_BGR2RGB);
                // pts3d1
				// std::vector<cv::Point2f> imagePointsReproj;
				// cv::projectPoints(pts3d1, rvec, tvec, this->cameraMat, this->distCoeffs, imagePointsReproj);
                
				float err = 0.0f;
                float minErr = 999999;
                float maxErr = -10000;

                // reprojection error check
				for (int i = 0; i < pts3d1.size(); ++i)
				{
                    const int id = i;//inliers[i];

                    auto pt = pts3d1.at(id);
                    auto pp1 = transformPointD(pt, R, tvec);
                    const auto pp = project3dPoint(pp1, this->cameraMatOneDim);

                    
                    // real points
                    cv::circle(reprIm2, pts2d2.at(id), 6, {0, 255, 0}, 3);
                    cv::circle(reprIm2, pp, 4, {0, 255, 255}, 3);
                    cv::line(reprIm2, pts2d2.at(id), pp, {0, 255, 255});

                    // reprojected
                    // cv::circle(reprIm2, imagePointsReproj.at(id), 2, {255, 0, 255}, 3);
                    // cv::line(reprIm2, pts2d2.at(id), imagePointsReproj.at(id), {255, 0, 0});

                    // err stat
					// const float err1 = std::pow(pts2d2.at(id).x - imagePointsReproj.at(id).x, 2) + 
                    //     std::pow(pts2d2.at(id).y - imagePointsReproj.at(id).y, 2);
					const float err1 = std::pow(pts2d2.at(id).x - pp.x, 2) + 
                        std::pow(pts2d2.at(id).y - pp.y, 2);

                    err += err1;
                    if (err1 < minErr)
                    {
                        minErr = err1;
                    }
                    if (maxErr < err1)
                    {
                        maxErr = err1;
                    }

                    // add observations
                    pts3d2[id].point3d.SetMapPointId(pts3d1_data[id].mapPointId);

				}
                
                frame2.SetPointsData(pts3d2);

                err = err / inliers.size();
                err = std::sqrt(err);

                cv::imshow("reprIm2", reprIm2);

				// *covariance *= std::sqrt(err/float(inliers.size()));
                std::cout << "===========================================" << std::endl;
                std::cout << "repr err: " << err << std::endl;
                std::cout << "repr minErr: " << std::sqrt(minErr) << std::endl;
                std::cout << "repr maxErr: " << std::sqrt(maxErr) << std::endl;
                std::cout << "===========================================" << std::endl;
            }
		}

    }

    return {motion.matrix(), inliersCount};
}

}