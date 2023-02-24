#pragma once

#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace tools
{
	/// @brief 
	// z = f * b / d
	// x = xl * z / f
	// y = yl * z / f
	/// @param baseline caution: send raw baseline in m.
	/// @return z: (baseline * f) / d;
	static cv::Point3f Get3dPointByStereoPair(const cv::Point2f& left, const cv::Point2f& right, 
		const float fx, const float cx, const float cy, const float baseline)
	{
		const float zDivFx = baseline / (left.x - right.x);
		return cv::Point3f((left.x - cx) * zDivFx, (right.y - cy) * zDivFx, fx * zDivFx);
  	}

	/// @brief 
	// z = f * b / d
	// x = (xl - cx) * z / f
	// y = (yl - cy) * z / f
	/// @param baseline caution: send raw baseline in m.
	/// @return z: (baseline * f) / d;
	static cv::Point3f Get3dPointByStereoPair(const cv::Point2f& left, const cv::Point2f& right, 
		const cv::Mat &camera)
	{
		const float fx = camera.at<float>(0, 0);

		if ((left.x - right.x) == 0 || fx == 0.0f)
		{
			return {-1, -1, -1};
		}

		const float z = (camera.at<float>(0, 4)) / (left.x - right.x);
		return cv::Point3f(((left.x - camera.at<float>(0, 2)) * z) / fx, ((right.y - camera.at<float>(0, 3)) * z) / fx, z);
  	}

	// cv::Point3f transformPoint(
	// 		const cv::Point3f & point,
	// 		const Transform & transform)
	// {
	// 	cv::Point3f ret = point;
	// 	ret.x = transform (0, 0) * point.x + transform (0, 1) * point.y + transform (0, 2) * point.z + transform (0, 3);
	// 	ret.y = transform (1, 0) * point.x + transform (1, 1) * point.y + transform (1, 2) * point.z + transform (1, 3);
	// 	ret.z = transform (2, 0) * point.x + transform (2, 1) * point.y + transform (2, 2) * point.z + transform (2, 3);
	// 	return ret;	
	// }	
}