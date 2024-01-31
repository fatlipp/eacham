#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>

namespace eacham::tools
{
/// @brief 
/// @z = f * b / d
/// @x = (xl - cx) * z / f
/// @y = (yl - cy) * z / f
/// @param baseline feed (bl * fx)
/// @param camera 1x5 matrix
static cv::Point3f Get3dPointByStereoPair(const cv::Point2f& left, const cv::Point2f& right, 
	const cv::Mat &camera)
{
	const float fx = camera.at<float>(0, 0);

	if ((left.x - right.x) == 0 || fx == 0.0f)
	{
		return {-1, -1, -1};
	}

	const float z = (camera.at<float>(0, 4)) / (left.x - right.x);
	return cv::Point3f(((left.x - camera.at<float>(0, 2)) * z) / fx, ((left.y - camera.at<float>(0, 3)) * z) / fx, z);
}

static cv::Point3f Get3dPointByDepthMap(const cv::Point2f& point, const cv::Mat &depthMap, const cv::Mat &camera)
{
	const float fx = camera.at<float>(0, 0);
	const float fy = camera.at<float>(0, 1);
	const float cx = camera.at<float>(0, 2);
	const float cy = camera.at<float>(0, 3);
	const float scale = camera.at<float>(0, 4);

	const int u = point.x;
	const int v = point.y;

	const float z = depthMap.at<ushort>(v, u) * scale;
	const float x = (u - cx) * z / fx;
	const float y = (v - cy) * z / fy;

	return cv::Point3f(x, y, z);
}

// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
static int BinaryDescriptorDist(const cv::Mat &a, const cv::Mat &b)
{
	const int *pa = a.ptr<int32_t>();
	const int *pb = b.ptr<int32_t>();

	int dist = 0;

	for(int i=0; i<8; i++, pa++, pb++)
	{
		unsigned  int v = *pa ^ *pb;
		v = v - ((v >> 1) & 0x55555555);
		v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
		dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
	}

	return dist;
}

/// @brief 
/// @param rot <DOUBLE>
/// @param translation <DOUBLE> ROW MAJOR
static cv::Point3f transformPointD(
	const cv::Point3f & point,
	const cv::Mat & rot,
	const cv::Mat &translation)
{
	cv::Point3f ret;
	ret.x = rot.at<double>(0, 0) * point.x + rot.at<double>(0, 1) * point.y + rot.at<double>(0, 2) * point.z + translation.at<double>(0, 0);
	ret.y = rot.at<double>(1, 0) * point.x + rot.at<double>(1, 1) * point.y + rot.at<double>(1, 2) * point.z + translation.at<double>(1, 0);
	ret.z = rot.at<double>(2, 0) * point.x + rot.at<double>(2, 1) * point.y + rot.at<double>(2, 2) * point.z + translation.at<double>(2, 0);

	return ret;
}

static cv::Point3f transformPoint3x3(
	const cv::Point3f & point,
	const cv::Mat & transform)
{
	cv::Point3f ret = point;
	ret.x = transform.at<float>(0, 0) * point.x + transform.at<float>(0, 1) * point.y + transform.at<float>(0, 2) * point.z;
	ret.y = transform.at<float>(1, 0) * point.x + transform.at<float>(1, 1) * point.y + transform.at<float>(1, 2) * point.z;
	ret.z = transform.at<float>(2, 0) * point.x + transform.at<float>(2, 1) * point.y + transform.at<float>(2, 2) * point.z;
	return ret;
}

// static cv::Point3f transformPoint3d(
// 	const cv::Point3f & point,
// 	const Eigen::Matrix4f & transform)
// {
// 	cv::Point3f ret = point;
// 	ret.x = transform(0, 0) * point.x + transform(0, 1) * point.y + transform(0, 2) * point.z + transform(0, 3);
// 	ret.y = transform(1, 0) * point.x + transform(1, 1) * point.y + transform(1, 2) * point.z + transform(1, 3);
// 	ret.z = transform(2, 0) * point.x + transform(2, 1) * point.y + transform(2, 2) * point.z + transform(2, 3);
// 	return ret;
// }

static Eigen::Vector3d transformPoint3d(
	const Eigen::Vector3d & point,
	const Eigen::Matrix4d & transform)
{
	Eigen::Vector3d ret = point;
	ret.x() = transform(0, 0) * point.x() + transform(0, 1) * point.y() + transform(0, 2) * point.z() + transform(0, 3);
	ret.y() = transform(1, 0) * point.x() + transform(1, 1) * point.y() + transform(1, 2) * point.z() + transform(1, 3);
	ret.z() = transform(2, 0) * point.x() + transform(2, 1) * point.y() + transform(2, 2) * point.z() + transform(2, 3);
	return ret;
}

template<typename T>
static Eigen::Vector2d Project3dPoint(const T& point3d, const cv::Mat &camera)
{
	return {((camera.at<double>(0, 0) * point3d.x()) / point3d.z()) + camera.at<double>(0, 2),
			 ((camera.at<double>(1, 1) * point3d.y()) / point3d.z()) + camera.at<double>(1, 2)};
}

static float GetDistance(const cv::Point3f &point1, const cv::Point3f &point2)
{
	const float d1 = (point1.x - point2.x);
	const float d2 = (point1.y - point2.y);
	const float d3 = (point1.z - point2.z);

	return std::sqrt(d1 * d1 + d2 * d2 + d3 * d3);
}
	
}