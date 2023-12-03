#pragma once

#include "sfm/data/Types.h"
#include "base/tools/Tools2d.h"

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/highgui.hpp>

namespace eacham
{

static void DrawMatches(const std::string name, const cv::Mat& image1, const cv::Mat& image2, 
    std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, 
    const int delay, const std::vector<cv::Scalar>& colors)
{
    cv::Mat outImg2 = ConcatImages(image1, image2);

    for (int i = 0; i < pts1.size(); ++i)
    {
        cv::Point p1 = pts1[i];
        cv::Point p2 = pts2[i];
        p2.x += image1.cols;

        cv::Scalar col = colors.size() > 0 ? colors[i] : cv::Scalar{255, 255, 0};

        cv::circle(outImg2, p1, 3, col);
        cv::circle(outImg2, p2, 3, col);
        cv::line(outImg2, p1, p2, col, 2, cv::LINE_AA );

    }
    
    while (outImg2.cols > 1200 || outImg2.rows > 700)
        ResizeImage(outImg2, 0.8);

    cv::imshow(name, outImg2);
    cv::waitKey(delay);
}

static void DrawMatches(const std::string name, const unsigned id1, const unsigned id2,
    std::shared_ptr<graph_t> graph, const int delay = 0)
{
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    std::vector<cv::Scalar> colors;

    const auto node1 = graph->Get(id1);
    const auto node2 = graph->Get(id2);
    const auto& factor = node1->GetFactor(node2->GetId());

    std::for_each(factor.matches.begin(), factor.matches.end(), [&](const auto& math){
        const auto p1 = node1->GetKeyPoint(math.id1);
        const auto p2 = node2->GetKeyPoint(math.id2);
        pts1.push_back({p1.x, p1.y});
        pts2.push_back({p2.x, p2.y});


        if (math.triangulatedPointId < 9999999)
            colors.push_back({0, 255, 0});
        else
            colors.push_back({0, 0, 255});
    });

    DrawMatches(name, node1->GetImage(), node2->GetImage(), pts1, pts2, delay, colors);
}

// static void DrawMatches(const std::string name, const Frame& frame1, const Frame& frame2,
//     std::vector<std::pair<unsigned, unsigned>> matches, const int delay = 0)
// {
//     std::vector<cv::Point2f> pts1;
//     std::vector<cv::Point2f> pts2;
//     std::vector<cv::Scalar> colors;

//     std::for_each(matches.begin(), matches.end(), [&](const auto& math){
//         pts1.push_back(frame1.keyPoints[math.first].pt);
//         pts2.push_back(frame2.keyPoints[math.second].pt);
//         colors.push_back({0, 0, 255});
//     });

//     DrawMatches(name, frame1.image, frame2.image, pts1, pts2, delay, colors);
// }


// static void DrawEpipolarLines(const std::string name, cv::Mat leftImageRGB, cv::Mat rightImageRGB, 
//     std::vector<cv::Point2f> imagePointsLeftCamera, std::vector<cv::Point2f> imagePointsRightCamera,
//     cv::Mat& fundamentalMatrix
//     )
// {
//     /*
// cv::Mat cv::findFundamentalMat( InputArray _points1, InputArray _points2,
//                                 int method, 
//                                 ransacReprojThreshold, -- if far than this then outlier 
//                                 confidence, Parameter used for the RANSAC and LMedS methods only. 
//                                     It specifies a desirable level of confidence (probability) that the estimated 
//                                     matrix is correct.
//                                 int maxIters, OutputArray _mask )
//     FM_7POINT   7-point algorithm
//     FM_8POINT   8-point algorithm
//     FM_LMEDS    least-median algorithm. 7-point algorithm is used.
//     FM_RANSAC   ANSAC algorithm. It needs at least 15 points. 7-point algorithm is used
//     */

//    cv::Mat leftIm = leftImageRGB.clone();
//    cv::Mat rightIm = rightImageRGB.clone();

//     fundamentalMatrix = cv::findFundamentalMat(imagePointsLeftCamera, imagePointsRightCamera, cv::FM_LMEDS,
//         2.0, 0.99);
    
//     std::vector<cv::Vec3f> leftLines, rightLines;
//     cv::computeCorrespondEpilines(imagePointsLeftCamera, 1, fundamentalMatrix, rightLines);
//     cv::computeCorrespondEpilines(imagePointsRightCamera, 2, fundamentalMatrix, leftLines);
	

//     if (leftLines.size() > 0)
//     {
//         float meanError = 0;
//         float minError = 99999999990;
//         float maxError = 0;

//         for(std::size_t i=0;i<leftLines.size();i=i+1)
//         {
//             cv::Vec3d l = leftLines.at(i);
//             double a=l.val[0];
//             double b=l.val[1];
//             double c=l.val[2];
        
//             // cv::Mat imagePointLeftCameraMatrix=cv::Mat_<double>(3,1);
//             // imagePointLeftCameraMatrix.at<double>(0,0)=imagePointsRightCamera[i].x;
//             // imagePointLeftCameraMatrix.at<double>(1,0)=imagePointsRightCamera[i].y;
//             // imagePointLeftCameraMatrix.at<double>(2,0)=1;
//             // cv::Mat rightLineMatrix = fundamentalMatrix * imagePointLeftCameraMatrix;
            
//             double x0,y0,x1,y1;
//             x0=0;
//             y0=(-c - a * x0) / b;
//             x1=leftIm.cols;
//             y1=(-c - a * x1) / b;

//             cv::line(leftIm, cv::Point2f(x0,y0), cv::Point2f(x1,y1), cv::Scalar(0, 255, 0), 1);
//             cv::circle(leftIm, imagePointsLeftCamera.at(i), 3, cv::Scalar(255, 0, 0), 3);

//             const double error = std::abs(a * imagePointsLeftCamera.at(i).x + b * imagePointsLeftCamera.at(i).y + c);
//             meanError += error;

//             if (error < minError) minError = error;
//             if (error > maxError) maxError = error;
//         }

//         std::cout << "LEFT epip:\nmin: " << minError << std::endl;
//         std::cout << "max: " << maxError << std::endl;
//         std::cout << "mean: " << (meanError / leftLines.size()) << std::endl;
//     }

//     if (rightLines.size() > 0)
//     {
//         float meanError = 0;
//         float minError = 99999999990;
//         float maxError = 0;

//         for(std::size_t i=0;i<rightLines.size();i=i+1)
//         {
//             cv::Vec3d l = rightLines.at(i);
//             double a=l.val[0];
//             double b=l.val[1];
//             double c=l.val[2];

//             // cv::Mat imagePointLeftCameraMatrix=cv::Mat_<double>(3,1);
//             // imagePointLeftCameraMatrix.at<double>(0,0)=imagePointsLeftCamera[i].x;
//             // imagePointLeftCameraMatrix.at<double>(1,0)=imagePointsLeftCamera[i].y;
//             // imagePointLeftCameraMatrix.at<double>(2,0)=1;
//             // cv::Mat rightLineMatrix = fundamentalMatrix * imagePointLeftCameraMatrix;
            
//             double x0,y0,x1,y1;
//             x0=0;
//             y0=(-c - a * x0) / b;
//             x1=rightIm.cols;
//             y1=(-c - a * x1) / b;

//             cv::line(rightIm, cv::Point2f(x0,y0), cv::Point2f(x1,y1), cv::Scalar(0, 255, 0), 1);
//             cv::circle(rightIm, imagePointsRightCamera.at(i), 3, cv::Scalar(255, 0, 0), 3);

//             const double error = std::abs(a * imagePointsRightCamera.at(i).x + b * imagePointsRightCamera.at(i).y + c);
//             meanError += error;

//             if (error < minError) minError = error;
//             if (error > maxError) maxError = error;
//         }

//         std::cout << "RIGHT epip:\nmin: " << minError << std::endl;
//         std::cout << "max: " << maxError << std::endl;
//         std::cout << "mean: " << (meanError / rightLines.size()) << std::endl;

//     }

//     cv::Mat outImg2 = ConcatImages(leftIm, rightIm);
//     ResizeImage(outImg2, 0.5);

//     cv::imshow(name, outImg2);
//     cv::waitKey(0);
// }

// }

// // const unsigned bestFor0 = frames[0].GetBestCorr();

// // if (bestFor0 < 1000)
// // {
// //     const auto corr = frames[0].correspondences[bestFor0];
// //     if (corr.isValid)
// //     {
// //         std::cout << "Best corr: " << corr.id1 << " -> " << corr.id2 << std::endl;

// //         graph.AddNode(corr);

// //         const unsigned bestFor1 = frames[corr.id2].GetBestCorr(0);

// //         if (bestFor1 < 1000)
// //         {
// //             const auto corr2 = frames[corr.id2].correspondences[bestFor1];

// //             if (corr2.isValid)
// //             {
// //                 graph.AddNode(corr2);

// //                 cv::Mat outImg;
// //                 cv::hconcat(frames[corr.id1].image, frames[corr.id2].image, outImg);
// //                 cv::hconcat(outImg, frames[corr2.id2].image, outImg);

// //                 const int size = 1280 * 720;
// //                 cv::resize(outImg, outImg, 
// //                     cv::Size((int)std::sqrt((double) outImg.cols * size / outImg.rows),
// //                                 (int)std::sqrt((double) outImg.rows * size / outImg.cols)));

// //                 cv::imshow("outImg", outImg);
// //                 cv::waitKey(0);

// //                 std::cout << "corr2: " << corr2.id1 << " -> " << corr2.id2 << std::endl;
// //             }
// //         }

// //     }
}