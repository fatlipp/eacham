
#include "onnx/lightglue/core/FeatureExtractorLightglue.h"
#include "onnx/lightglue/core/FeatureMatcherLightglue.h"

#include <onnxruntime_cxx_api.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <iostream>
#include <string>


void ExtractAndMatch(
    const std::string& onnxpath, 
    const cv::Mat& image1, const cv::Mat& image2, 
    std::vector<float>& scores, 
    std::vector<cv::Point2f>& keypoints1, std::vector<cv::Point2f>& keypoints2, 
    std::vector<std::pair<unsigned, unsigned>>& matches)
{
    std::cout << "ExtractAndMatch\n";

    Ort::Env env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "Feat_extractor");
    Ort::SessionOptions sessionOptions;
    OrtCUDAProviderOptionsV2 *options;
    Ort::GetApi().CreateCUDAProviderOptions(&options);
    sessionOptions.AppendExecutionProvider_CUDA_V2(*options);
    Ort::GetApi().ReleaseCUDAProviderOptions(options);

    Ort::Session* session = new Ort::Session(env, onnxpath.c_str(), sessionOptions);

    std::vector<const char*> input_names;
    input_names.push_back("image0");
    input_names.push_back("image1");

    std::vector<const char*> output_names;
    output_names.push_back("kpts0");
    output_names.push_back("kpts1");
    output_names.push_back("matches0");
    output_names.push_back("mscores0");

    std::array<int64_t, 4> input_shape_1{1, 1, image1.rows, image1.cols};
    std::array<int64_t, 4> input_shape_2{1, 1, image2.rows, image2.cols};
    
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU);

    auto build_tensor_image = [&memory_info](const cv::Mat& image, std::vector<float>& vec, const auto& shape){
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            gray.convertTo(gray, CV_8UC1);
            
            std::vector<uint8_t> imageVec(gray.begin<uint8_t>(), gray.end<uint8_t>());
            vec.resize(imageVec.size());

            for (int i = 0; i < imageVec.size(); ++i)
            {
                vec[i] = static_cast<float>(imageVec[i]) / 255.0f;
            }
            
            return Ort::Value::CreateTensor<float>(memory_info, 
                                            vec.data(), vec.size(),
                                            shape.data(), shape.size());
        };

    std::vector<Ort::Value> input_tensors_arr;
    std::vector<float> imageVecFloat1;
    std::vector<float> imageVecFloat2;
    input_tensors_arr.push_back(build_tensor_image(image1, imageVecFloat1, input_shape_1));
    input_tensors_arr.push_back(build_tensor_image(image2, imageVecFloat2, input_shape_2));

    std::cout << "Run: start" << std::endl;

    auto outputTensors = session->Run(Ort::RunOptions{nullptr}, 
        input_names.data(), input_tensors_arr.data(), input_names.size(), 
        output_names.data(), output_names.size());

    std::cout << "result size: " << outputTensors.size() << std::endl;

    auto kps1_raw = outputTensors[0].GetTensorMutableData<int64_t>();
    auto kps1_count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount() / 2;
    auto kps1_type = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementType();

    auto kps2_raw = outputTensors[1].GetTensorMutableData<int64_t>();
    auto kps2_count = outputTensors[1].GetTensorTypeAndShapeInfo().GetElementCount() / 2;
    auto kps2_type = outputTensors[1].GetTensorTypeAndShapeInfo().GetElementType();

    auto matches_raw = outputTensors[2].GetTensorMutableData<int64_t>();
    auto matches_count = outputTensors[2].GetTensorTypeAndShapeInfo().GetElementCount() / 2;
    auto matches_type = outputTensors[2].GetTensorTypeAndShapeInfo().GetElementType();

    auto scores_raw = outputTensors[3].GetTensorMutableData<float>();
    auto scores_count = outputTensors[3].GetTensorTypeAndShapeInfo().GetElementCount();
    auto scores_type = outputTensors[3].GetTensorTypeAndShapeInfo().GetElementType();

    std::cout << "kps1: " << kps1_count << ", kps2: " << kps2_count 
        << ", matches: " << matches_count << std::endl;

    keypoints1.clear();
    keypoints2.clear();
    matches.clear();
    scores.clear();

    for (int i = 0; i < kps1_count; ++i) 
    {
        keypoints1.push_back({ static_cast<float>(kps1_raw[i * 2]), static_cast<float>(kps1_raw[i * 2 + 1]) });
    }
    for (int i = 0; i < kps2_count; ++i) 
    {
        keypoints2.push_back({ static_cast<float>(kps2_raw[i * 2]), static_cast<float>(kps2_raw[i * 2 + 1]) });
    }
    for (int i = 0; i < matches_count; ++i) 
    {
        matches.push_back({ static_cast<unsigned>(matches_raw[i * 2]), static_cast<unsigned>(matches_raw[i * 2 + 1]) });
    }
    for (int i = 0; i < scores_count; ++i) 
    {
        scores.push_back(scores_raw[i]);
    }

    std::cout << "END.."<< std::endl;
}


int main(int argc, const char* argv[]) 
{
    std::string imagepath1 = argv[1];
    std::string imagepath2 = argv[2];
    std::string onnxpath = argv[3];

    cv::Mat image1 = cv::imread(imagepath1);
    cv::Mat image2 = cv::imread(imagepath2);

    cv::resize(image1, image1, cv::Size(), 512.0 / image1.cols, 512.0 / image1.cols);
    cv::resize(image2, image2, cv::Size(), 512.0 / image2.cols, 512.0 / image2.cols);
    // std::cout << "" << image1.size() << std::endl;
    // cv::imshow("image1", image1);
    // cv::imshow("image2", image2);
    // cv::waitKey(0);
    // cv::resize(image1, image1, cv::Size(), 512.0 / image1.cols, 512.0 / image1.cols);
    // cv::resize(image2, image2, cv::Size(), 512.0 / image2.cols, 512.0 / image2.cols);
    // // cv::resize(image2, image2, cv::Size(), 256.0 / image2.cols, 256.0 / image2.rows);

    std::vector<float> scores;
    std::vector<cv::Point2f> keypoints1;
    std::vector<cv::Point2f> keypoints2;
    std::vector<std::pair<unsigned, unsigned>> matches;

    ExtractAndMatch(onnxpath, image1, image2, scores, keypoints1, keypoints2, matches);

    for (int i = 0; i < keypoints1.size(); ++i)
    {
        cv::circle(image1, keypoints1[i], 2, cv::Scalar{0, 0, 250}, 2);
    }
    for (int i = 0; i < keypoints2.size(); ++i)
    {
        cv::circle(image2, keypoints2[i], 2, cv::Scalar{0, 0, 250}, 2);
    }

    cv::Mat output;
    cv::hconcat(image1, image2, output);

    for (int i = 0; i < matches.size(); ++i)
    {
        cv::Point2f p1 = keypoints1[std::get<0>(matches[i])];
        cv::circle(output, p1, 1, cv::Scalar{0, 0, 255}, 2);
        cv::circle(output, p1, scores[i] * 15, cv::Scalar{0, 255, 0}, 2);

        cv::Point2f p2 = keypoints2[std::get<1>(matches[i])] + cv::Point2f{image1.cols, 0};
        cv::circle(output, p2, 1, cv::Scalar{0, 0, 255}, 2);
        cv::circle(output, p2, scores[i] * 15, cv::Scalar{0, 255, 0}, 2);

        cv::line(output, p1, p2, cv::Scalar{0, 255, 0});
    }

    cv::imshow("output", output);
    cv::waitKey(0);

    return 0;
}
