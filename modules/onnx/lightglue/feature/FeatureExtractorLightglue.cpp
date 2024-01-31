#include "FeatureExtractorLightglue.h"

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

namespace eacham
{

FeatureExtractorLightglue::ReturnType FeatureExtractorLightglue::Extract(const cv::Mat &image)
{
    std::cout << "FeatureExtractorLightglue::Extract\n";
    const std::string onnxpath = "/home/blackdyce/Projects/eacham/modules/onnx/lightglue/assets/superpoint2.onnx";

    Ort::Env env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "Feat_extractor");
    Ort::SessionOptions sessionOptions;
    OrtCUDAProviderOptionsV2 *options;
    Ort::GetApi().CreateCUDAProviderOptions(&options);
    sessionOptions.AppendExecutionProvider_CUDA_V2(*options);
    Ort::GetApi().ReleaseCUDAProviderOptions(options);

    std::cout << "FeatureExtractorLightglue::session\n";
    Ort::Session* session = new Ort::Session(env, onnxpath.c_str(), sessionOptions);

    std::cout << "FeatureExtractorLightglue::inputNames\n";
    std::vector<const char*> inputNames;
    inputNames.push_back("image");

    std::vector<const char*> outputNames;
    outputNames.push_back("keypoints");
    outputNames.push_back("scores");
    outputNames.push_back("descriptors");
    
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU);

    auto build_tensor_image = [&memory_info](const cv::Mat& image, std::vector<float>& vec, const auto& shape){
            cv::Mat gray;

            if (gray.channels() == 3)
            {
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                gray.convertTo(gray, CV_8UC1);
            }
            else
            {
                gray = image;
            }
            
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

    std::array<int64_t, 4> inputShape{1, 1, image.rows, image.cols};
    std::vector<Ort::Value> inputTensors;
    std::vector<float> imageVecFloat;
    inputTensors.push_back(build_tensor_image(image, imageVecFloat, inputShape));

    std::cout << "Run: start" << std::endl;

    auto outputTensors = session->Run(Ort::RunOptions{nullptr}, 
        inputNames.data(), inputTensors.data(), inputNames.size(), 
        outputNames.data(), outputNames.size());

    if (outputTensors.size() != outputNames.size())
    {
        return {};
    }

    std::cout << "size: " << outputTensors.size() << std::endl;
    
    auto kpsRaw = outputTensors[0].GetTensorMutableData<int64_t>();
    auto kpsCount = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount() / 2;

    auto scoresRaw = outputTensors[1].GetTensorMutableData<float>();
    auto scoresCount = outputTensors[1].GetTensorTypeAndShapeInfo().GetElementCount();

    auto descriptorsRaw = outputTensors[2].GetTensorMutableData<float>();
    auto descriptorsCount = outputTensors[2].GetTensorTypeAndShapeInfo().GetElementCount();

    std::cout << "kpsCount: " << kpsCount << std::endl;
    std::cout << "descriptorsCount: " << (descriptorsCount / 256.0) << std::endl;

    if (scoresCount == 0)
    {
        return {};
    }

    keypoints_t features;
    descriptor_t descriptors;

    for (int i = 0; i < scoresCount; ++i) 
    {
        if (static_cast<float>(scoresRaw[i]) < 0.05f)
        {
            continue;
        }

        features.push_back({ 
                static_cast<float>(kpsRaw[i * 2]), 
                static_cast<float>(kpsRaw[i * 2 + 1])
            });

        std::array<float, 256> desc;

        for (int j = 0; j < 256; ++j)
        {
            desc[j] = descriptorsRaw[i * 256 + j];
        }

        descriptors.push_back(desc);
    }

    std::cout << "total: " << features.size() << std::endl;

    delete session;

    return {features, descriptors};
}

}