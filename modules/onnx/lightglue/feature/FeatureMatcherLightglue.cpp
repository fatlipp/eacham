#include "FeatureMatcherLightglue.h"

#include <iostream>

#include <onnxruntime_cxx_api.h>

namespace eacham
{

FeatureMatcherLightglue::MatchType
    FeatureMatcherLightglue::Match(
        const matcher_t& descriptor1, 
        const matcher_t& descriptor2)
{
    std::cout << "FeatureMatcherLightglue::Match\n";

    if (descriptor1.first.size() != descriptor1.second.size()
        || descriptor1.second[0].size() != 256)
        {
            return {};
        }

    const std::string onnxpath = "/home/blackdyce/Projects/eacham/modules/onnx/lightglue/assets/superpoint_lightglue2.onnx";

    Ort::Env env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "FeatureExtractor");
    Ort::SessionOptions sessionOptions;
    OrtCUDAProviderOptionsV2 *options;
    Ort::GetApi().CreateCUDAProviderOptions(&options);
    sessionOptions.AppendExecutionProvider_CUDA_V2(*options);
    Ort::GetApi().ReleaseCUDAProviderOptions(options);

    Ort::Session* session = new Ort::Session(env, onnxpath.c_str(), sessionOptions);

    std::vector<const char*> inputNames;
    inputNames.push_back("kpts0");
    inputNames.push_back("kpts1");
    inputNames.push_back("desc0");
    inputNames.push_back("desc1");

    std::vector<const char*> outputNames;
    outputNames.push_back("matches0");
    outputNames.push_back("mscores0");
    
    const int size1 = descriptor1.first.size();
    const int size2 = descriptor2.first.size();

    std::array<int64_t, 3> inputShape1{1, size1, 2};
    std::array<int64_t, 3> inputShape2{1, size2, 2};
    std::array<int64_t, 3> inputShape3{1, size1, 256};
    std::array<int64_t, 3> inputShape4{1, size2, 256};
    
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU);

    auto build_tensor_kp = [&memory_info](const auto& keypointsInp, std::vector<float>& keypoints, const auto& shape)
        {
            for (const auto& d : keypointsInp)
            {
                keypoints.push_back(d.x);
                keypoints.push_back(d.y);
            }

            return Ort::Value::CreateTensor<float>(memory_info, 
                                            keypoints.data(), keypoints.size(),
                                            shape.data(), shape.size());
        };

    auto build_tensor_ds = [&memory_info](const auto& descriptorsInp, std::vector<float>& descriptorsFloat, auto& shape){
            for (const auto d : descriptorsInp)
            {
                for (int i = 0; i < 256; ++i)
                {
                    descriptorsFloat.push_back(d[i]);
                }
            }
            std::cout << "desc: " << descriptorsInp.size() << " -> " << descriptorsFloat.size() / 256 << std::endl;

            return Ort::Value::CreateTensor<float>(memory_info, 
                                            descriptorsFloat.data(), descriptorsFloat.size(),
                                            shape.data(), shape.size());
        };
    
    std::vector<float> keypoints1;
    std::vector<float> keypoints2;
    std::vector<float> descriptorsFloat1;
    std::vector<float> descriptorsFloat2;
    std::vector<Ort::Value> inputTensors;
    inputTensors.push_back(build_tensor_kp(descriptor1.first, keypoints1, inputShape1));
    inputTensors.push_back(build_tensor_kp(descriptor2.first, keypoints2, inputShape2));
    inputTensors.push_back(build_tensor_ds(descriptor1.second, descriptorsFloat1, inputShape3));
    inputTensors.push_back(build_tensor_ds(descriptor2.second, descriptorsFloat2, inputShape4));

    std::cout << "Run: start" << std::endl;

    auto outputTensors = session->Run(Ort::RunOptions{nullptr}, 
        inputNames.data(), inputTensors.data(), inputNames.size(), 
        outputNames.data(), outputNames.size());

    if (outputTensors.size() != outputNames.size())
    {
        return {};
    }

    auto matchesRaw = outputTensors[0].GetTensorMutableData<int64_t>();
    auto matchesCount = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount() / 2;

    auto scoresRaw = outputTensors[1].GetTensorMutableData<float>();

    std::cout << "matchesCount: " << matchesCount << std::endl;

    if (matchesCount == 0)
    {
        return {};
    }

    FeatureMatcherLightglue::MatchType matches;
    for (int i = 0; i < matchesCount; ++i)
    {
        if (scoresRaw[i] > 0.5)
        {
            const int m1 = matchesRaw[i * 2];
            const int m2 = matchesRaw[i * 2 + 1];
            matches.insert({m1, m2});
        }
    }

    std::cout << "total: " << matches.size() << std::endl;

    delete session;

    return matches;
}

}