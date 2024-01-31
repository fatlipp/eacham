#pragma once

#include "sfm/data/Types.h"
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <Eigen/Core>

#include <map>
#include <string>
#include <nlohmann/json.hpp>

namespace eacham
{

struct OptimizerConfig
{
    std::string method;
    int maxIter;
    float maxTolerance;
    float delta;
    bool usePreconditioner;
};

class SfmConfig
{
public:
    static SfmConfig Parse(const nlohmann::json& data)
    {
        SfmConfig result;
        std::string root = data["root_path"];
        std::string images = data["images_path"];
        std::string transform = data["transform_path"];
        result.imagesPath = root + images;
        result.outputTransformPath = root + transform;
        result.maxDataSize = data["max_data_count"];
        result.ui = data["ui"] == "true";

        auto feature = data["feature"];
        result.minFeaturesCount = feature["min_features_count"];
        result.maxFeaturesCount = feature["max_features_count"];
        result.inliersRatio = feature["inliers_ratio"];

        auto reconstruction = data["reconstruction"];
        auto initial = reconstruction["initial_pair"];
        result.initialMinInliers = initial["min_inliers"];
        result.initialMaxReprError = initial["max_reprojection_error"];
        result.initialMinTriAngle = initial["min_angle"];
        result.initialMinTriAngle *= 3.141592 / 180.0;

        auto processing = reconstruction["processing"];
        result.maxReprError = processing["max_reprojection_error"];
        result.minTriAngle = processing["min_angle"];
        result.minTriAngle *= 3.141592 / 180.0;
        result.minPnpInliers = processing["min_pnp_inliers"];

        auto refine = data["refine_ba"];
        result.refineOpt.maxIter = refine["max_iter"];
        result.refineOpt.maxTolerance = refine["max_toler"];
        result.refineOpt.method = refine["method"];
        result.refineOpt.delta = refine["delta"];
        result.refineOpt.usePreconditioner = refine["use_preconditioner"];

        auto global = data["global_ba"];
        result.globalOpt.maxIter = global["max_iter"];
        result.globalOpt.maxTolerance = global["max_toler"];
        result.globalOpt.method = global["method"];
        result.globalOpt.delta = refine["delta"];
        result.globalOpt.usePreconditioner = refine["use_preconditioner"];

        return result;
    }

public:
    std::string imagesPath;
    std::string outputTransformPath;
    
    int minFeaturesCount;
    int maxFeaturesCount;
    float inliersRatio;
    int maxDataSize;
    
    unsigned initialMinInliers;
    float initialMaxReprError;
    float initialMinTriAngle;
    float maxReprError;
    float minTriAngle;
    int minPnpInliers;

    OptimizerConfig refineOpt;
    OptimizerConfig globalOpt;

    bool ui;
};

}