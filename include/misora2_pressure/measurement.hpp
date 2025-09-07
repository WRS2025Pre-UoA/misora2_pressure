#ifndef MISORA2_PRESSURE_MEASUREMENT_HPP
#define MISORA2_PRESSURE_MEASUREMENT_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <onnxruntime_cxx_api.h>  // ONNX Runtime C++ API
#include <limits>

class Measurement{
public:
    struct AngleInput {
        cv::Point2d center;  // 円の中心
        cv::Vec3f circle;   // 採用円 (x,y,r)
        cv::Vec4i   line;    // 採用線 (x1,y1,x2,y2)
        bool        valid;   // 成功/失敗フラグ
    };
    
    // 学習時のフォルダ辞書順に合わせる
    // static const char* kClassNames[] = {"0_25MPa", "1_0MPa", "1_6MPa"};

    static double calculate_angle_12cw(const cv::Point2d& center, const cv::Vec4i& line);
    static cv::Mat make_skeleton(const cv::Mat& morph);
    static AngleInput detect_line(const cv::Mat& image);
    static double pressure_value_from_angle(double angle_deg, std::string meter_type);
    static int classify_meter_type(const cv::Mat& image, const std::string& model_path, int imgsize=224);
};

#endif // MISORA2_PRESSURE_MEASUREMENT_HPP