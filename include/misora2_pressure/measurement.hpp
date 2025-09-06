#ifndef MISORA2_PRESSURE_MEASUREMENT_HPP
#define MISORA2_PRESSURE_MEASUREMENT_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <limits>

class Measurement{
public:
    struct AngleInput {
        cv::Point2d center;  // 円の中心
        cv::Vec3f circle;   // 採用円 (x,y,r)
        cv::Vec4i   line;    // 採用線 (x1,y1,x2,y2)
        bool        valid;   // 成功/失敗フラグ
    };
    
    static double calculate_angle_12cw(const cv::Point2d& center, const cv::Vec4i& line);
    static cv::Mat make_skeleton(const cv::Mat& morph);
    static AngleInput detect_line(const cv::Mat& image);
    static double pressure_value_from_angle(double angle_deg, std::string meter_type);
};

#endif // MISORA2_PRESSURE_MEASUREMENT_HPP