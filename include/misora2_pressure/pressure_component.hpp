#ifndef PRESSURE_COMPONENT_HPP
#define PRESSURE_COMPONENT_HPP

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

#include "misora2_pressure/cv_mat_type_adapter.hpp"

using namespace std::chrono_literals;

namespace component_pressure
{
class PressureMeasurement : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;

    bool flag = false;

    explicit PressureMeasurement(const rclcpp::NodeOptions &options);
    PressureMeasurement() : PressureMeasurement(rclcpp::NodeOptions{}) {}

private:
    void update_image_callback(const std::unique_ptr<cv::Mat> msg);
    std::string to_string_with_precision(double value, int precision);

    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_image_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pressure_value_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_value_publisher_f;
    rclcpp::Publisher<MyAdaptedType>::SharedPtr result_image_publisher_;
};

} // namespace component_pressure

#endif // PRESSURE_COMPONENT_HPP