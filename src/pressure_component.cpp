#include "misora2_pressure/pressure_component.hpp"

namespace component_pressure
{
PressureMeasurement::PressureMeasurement(const rclcpp::NodeOptions &options)
    : Node("measure_pressure", options)
{
    receive_image_ = this->create_subscription<MyAdaptedType>("pressure_image",10,std::bind(&PressureMeasurement::update_image_callback,this,std::placeholders::_1));
    
    pressure_value_publisher_ = this->create_publisher<std_msgs::msg::String>("pressure_result_data",10);
    result_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("pressure_result_image",10);//不要だったらコメントアウト

    // cv::Mat型のreceive_imageを入力としたメーター値検出関数 返り値std::pair<string,cv::Mat>func(cv::Mat )
    // auto [pressure_value, result_image] = func(receive_image)
}   

void PressureMeasurement::update_image_callback(const std::unique_ptr<cv::Mat> msg){
    receive_image = std::move(*msg);

    RCLCPP_INFO_STREAM(this->get_logger(),"Receive image address: " << &(msg->data));
    
}

} //namespace component_pressure
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_pressure::PressureMeasurement)