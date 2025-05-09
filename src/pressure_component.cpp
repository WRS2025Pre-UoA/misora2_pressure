#include "misora2_pressure/pressure_component.hpp"

namespace component_pressure
{
PressureMeasurement::PressureMeasurement(const rclcpp::NodeOptions &options)
    : Node("measure_pressure", options)
{
    receive_image_ = this->create_subscription<MyAdaptedType>("pressure_image",10,std::bind(&PressureMeasurement::update_image_callback,this,std::placeholders::_1));
    
    // 結果の報告はFloat64型のメッセージで返すかもしれない
    pressure_value_publisher_f = this->create_publisher<std_msgs::msg::Float64>("pressure_result_data",10);
    pressure_value_publisher_ = this->create_publisher<std_msgs::msg::String>("pressure_result_data",10);
    
    result_image_publisher_ = this->create_publisher<MyAdaptedType>("pressure_result_image",10);//不要だったらコメントアウト

    // cv::Mat型のreceive_imageを入力としたメーター値検出関数 返り値std::pair<string,cv::Mat>func(cv::Mat )
    // auto [pressure_value, result_image] = func(receive_image)
}   

void PressureMeasurement::update_image_callback(const std::unique_ptr<cv::Mat> msg){
    receive_image = std::move(*msg);
    
    // RCLCPP_INFO_STREAM(this->get_logger(),"Receive image address: " << &(msg->data));
    pressure_value = "0.67";
    std_msgs::msg::String msg_S;
    msg_S.data = pressure_value;
    
    // 黒画像が来るまで値を格納し続け、近似値を渡すか？
    // 最後の値を渡すか？
    // 安全性チェックを追加
    if (receive_image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received image is empty. Skipping.");
        return;
    }
    if (receive_image.type() != CV_8UC1) {
        RCLCPP_WARN(this->get_logger(), "Image type is not CV_8UC1 (grayscale). countNonZero may fail. Type: %d", receive_image.type());
        return;
    }
    if (cv::countNonZero(receive_image) == 0) {
        pressure_value_publisher_->publish(msg_S);
        RCLCPP_INFO_STREAM(this->get_logger(),"Receive brack");
        result_image_publisher_->publish(result_image);
    }else{
        result_image = receive_image.clone();
    }
    
}

} //namespace component_pressure
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_pressure::PressureMeasurement)