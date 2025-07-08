#include "misora2_pressure/pressure_component.hpp"

namespace component_pressure
{
PressureMeasurement::PressureMeasurement(const rclcpp::NodeOptions &options)
    : Node("measure_pressure", options),
     model(Detection::MODEL_PATH, Detection::ONNX_LOGID, Detection::ONNX_PROVIDER)
{
    receive_image_ = this->create_subscription<MyAdaptedType>("pressure_image",10,std::bind(&PressureMeasurement::update_image_callback,this,std::placeholders::_1));
    
    pressure_value_publisher_ = this->create_publisher<std_msgs::msg::String>("pressure_result_data",10);
    result_image_publisher_ = this->create_publisher<MyAdaptedType>("pressure_result_image",10);//不要だったらコメントアウト

    // 初期設定-----------------------------------------------------
    if (!std::filesystem::exists(Detection::MODEL_PATH)) {
        std::cerr << "Model file does not exist at path: " << Detection::MODEL_PATH << std::endl;
        throw std::runtime_error("Model file not found.");
    }
    
    colors = Detection::generateRandomColors(model.getNc(), model.getCh());
    names = model.getNames();
    // -------------------------------------------------------------------
    // cv::Mat型のreceive_imageを入力としたメーター値検出関数 返り値std::pair<string,cv::Mat>func(cv::Mat )
    // auto [pressure_value, result_image] = func(receive_image)
}   

void PressureMeasurement::update_image_callback(const std::unique_ptr<cv::Mat> msg){
    cv::Mat receive_image = std::move(*msg);
    double pressure_value;
    bool flag_sf; // success / failure
    cv::Mat result_image, trimming_image;

    if (not(receive_image.empty())){
        if (flag == false and receive_image.channels() != 1){// カラー画像である
            // 実装分部
             // 推論実行
            std::vector<YoloResults> objs = model.predict_once(
                receive_image,
                Detection::CONF_THRESHOLD,
                Detection::IOU_THRESHOLD,
                Detection::MASK_THRESHOLD,
                Detection::CONVERSION_CODE
            );
            auto [trimming_image, result_image] = Detection::plot_results(receive_image, objs, colors, names, receive_image.size());
            if(trimming_image.channels() == 1) std::cout << "Not found" << std::endl;
            else{ std::cout << "trimmed: " << trimming_image.size() << std::endl;
            // auto[result_image, trimming_image] = func1(receive_image); // 成功：検出したメーターを囲んだ画像、切り抜いた画像　失敗：黒画像ｘ２
            // if( result_image.channels() != 1 and trimming_image.channels() != 1 ){ // 検出成功時
            //     auto[pressure_value, flag_sf] = func2(trimming_image);
            //     if(flag_sf){
            //         std_msgs::msg::String msg_S;
            //         msg_S.data = to_string_with_precision(pressure_value, 7); // 桁数は後々調整
            //         pressure_value_publisher_->publish(msg_S);
            //         // 取得したメーター値を検出画像に書き込むこと
            //         result_image_publisher_->publish(result_image);
            //         flag = true;
            //     }else RCLCPP_INFO_STREAM(this->get_logger(), "False get pressure value");
            // }else RCLCPP_INFO_STREAM(this->get_logger(), "Couldn't find meter");
                // テスト用-------------------------------------------
                std_msgs::msg::String msg_S;
                msg_S.data = "1.56";
                pressure_value_publisher_->publish(msg_S);
                result_image_publisher_->publish(result_image); // ボックス付き画像
                // result_image_publisher_->publish(trimming_image);
                RCLCPP_INFO_STREAM(this->get_logger(),"Publish: "<< receive_image.size() );
                flag = true;
                // ---------------------------------------------------
            }
        }
        else if(receive_image.channels() == 1){
            RCLCPP_INFO_STREAM(this->get_logger(),"Receive: black image" );
            flag = false;// 1 chanelある画像　黒画像  
        }
    }
}

std::string PressureMeasurement::to_string_with_precision(double value, int precision = 6) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

} //namespace component_pressure
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_pressure::PressureMeasurement)