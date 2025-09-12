#include "misora2_pressure/pressure_component.hpp"

namespace component_pressure
{
PressureMeasurement::PressureMeasurement(const rclcpp::NodeOptions &options)
    : Node("measure_pressure", options),
     model(Detection::MODEL_PATH, Detection::ONNX_LOGID, Detection::ONNX_PROVIDER)
{
    receive_image_ = this->create_subscription<MyAdaptedType>("pressure_image",10,std::bind(&PressureMeasurement::update_image_callback,this,std::placeholders::_1));
    
    publisher_ = this->create_publisher<misora2_custom_msg::msg::Custom>("pressure_results",10);
    // 初期設定-----------------------------------------------------
    if (!std::filesystem::exists(Detection::MODEL_PATH)) {
        std::cerr << "Model file does not exist at path: " << Detection::MODEL_PATH << std::endl;
        throw std::runtime_error("Model file not found.");
    }
    
    colors = Detection::generateRandomColors(model.getNc(), model.getCh());
    names = model.getNames();
    RCLCPP_INFO_STREAM(this->get_logger(),"Complete Initialize");
    // -------------------------------------------------------------------
    // cv::Mat型のreceive_imageを入力としたメーター値検出関数 返り値std::pair<string,cv::Mat>func(cv::Mat )
    // auto [pressure_value, result_image] = func(receive_image)
}   

void PressureMeasurement::update_image_callback(const std::unique_ptr<cv::Mat> msg){
    cv::Mat receive_image = std::move(*msg);
    // double pressure_value;

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
            auto [trimming_image, result_image] = Detection::plot_results(receive_image, objs, colors, names);
            if(trimming_image.channels() == 1) std::cout << "Not found" << std::endl;
            else{ std::cout << "trimmed: " << trimming_image.size() << std::endl;
                Measurement::AngleInput args = Measurement::detect_line(trimming_image);
                // RCLCPP_INFO_STREAM(this->get_logger(),"args center: " << args.center << ", line: " << args.line );
                if( args.line[0] == 0 && args.line[1] == 0 && args.line[2] == 0 && args.line[3] == 0  ){
                    RCLCPP_INFO_STREAM(this->get_logger(), "Meter line not found");
                }
                else{
                    double angle = Measurement::calculate_angle_12cw(args.center, args.line);

                    // メーター種別分類
                    int meter_type_I = Measurement::classify_meter_type(trimming_image, model_path);
                    if (meter_type_I > 0){ // メーターの区別ができた
                        std::string meter_type = std::to_string(meter_type_I); // ここメーターの最大値の種類を決める　オペレータからStringトピックで受け取る or Yoloで自動判別どちらか
                        double pressure = Measurement::pressure_value_from_angle(angle, meter_type);
                    
                        flag = true;

                        misora2_custom_msg::msg::Custom data;
                        data.result = std::to_string(pressure);
                        cv::cvtColor(result_image, result_image, cv::COLOR_RGB2BGR);
                        data.raw_image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", receive_image).toImageMsg());
                        publisher_->publish(data);
                        RCLCPP_INFO_STREAM(this->get_logger(),"Publish data: "<< pressure << ", and image: " << result_image.size );
                    }
                    else RCLCPP_INFO_STREAM(this->get_logger(), "Meter type not found");
                }
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