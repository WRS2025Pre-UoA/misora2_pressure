#include "misora2_pressure/detection.hpp"

int main(int argc, char *argv[]) {
    std::string image_file;

    // if(argc > 1)image_file = argv[1];
    // else {
    //     std::cout << "Please input image path" << std::endl;
    //     return 0;
    // }
    // // std::cout << image_file << std::endl;
    // 初期設定-----------------------------------------------------
    if (!std::filesystem::exists(Detection::MODEL_PATH)) {
        std::cerr << "Model file does not exist at path: " << Detection::MODEL_PATH << std::endl;
        throw std::runtime_error("Model file not found.");
    }
    // モデルの準備
    AutoBackendOnnx model(Detection::MODEL_PATH, Detection::ONNX_LOGID, Detection::ONNX_PROVIDER);
    // std::cout << image_file << 1 << std::endl;
    std::vector<cv::Scalar> colors = Detection::generateRandomColors(model.getNc(), model.getCh());
    std::unordered_map<int, std::string> names = model.getNames();

    // // 画像の読み込み
    // cv::Mat img = cv::imread(image_file, cv::IMREAD_UNCHANGED);
    // // 推論実行
    // std::vector<YoloResults> objs = model.predict_once(
    //     img,
    //     Detection::CONF_THRESHOLD,
    //     Detection::IOU_THRESHOLD,
    //     Detection::MASK_THRESHOLD,
    //     Detection::CONVERSION_CODE
    // );
    // // -------------------------------------------------------------------------------------
    
    // if (img.empty()) {
    //     std::cerr << "画像の読み込みに失敗しました: " << image_file << std::endl;
    //     return -1;
    // }
    // if(objs.empty()){
    //     std::cout << "Not Found" << std::endl;
    // }
    // std::cout << size(objs) <<std::endl;
    // std::cout << "Processing image: " << image_file << " (size: " << img.size() << ")" << std::endl;
    // // 結果の描画
    // cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    // auto [trimmed, boxed] = Detection::plot_results(img, objs, colors, names);
    // if(trimmed.channels() == 1){
    //     std::cout << "Not found" << std::endl;
    // }
    // else {
    //     std::cout << "trimmed: " << trimmed.size() << std::endl;
    //     cv::imshow("trimmed",trimmed);
    //     cv::imshow("with box",boxed);
    //     cv::waitKey(0);
    //     cv::destroyAllWindows();
    // }

    // ディレクトリごとに処理
    // std::string folder_path = "src/misora2_pressure/data/distance-15/";// 0~44
    // std::string folder_path = "src/misora2_pressure/data/distance-30/";// 0~63
    // std::string folder_path = "src/misora2_pressure/data/distance-45/";// 0~7
    // std::string folder_path = "../20250905_Meter/original/";
    std::string folder_path = "../OriginalData_640/original/";
    int max_num = 482;

    for( int i = 1 ; i < max_num ; i++){
        image_file = folder_path + "meter_" + std::to_string(i) + ".jpg";
        std::cout << image_file << std::endl;
    
        // 画像の読み込み
        cv::Mat img = cv::imread(image_file, cv::IMREAD_UNCHANGED);
        // 推論実行
        std::vector<YoloResults> objs = model.predict_once(
            img,
            Detection::CONF_THRESHOLD,
            Detection::IOU_THRESHOLD,
            Detection::MASK_THRESHOLD,
            Detection::CONVERSION_CODE
        );
        // -------------------------------------------------------------------------------------
        
        if (img.empty()) {
            std::cerr << "画像の読み込みに失敗しました: " << image_file << std::endl;
            return -1;
        }
        if(objs.empty()){
            std::cout << "Not Found" << std::endl;
        }
        std::cout << size(objs) <<std::endl;
        std::cout << "Processing image: " << image_file << " (size: " << img.size() << ")" << std::endl;
        // 結果の描画
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        auto [trimmed, boxed] = Detection::plot_results(img, objs, colors, names);
        if(trimmed.channels() == 1){
            std::cout << "Not found" << std::endl;
        }
        else {
            std::cout << "trimmed: " << trimmed.size() << std::endl;
            // cv::imshow("trimmed",trimmed);
            // cv::imshow("with box",boxed);
            // cv::waitKey(0);
            // cv::destroyAllWindows();
            // std::string save_file_cropped = "src/misora2_pressure/20250807_cropped/distance-15/pic_" + std::to_string(i) + ".png";
            // std::string save_file_cropped = "src/misora2_pressure/20250807_cropped/distance-30/pic_" + std::to_string(i) + ".png";
            // std::string save_file_cropped = "src/misora2_pressure/20250807_cropped/distance-45/pic_" + std::to_string(i) + ".png";
            // std::string save_file_detected = "src/misora2_pressure/20250807_detected/distance-15/pic_" + std::to_string(i) + ".png";
            // std::string save_file_detected = "src/misora2_pressure/20250807_detected/distance-30/pic_" + std::to_string(i) + ".png";
            // std::string save_file_detected = "src/misora2_pressure/20250807_detected/distance-45/pic_" + std::to_string(i) + ".png";
            // std::string save_file_cropped = "../20250905_Meter/cropped/meter_" + std::to_string(i) + ".jpg";
            // std::string save_file_detected = "../20250905_Meter/detected/meter_" + std::to_string(i) + ".jpg";
            std::string save_file_cropped = "../OriginalData_640/cropped/meter_" + std::to_string(i) + ".jpg";
            std::string save_file_detected = "../OriginalData_640/detected/meter_" + std::to_string(i) + ".jpg";
            cv::imwrite(save_file_cropped, trimmed);
            cv::imwrite(save_file_detected,boxed);
        }
    } 

    return 0;
}
