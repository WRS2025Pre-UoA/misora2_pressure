#include "misora2_pressure/measurement.hpp"

int main(int argc, char *argv[]){
    if (argc != 2) {
        std::cerr << "please input file path" << std::endl;
        return 1;
    }
    
    // 画像読み込み
    cv::Mat image = cv::imread(argv[1]);
    if (image.empty()) {
        std::cerr << "Failed to read image: " << argv[1] << std::endl;
        return 0;
    }
    int width  = image.cols;
    int height = image.rows;
    std::cout << "Width: " << width << ", Height: " << height << std::endl;

    // 検出
    Measurement::AngleInput args = Measurement::detect_line(image);
    if (!args.valid) {
        std::cerr << "Detection failed.\n";
        return 0;
    }

    // 座標を出力（要件より）
    cv::Point p1(args.line[0], args.line[1]);
    cv::Point p2(args.line[2], args.line[3]);
    cv::Point centerPt(cvRound(args.circle[0]), cvRound(args.circle[1]));
    std::cout << "Final line endpoints: "
              << "(" << p1.x << "," << p1.y << ") - "
              << "(" << p2.x << "," << p2.y << ")\n";
    std::cout << "Detected circle center: "
              << "(" << centerPt.x << "," << centerPt.y << ")\n";

    // （任意）可視化
    cv::Mat vis = image.clone();
    
    cv::circle(vis, centerPt, cvRound(args.circle[2]), cv::Scalar(0, 0, 255), 2);
    
    cv::circle(vis, centerPt, 4, cv::Scalar(0,255,0), -1);
    cv::line(vis, p1, p2, cv::Scalar(0,0,255), 2);
    cv::imshow("Detection result (line & center)", vis);
    cv::waitKey(0);
    cv::destroyAllWindows();

    // 角度計算
    double angle12cw = Measurement::calculate_angle_12cw(args.center, args.line);
    // 出力（Python関数と同様に角度のみ）
    std::cout << "Angle (12 o'clock, clockwise): " << angle12cw << " deg" << std::endl;

    std::string meter_type = "3"; // 1:1.0MPa, 2:0.25MPa, 3:1.6MPa
    double pressure = Measurement::pressure_value_from_angle(angle12cw, meter_type);
    std::cout << "Pressure: " << pressure << " MPa" << std::endl;

    return 0;
}