#include "misora2_pressure/measurement.hpp"

// 12時方向(0,-1) を 0°、時計回りを正で [0,360) に正規化
double Measurement::calculate_angle_12cw(const cv::Point2d& center, const cv::Vec4i& line) {
    cv::Point2d p1(line[0], line[1]);
    cv::Point2d p2(line[2], line[3]);

    // 中心から遠い端点＝針先端
    double d1 = cv::norm(p1 - center);
    double d2 = cv::norm(p2 - center);
    cv::Point2d tip = (d1 > d2) ? p1 : p2;

    // 針ベクトルを正規化
    cv::Point2d v = tip - center;
    double n = std::hypot(v.x, v.y);
    if (n < 1e-9) return std::numeric_limits<double>::quiet_NaN();
    v.x /= n; v.y /= n;

    // 12時方向ベクトル
    const cv::Point2d ref(0.0, -1.0);

    double dot = v.x * ref.x + v.y * ref.y;
    dot = std::max(-1.0, std::min(1.0, dot));
    double angle_deg = std::acos(dot) * 180.0 / CV_PI;

    // 左半平面（v.x<0）の場合は 360-θ
    if (v.x < 0.0) angle_deg = 360.0 - angle_deg;

    return angle_deg;
}

cv::Mat Measurement::make_skeleton(const cv::Mat& morph) {
    cv::Mat skel(morph.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat bin = morph.clone(), temp, eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
    while (true) {
        cv::erode(bin, eroded, element);
        cv::dilate(eroded, temp, element);
        cv::subtract(bin, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(bin);
        if (cv::countNonZero(bin) == 0) break;
    }
    return skel;
}

// 画像から最終線分と中心を検出して返す
Measurement::AngleInput Measurement::detect_line(const cv::Mat& image){
    Measurement::AngleInput out{cv::Point2d(0,0), cv::Vec3f(0.0,0.0,0.0), cv::Vec4i(0,0,0,0), false};

    int height = image.rows;

    // 前処理
    cv::Mat gray;  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    cv::Mat bit;
    cv::adaptiveThreshold(gray, bit, 255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
        11, 2);

    cv::Mat morph;
    cv::morphologyEx(bit, morph, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));

    cv::Mat skel = make_skeleton(morph);

    // 円検出
    std::vector<cv::Vec3f> circles;
    for (int i = 1; i < 100; i++) {
        circles.clear();
        int dp = 2;
        double minDist = height / 4.0;
        double param1 = 400.0 / i;
        double param2 = param1 / 2.0;
        int minRadius = static_cast<int>(height / 4.0);
        int maxRadius = static_cast<int>(height / 2.0);
        cv::HoughCircles(morph, circles, cv::HOUGH_GRADIENT,
                         dp, minDist, param1, param2, minRadius, maxRadius);
        if (!circles.empty()) break;
    }

    // 円中心
    cv::Point centerPt = circles.empty()
        ? cv::Point(image.cols/2, image.rows/2)
        : cv::Point(cvRound(circles[0][0]), cvRound(circles[0][1]));

    double r = circles.empty() ? -1 : cvRound(circles[0][2]);
    out.center = centerPt;

    // 円外マスク
    cv::Mat skelROI = skel.clone();
    if (!circles.empty()) {
        int r = cvRound(circles[0][2]);
        cv::Mat circleMask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::circle(circleMask, centerPt, std::max(0, r-1), cv::Scalar(255), -1);
        cv::erode(circleMask, circleMask,
                  cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
        cv::bitwise_and(skel, circleMask, skelROI);
    }

    // 線分検出
    std::vector<cv::Vec4i> lines;
    const double rho = 1.0;
    const double theta = CV_PI / 180.0; // 必要に応じ 0.5° 等へ
    const int    houghThresh   = 30;
    const double minLineLength = 0.3 * r;
    const double maxLineGap    = 5;

    cv::HoughLinesP(skelROI, lines, rho, theta, houghThresh, minLineLength, maxLineGap);
    if (lines.empty()) {
        std::cerr << "No lines detected on masked skeleton." << std::endl;
        return out;
    }

    // 中心に最も近い端点を持つ線を採用
    double minDist = 1e9;
    cv::Vec4i bestLine;
    for (const auto& L : lines) {
        cv::Point p1(L[0], L[1]), p2(L[2], L[3]);
        double dmin = std::min(cv::norm(p1 - centerPt), cv::norm(p2 - centerPt));
        if (dmin < minDist) { minDist = dmin; bestLine = L; }
    }

    // 出力値を設定
    out.line  = bestLine;
    out.circle = circles[0];
    out.valid = true;

    return out;
}

// メーター種別分類
int Measurement::classify_meter_type(const cv::Mat& image, const std::string& model_path, int imgsize){
    try {
        if (image.empty()) {
            std::cerr << "[CLS] failed to read image "<< std::endl;
            return -1;
        }
        cv::Mat rgb; cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);

        int s = int(imgsize * 1.1);
        double scale = std::max(s / double(rgb.cols), s / double(rgb.rows));
        cv::Mat resized; cv::resize(rgb, resized, cv::Size(), scale, scale, cv::INTER_AREA);
        int x0 = std::max(0, (resized.cols - imgsize) / 2);
        int y0 = std::max(0, (resized.rows - imgsize) / 2);
        cv::Mat crop = resized(cv::Rect(x0, y0, std::min(imgsize, resized.cols - x0),
                                                  std::min(imgsize, resized.rows - y0))).clone();
        if (crop.cols != imgsize || crop.rows != imgsize) {
            cv::resize(crop, crop, cv::Size(imgsize, imgsize), 0, 0, cv::INTER_AREA);
        }
        crop.convertTo(crop, CV_32F, 1.0/255.0);

        // Normalize (ImageNet)
        const cv::Scalar mean(0.485, 0.456, 0.406);
        const cv::Scalar stdv(0.229, 0.224, 0.225);
        cv::Mat normed = (crop - mean) / stdv;

        // HWC -> NCHW
        std::vector<float> input(1 * 3 * imgsize * imgsize);
        std::vector<cv::Mat> ch(3);
        cv::split(normed, ch);
        for (int c = 0; c < 3; ++c) {
            std::memcpy(
                input.data() + c * imgsize * imgsize,
                ch[c].ptr<float>(),
                sizeof(float) * imgsize * imgsize
            );
        }

        // 2) ONNX Runtime で推論
        static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "gaugecls");
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        // opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        Ort::Session session(env, model_path.c_str(), opts);

        Ort::AllocatorWithDefaultOptions allocator;
        Ort::AllocatedStringPtr in_name_alloc = session.GetInputNameAllocated(0, allocator);
        Ort::AllocatedStringPtr out_name_alloc = session.GetOutputNameAllocated(0, allocator);
        const char* input_name  = in_name_alloc.get();
        const char* output_name = out_name_alloc.get();

        std::vector<int64_t> input_shape = {1, 3, imgsize, imgsize};
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem, input.data(), input.size(), input_shape.data(), input_shape.size()
        );

        auto out_tensors = session.Run(
            Ort::RunOptions{nullptr},
            &input_name, &input_tensor, 1,
            &output_name, 1
        );
        // logits (1, num_classes)
        float* logits = out_tensors.front().GetTensorMutableData<float>();
        size_t num_classes = out_tensors.front().GetTensorTypeAndShapeInfo().GetShape()[1];

        // 3) argmax でクラスID → meter_type へマッピング
        size_t argmax = 0;
        float best = logits[0];
        for (size_t i = 1; i < num_classes; ++i) {
            if (logits[i] > best) { best = logits[i]; argmax = i; }
        }


        // ラベル名
        // const char* label = (argmax < sizeof(kClassNames)/sizeof(kClassNames[0]))? kClassNames[argmax] : "UNKNOWN";

        // 学習フォルダ順（ImageFolderのデフォルト）は**辞書順**:
        // ["0_25MPa", "1_0MPa", "1_6MPa"] と仮定（必要ならクラス順をここで合わせてください）
        // それぞれ既存関数の meter_type へ変換:
        // 1=1.0MPa, 2=0.25MPa, 3=1.6MPa
        //   idx 0 -> "0_25MPa" -> 2
        //   idx 1 -> "1_0MPa"  -> 1
        //   idx 2 -> "1_6MPa"  -> 3
        static const int idx_to_meter_type[] = {2, 1, 3};
        if (argmax < sizeof(idx_to_meter_type)/sizeof(int)) {
            int mt = idx_to_meter_type[argmax];
            // std::cout << "[CLS] meter_type=" << mt << "(" << label << ")" << std::endl;

            return mt;
        } else {
            std::cerr << "[CLS] unexpected class index: " << argmax << std::endl;
            return -1;
        }
    } catch (const std::exception& e) {
        std::cerr << "[CLS] exception: " << e.what() << std::endl;
        return -1;
    }
}
// 角度(12時=0°, 時計回り, 単位:度) → 圧力(MPa)
// 角度(12時=0°, 時計回り, 単位:度) ＋ メーター種別 → 圧力(MPa)
// meter_type: 1=1.0MPa, 2=0.25MPa, 3=1.6MPa
double Measurement::pressure_value_from_angle(double angle_deg, std::string meter_type) {
    // [0,360) に正規化
    double a = std::fmod(angle_deg, 360.0);
    if (a < 0.0) a += 360.0;

    double pressure = 0.0;

    if( meter_type == "1") { // 1.0 MPa
        if (228.0 <= a && a < 270.0) {
            pressure = (a - 228.0) * (0.16 / (270.0 - 228.0));
        } else if (270.0 <= a && a <= 360.0) {
            pressure = 0.16 + (a - 270.0) * (0.34 / (360.0 - 270.0));
        } else if (0.0 <= a && a <= 90.0) {
            pressure = 0.50 + (a / 90.0) * (0.84 - 0.50);
        } else if (90.0 < a && a <= 132.0) {
            pressure = 0.84 + ((a - 90.0) / (132.0 - 90.0)) * (1.00 - 0.84);
        } else {
            pressure = 0.0; 
        }
    }
    else if( meter_type == "2") { // 0.25 MPa
        if (228.0 <= a && a < 270.0) {
            pressure = (a - 228.0) * (0.04 / (270.0 - 228.0));
        } else if (270.0 <= a && a <= 360.0) {
            pressure = 0.04 + (a - 270.0) * (0.125 / (360.0 - 270.0));
        } else if (0.0 <= a && a <= 90.0) {
            pressure = 0.125 + (a / 90.0) * (0.21 - 0.125);
        } else if (90.0 < a && a <= 132.0) {
            pressure = 0.21 + ((a - 90.0) / (132.0 - 90.0)) * (0.25 - 0.21);
        } else {
            pressure = 0.0;
        }
    }
    else if( meter_type == "3") { // 1.6 MPa
        if (228.0 <= a && a < 270.0) {
            pressure = (a - 228.0) * (0.256 / (270.0 - 228.0));
        } else if (270.0 <= a && a <= 360.0) {
            pressure = 0.256 + (a - 270.0) * (0.8 / (360.0 - 270.0));
        } else if (0.0 <= a && a <= 90.0) {
            pressure = 0.8 + (a / 90.0) * (1.344 - 0.8);
        } else if (90.0 < a && a <= 132.0) {
            pressure = 1.344 + ((a - 90.0) / (132.0 - 90.0)) * (1.6 - 1.344);
        } else {
            pressure = 0.0;
        }
    }
    else{
        pressure = 0.0; 
    }

    return pressure;
}
