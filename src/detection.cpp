// detection.cpp
#include "misora2_pressure/detection.hpp"

const std::vector<std::vector<int>> Detection::skeleton = {
    {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13}, {6, 7},
    {6, 8}, {7, 9}, {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}
};

const std::vector<cv::Scalar> Detection::posePalette = {
    cv::Scalar(255, 128, 0), cv::Scalar(255, 153, 51), cv::Scalar(255, 178, 102), cv::Scalar(230, 230, 0),
    cv::Scalar(255, 153, 255), cv::Scalar(153, 204, 255), cv::Scalar(255, 102, 255), cv::Scalar(255, 51, 255),
    cv::Scalar(102, 178, 255), cv::Scalar(51, 153, 255), cv::Scalar(255, 153, 153), cv::Scalar(255, 102, 102),
    cv::Scalar(255, 51, 51), cv::Scalar(153, 255, 153), cv::Scalar(102, 255, 102), cv::Scalar(51, 255, 51),
    cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255)
};

const std::vector<int> Detection::limbColorIndices = {9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16};
const std::vector<int> Detection::kptColorIndices = {16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9};

cv::Scalar Detection::generateRandomColor(int numChannels) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);
    cv::Scalar color;
    for (int i = 0; i < numChannels; i++) {
        color[i] = dis(gen);
    }
    return color;
}

std::vector<cv::Scalar> Detection::generateRandomColors(int class_names_num, int numChannels) {
    std::vector<cv::Scalar> colors;
    for (int i = 0; i < class_names_num; i++) {
        colors.push_back(generateRandomColor(numChannels));
    }
    return colors;
}

void Detection::plot_masks(cv::Mat img, std::vector<YoloResults>& result, std::vector<cv::Scalar> color,
    std::unordered_map<int, std::string>& names)
{
    cv::Mat mask = img.clone();
    for (size_t i = 0; i < result.size(); i++)
    {
        float left, top;
        left = result[i].bbox.x;
        top = result[i].bbox.y;
        // int color_num = i;
        int& class_idx = result[i].class_idx;
        rectangle(img, result[i].bbox, color[result[i].class_idx], 2);

        // try to get string value corresponding to given class_idx
        std::string class_name;
        auto it = names.find(class_idx);
        if (it != names.end()) {
            class_name = it->second;
        }
        else {
            std::cerr << "Warning: class_idx not found in names for class_idx = " << class_idx << std::endl;
            // then convert it to string anyway
            class_name = std::to_string(class_idx);
        }

        if (result[i].mask.rows && result[i].mask.cols > 0)
        {
            mask(result[i].bbox).setTo(color[result[i].class_idx], result[i].mask);
        }
        std::stringstream labelStream;
        labelStream << class_name << " " << std::fixed << std::setprecision(2) << result[i].conf;
        std::string label = labelStream.str();

    	cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);
        cv::Rect rect_to_fill(left - 1, top - text_size.height - 5, text_size.width + 2, text_size.height + 5);
        cv::Scalar text_color = cv::Scalar(255.0, 255.0, 255.0);
        rectangle(img, rect_to_fill, color[result[i].class_idx], -1);

        putText(img, label, cv::Point(left - 1.5, top - 2.5), cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);
    }
    addWeighted(img, 0.6, mask, 0.4, 0, img); //add mask to src
    resize(img, img, img.size());
}

std::tuple<cv::Mat, cv::Mat> Detection::plot_results(
    cv::Mat img,
    std::vector<YoloResults>& results,
    std::vector<cv::Scalar> color,
    std::unordered_map<int, std::string>& names)
{
    cv::Mat image_with_box = img.clone();
    cv::Mat trimming = cv::Mat(); // 初期値は空画像

    if (results.empty()) {
        std::cerr << "No detections." << std::endl;
        return {trimming, image_with_box};
    }

    const auto& res = results[0];  // 最初の検出だけ使う（必要に応じて変更可）

    // バウンディングボックス描画
    cv::rectangle(image_with_box, res.bbox, color[res.class_idx], 2);

    // ラベル描画
    std::string class_name = names.count(res.class_idx) ? names[res.class_idx] : std::to_string(res.class_idx);
    std::stringstream labelStream;
    labelStream << class_name << " " << std::fixed << std::setprecision(2) << res.conf;
    std::string label = labelStream.str();
    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);
    cv::Rect rect_to_fill(res.bbox.x, res.bbox.y - text_size.height - 5, text_size.width + 2, text_size.height + 5);
    cv::rectangle(image_with_box, rect_to_fill, color[res.class_idx], -1);
    cv::putText(image_with_box, label, cv::Point(res.bbox.x, res.bbox.y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);

    // トリミング画像作成（矩形部分をそのまま切り取る）
    trimming = img(res.bbox).clone();

    return {trimming, image_with_box};
}
