#ifndef AUGMENT_HPP
#define AUGMENT_HPP

#include <opencv2/core/mat.hpp>  // cv::Mat 定義
#include <opencv2/core/types.hpp> // cv::Size, cv::Point2f 定義
#include <utility>                // std::pair

class Augment{
    public:
    static void letterbox(const cv::Mat& image,
        cv::Mat& outImage,
        const cv::Size& newShape = cv::Size(640, 640),
        cv::Scalar_<double> color = cv::Scalar(), bool auto_ = true,
        bool scaleFill = false,
        bool scaleUp = true,
        int stride = 32
    );

    // 定数として外部リンケージのない静的メンバ変数を宣言
    static const std::pair<float, cv::Point2f> DEFAULT_RATIO_PAD;

    cv::Mat scale_image(const cv::Mat& resized_mask,
                        const cv::Size& im0_shape,
                        const std::pair<float, cv::Point2f>& ratio_pad = DEFAULT_RATIO_PAD);

    static void scale_image2(cv::Mat& scaled_mask,
                             const cv::Mat& resized_mask,
                             const cv::Size& im0_shape,
                             const std::pair<float, cv::Point2f>& ratio_pad = DEFAULT_RATIO_PAD);


};
#endif //AUGMENT_HPP