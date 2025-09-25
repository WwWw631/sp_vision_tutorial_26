#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_image>\n";
        return 1;
    }
    std::string inPath = argv[1];
    std::cout << "Input image path: " << inPath;
    // 读取图片
    cv::Mat bgr_img = cv::imread(inPath);
    if (bgr_img.empty())
    {
        std::cout << "fail to load image" << std::endl;
        return -1;
    }
    // 转灰度
    cv::Mat gray_img;
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::imshow("1_gray_img", gray_img);
    // 二值化
    double threshold_ = 200;
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
    cv::imshow("2_binary_img", binary_img);
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::Mat contour_img = bgr_img.clone();
    cv::drawContours(contour_img, contours, -1, cv::Scalar(0, 0, 255), 6);
    cv::imshow("3_contours", contour_img);
    // 参数
    double min_lightbar_ratio_ = 2.0;  // 最小长宽比
    double max_lightbar_ratio_ = 9.0;  // 最大长宽比
    double min_lightbar_length_ = 5.0; // 最小长度

    // 遍历轮廓`
    for (const auto &contour : contours)
    {
        if (contour.size() < 5)
            continue; // 太小的轮廓忽略
        auto rect = cv::minAreaRect(contour);

        // 计算长宽
        float width = std::min(rect.size.width, rect.size.height);
        float height = std::max(rect.size.width, rect.size.height);
        float ratio = height / width;

        // 几何筛选
        if (ratio > min_lightbar_ratio_ && ratio < max_lightbar_ratio_ &&
            height > min_lightbar_length_)
        {
            // 判断颜色：蓝通道 vs 红通道
            int red_sum = 0, blue_sum = 0;
            for (const auto &p : contour)
            {
                auto pix = bgr_img.at<cv::Vec3b>(p);
                red_sum += pix[2];
                blue_sum += pix[0];
            }

            // 绘制旋转矩形
            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++)
            {
                cv::line(bgr_img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 5);
            }
            std::cout << "Lightbar color: " << (blue_sum > red_sum ? "Blue" : "Red") << std::endl;
        }
    }

    // Show result
    cv::imshow("4_Lightbars", bgr_img);
    cv::waitKey(0);
    return 0;
}
