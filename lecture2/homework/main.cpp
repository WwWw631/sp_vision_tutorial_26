#include <opencv2/opencv.hpp>
#include "tools.hpp"
#include <fmt/core.h>

int main()
{
    cv::Mat img = cv::imread("../img/test_1.jpg");
    if (img.empty()) {
        fmt::print("Failed to load image\n");
        return -1;
    }

    double scale;
    int offset_x, offset_y;
    cv::Mat result = resizeAndCenter(img, 640, scale, offset_x, offset_y);

    cv::imshow("Resized Image", result);
    cv::waitKey(0);
    return 0;
}
