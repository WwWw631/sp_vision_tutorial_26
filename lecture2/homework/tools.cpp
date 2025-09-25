#include "tools.hpp"
#include <fmt/core.h> 

cv::Mat resizeAndCenter(const cv::Mat& src, int canvas_size,
                        double &scale, int &offset_x, int &offset_y)
{
    int original_width = src.cols;
    int original_height = src.rows;

    double scale_x = static_cast<double>(canvas_size) / original_width;
    double scale_y = static_cast<double>(canvas_size) / original_height;
    scale = std::min(scale_x, scale_y);

    int new_width = static_cast<int>(original_width * scale);
    int new_height = static_cast<int>(original_height * scale);

    offset_x = (canvas_size - new_width) / 2;
    offset_y = (canvas_size - new_height) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_width, new_height));

    cv::Mat canvas = cv::Mat::zeros(canvas_size, canvas_size, src.type());

    resized.copyTo(canvas(cv::Rect(offset_x, offset_y, new_width, new_height)));

    fmt::print("Scale: {:.3f}, Offset X: {}, Offset Y: {}\n", scale, offset_x, offset_y);

    return canvas;
}
