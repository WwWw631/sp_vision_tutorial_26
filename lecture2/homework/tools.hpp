#pragma once
#include <opencv2/opencv.hpp>


cv::Mat resizeAndCenter(const cv::Mat& src, int canvas_size,
                        double &scale, int &offset_x, int &offset_y);
