#include <opencv2/opencv.hpp>
int main()
{
    // 读取图片
    cv::Mat bgr_img = cv::imread("./img/test_1.jpg");
    if (bgr_img.empty())
    {
        std::cout << "fail to load image" << std::endl;
        return -1;
    }
    cv::resize(bgr_img, bgr_img, cv::Size(bgr_img.cols*0.8, bgr_img.rows*0.8));
    // 分离通道并分别显示
    std::vector<cv::Mat> bgr_channels;
    cv::split(bgr_img, bgr_channels);
    cv::imshow("B_channel", bgr_channels[0]);
    cv::imshow("G_channel", bgr_channels[1]);
    cv::imshow("R_channel", bgr_channels[2]);
    cv::waitKey(0);
    return 0;
}