#include "io/my_camera.hpp"               // 相机封装
#include "tasks/yolos/yolov5.hpp"               // YOLOV5 检测器类
#include "tools/img_tools.hpp"            // draw_points 函数
#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    myCamera cam;

    auto_aim::YOLOV5 yolo("./configs/yolo.yaml", false);

    int frame_count = 0;

    while (1) {
        cv::Mat img = cam.read();
        if (img.empty()) {
            std::cerr << "[main] 读取图像失败，跳过本帧。" << std::endl;
            continue;
        }

        frame_count++;

        auto detections = yolo.detect(img, frame_count); 

        for (const auto &armor : detections) {

            tools::draw_points(img, armor.points, cv::Scalar(0, 0, 255)); 
        }

        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("Detection", img);

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    return 0;
}
