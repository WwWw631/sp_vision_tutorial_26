#include "tasks/buff_detector.hpp"
#include "tasks/buff_solver.hpp"
#include "tools/plotter.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <nlohmann/json.hpp>

using namespace tools;
using namespace auto_buff;

int main()
{
    cv::VideoCapture cap("assets/test.avi"); 
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频文件!" << std::endl;
        return -1;
    }

    Buff_Detector detector;
    Buff_Solver solver;
    Plotter plotter("127.0.0.1", 9870);

    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1000,0,640, 0,1000,360, 0,0,1);
    cv::Mat distCoeffs = cv::Mat::zeros(1,5,CV_64F);

    std::vector<cv::Point3f> history_centers;
    cv::Mat frame;

    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;

        auto fanblades = detector.detect(frame);
        if (fanblades.empty() || fanblades[0].points.size() < 4) {
            cv::imshow("buff tracking", frame);
            if (cv::waitKey(30) == 27) break;
            continue;
        }

        std::vector<cv::Point2f> image_points = {
            fanblades[0].points[0],
            fanblades[0].points[1],
            fanblades[0].points[2],
            fanblades[0].points[3]
        };

        cv::Point3f center_3d = solver.solvePnP(image_points, cameraMatrix, distCoeffs);
        if (center_3d == cv::Point3f(0,0,0)) continue;

        history_centers.push_back(center_3d);
        if (history_centers.size() > 20) history_centers.erase(history_centers.begin());

        cv::Point2f rotation_center_2d(0,0);
        if (history_centers.size() >= 5) {
            rotation_center_2d = solver.computeRotationCenter(history_centers);
        }
        cv::Point3f rotation_center(rotation_center_2d.x, rotation_center_2d.y, 0.f);

        // 任务一：符的中⼼（内环中⼼）
        nlohmann::json data_task1;
        data_task1["armor_center"]["x"] = center_3d.x;
        data_task1["armor_center"]["y"] = center_3d.y;
        plotter.plot(data_task1);

        // 任务二：旋转中⼼（R标位置）
        if (history_centers.size() >= 5) {  
            nlohmann::json data_task2;
            data_task2["rotation_center"]["x"] = rotation_center.x;
            data_task2["rotation_center"]["y"] = rotation_center.y;
            plotter.plot(data_task2);
        }


        // 可视化
        cv::Mat display_img = frame.clone();
        for (int i = 0; i < 4; ++i)
            cv::circle(display_img, image_points[i], 4, {0,255,0}, -1);

        cv::circle(display_img, fanblades[0].center, 5, {0,0,255}, -1);
        cv::circle(display_img, cv::Point(rotation_center_2d.x, rotation_center_2d.y), 5, {255,0,0}, -1);

        cv::imshow("buff tracking", display_img);
        if (cv::waitKey(30) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}