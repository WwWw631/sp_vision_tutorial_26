#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_buff
{

class Buff_Solver
{
public:
    Buff_Solver();

    // 解算单个叶片中心在相机坐标系下的位置
    cv::Point3f solvePnP(const std::vector<cv::Point2f>& image_points,
                         const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    // 根据多帧叶片中心位置，拟合旋转中心
    cv::Point2f computeRotationCenter(const std::vector<cv::Point3f>& centers);

private:
    // 单个叶片的四个角点（单位：mm）
    std::vector<cv::Point3f> model_points_;

    // 五个叶片的空间中心点
    std::vector<cv::Point3f> blade_centers_;
};

}  // namespace auto_buff

#endif  // SOLVER_HPP
