#include "buff_solver.hpp"
#include <opencv2/calib3d.hpp>
#include <numeric>
#include <cmath>

namespace auto_buff
{

Buff_Solver::Buff_Solver()
{
    // 单个叶片的尺寸（mm）
    const float w = 300.0f;  // 圆靶直径
    const float h = 300.0f;  // 圆靶直径

    // 单个叶片的模型点（以叶片中心为原点）
    model_points_ = {
        {-w/2, -h/2, 0.0f},
        { w/2, -h/2, 0.0f},
        { w/2,  h/2, 0.0f},
        {-w/2,  h/2, 0.0f}
    };

    // 计算五个叶片中心（绕旋转中心一圈）
    const float R = 700.0f;     // 内径半径
    const float z_center = 2300.0f;  // 中心高度
    for (int i = 0; i < 5; ++i)
    {
        float angle_deg = i * 72.0f;
        float angle_rad = angle_deg * CV_PI / 180.0f;
        float x = R * cos(angle_rad);
        float y = R * sin(angle_rad);
        blade_centers_.emplace_back(cv::Point3f(x, y, z_center));
    }
}

cv::Point3f Buff_Solver::solvePnP(const std::vector<cv::Point2f>& image_points,
                                  const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
    if (image_points.size() != 4)
        return cv::Point3f(0,0,0);

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(model_points_, image_points, cameraMatrix, distCoeffs,
                                rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

    if (!success)
        return cv::Point3f(0,0,0);

    return cv::Point3f(tvec); // 相机坐标系下的中心
}

cv::Point2f Buff_Solver::computeRotationCenter(const std::vector<cv::Point3f>& centers)
{
    if (centers.size() < 3)
        return cv::Point2f(0,0);

    std::vector<cv::Point2f> pts;
    for (auto& c : centers)
        pts.emplace_back(c.x, c.y);

    cv::RotatedRect ellipse = cv::fitEllipse(pts);
    return ellipse.center;
}

} // namespace auto_buff
