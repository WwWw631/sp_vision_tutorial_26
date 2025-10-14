#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include "buff_type.hpp"
#include "tools/img_tools.hpp"
#include "yolo11_buff.hpp"
namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector();
  std::vector<FanBlade> detect(cv::Mat & bgr_img);
  // 输入关键点，输出相机坐标系下的目标中心
  cv::Point3f solvePnP(const std::vector<cv::Point2f>& image_points, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

  // 计算旋转中心（R 标中心）
  cv::Point2f computeRotationCenter(const std::vector<cv::Point3f>& centers_2d);

private:
  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);
  YOLO11_BUFF MODE_;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP