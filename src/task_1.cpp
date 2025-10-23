#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;
  auto log = tools::logger();
  log->info("=== Task 1: 赛前⾃瞄⾃检 启动 ===");

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
  // Your code start

    // 1. 读取相机图像和时间戳
    camera.read(img, t);
    if (img.empty()) {
      // 图像为空，暂停1ms，防止空转
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // 2. 获取云台当前姿态四元数 q 和时间戳 t
    q = gimbal.q(t);
    
    // 3. 获取云台当前工作模式
    auto mode = gimbal.mode();

    // 4. 初始化发送给云台的控制变量
    float send_yaw = 0.f;
    float send_pitch = 0.f;
    bool control_gimbal = false; // 是否控制云台标志

    // 5. 检查是否处于自瞄模式
    if (mode == io::GimbalMode::AUTO_AIM) {
      // 6. 设置解算器(solver)的姿态，用于坐标系转换
      solver.set_R_gimbal2world(q);

      // 7. 使用YOLO检测图像中的装甲板
      auto armors = yolo.detect(img);

      // 8. 检查是否检测到装甲板
      if (!armors.empty()) {
        // 9. 选择目标 (任务一中，手持一个，选第一个即可)
        auto & target_armor = armors.front();

        // 10. 使用 solver 解算装甲板在世界坐标系下的3D坐标
        solver.solve(target_armor);

        // 11. 从目标3D坐标计算云台应转向的 Yaw 和 Pitch
        // 世界坐标系原点即云台中心, x-前, y-左, z-上
        // target_armor.xyz_in_world 是一个 Eigen::Vector3d
        auto target_xyz = target_armor.xyz_in_world;

        // Yaw: std::atan2(y, x)
        send_yaw = std::atan2(target_xyz.y(), target_xyz.x());
        
        // Pitch: std::atan2(z, sqrt(x^2 + y^2))
        double xy_distance = std::sqrt(target_xyz.x() * target_xyz.x() + target_xyz.y() * target_xyz.y());
        send_pitch = -std::atan2(target_xyz.z(), xy_distance);
        
        // 12. 设置控制标志为 true
        control_gimbal = true;
      }
      // 如果 armors.empty()，control_gimbal 保持 false，云台将不响应
    
    } else {
      // 如果不在自瞄模式，也重置控制标志
      control_gimbal = false;
    }

    // 13. 发送控制指令给云台
    // control_gimbal=true: 控制云台转向 (send_yaw, send_pitch)
    // control_gimbal=false: 不控制云台 (此时 send_yaw, send_pitch 会被忽略)
    // fire=false: 任务一不要求开火
    gimbal.send(control_gimbal, false, send_yaw, send_pitch);

    // 14. (考核要求 1) 使用 Plotter 输出发送给云台的控制命令
    // 无论是否控制，都发送当前的目标值 (不控制时为0)
    plotter.plot(nlohmann::json{
      {"task1.send_yaw", send_yaw},
      {"task1.send_pitch", send_pitch}
    });

    cv::imshow("Task 1 - Press ESC to exit", img);
    
    if (cv::waitKey(1) == 27) { // 27 是 ESC 键
      exiter.exit(); // 触发 exiter 退出循环
    }
    
  }



  return 0;
}//task_1.cpp