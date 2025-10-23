#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"

#include "tools/trajectory.hpp" 
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
  log->info("=== Task 2: 击打静止靶 启动 ===");

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
 

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  int shots_fired = 0;                     // 已发射子弹数
  const int SHOTS_PER_TARGET = 10;         // 每个目标的射击上限
  int stable_counter = 0;                  // 目标稳定帧数计数器
  const int STABLE_THRESHOLD_FRAMES = 10;  // 稳定10帧后开始射击
  bool is_target_stable = false;           // 当前目标是否稳定
  
  Eigen::Vector3d last_target_xyz = {0, 0, 0}; // 上一帧目标位置，用于检测是否移动
  float stable_send_yaw = 0.f;           // 稳定后的目标Yaw
  float stable_send_pitch = 0.f;         // 稳定后的目标Pitch (已补偿重力)

  auto last_shot_time = std::chrono::steady_clock::now(); // 上次射击时间
  const auto SHOT_COOLDOWN = 200ms;       // 射击间隔 (防止过快)


  while (!exiter.exit()) {
// 1. 读取相机图像和时间戳
    camera.read(img, t);
    if (img.empty()) {
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // 2. 获取云台姿态 和 模式
    q = gimbal.q(t);
    auto mode = gimbal.mode();
    auto gimbal_state = gimbal.state(); // 获取云台状态，包含弹速

    // 3. 初始化控制变量
    float send_yaw = 0.f;
    float send_pitch = 0.f;
    bool control_gimbal = false;
    bool fire = false;

    // 4. 检查是否处于自瞄模式
    if (mode == io::GimbalMode::AUTO_AIM) {
      solver.set_R_gimbal2world(q);
      auto armors = yolo.detect(img);

      if (!armors.empty()) {
        control_gimbal = true;
        auto & target_armor = armors.front(); // 选择第一个目标
        solver.solve(target_armor);
        auto target_xyz = target_armor.xyz_in_world;
        
        // 5. 检查目标是否移动
        double pos_diff = (target_xyz - last_target_xyz).norm();
        if (pos_diff < 0.05) { // 假设位置变化小于5cm视为稳定
          stable_counter++;
        } else {
          // 目标移动，重置所有状态
          stable_counter = 0;
          shots_fired = 0;
          is_target_stable = false;
          log->info("目标移动，重置射击计数");
        }
        last_target_xyz = target_xyz;

        // 6. 计算基础Yaw和Pitch (未补偿)
        double xy_distance = std::sqrt(target_xyz.x() * target_xyz.x() + target_xyz.y() * target_xyz.y());
        float base_yaw = std::atan2(target_xyz.y(), target_xyz.x());
        // (注意: trajectory.solve 应该需要未补偿的 pitch 或 z 高度)

        // 7. 弹道解算 (重力补偿)
        float bullet_speed = gimbal_state.bullet_speed;
        if (bullet_speed < 10.f) bullet_speed = 20.f; // 防止弹速为0
        
        // 假设 trajectory.solve 返回的是补偿后的绝对Pitch角度
        // API 可能为: solve(水平距离, 目标Z高度, 弹速)
        tools::Trajectory traj_calc(bullet_speed, xy_distance, target_xyz.z());
        float compensated_pitch = traj_calc.unsolvable ? 0.f : -(float)traj_calc.pitch;

        // 8. 射击与状态锁定
        if (stable_counter > STABLE_THRESHOLD_FRAMES) {
          if (!is_target_stable) {
            // 刚进入稳定状态，锁定当前的目标角度
            is_target_stable = true;
            stable_send_yaw = base_yaw;
            stable_send_pitch = compensated_pitch;
            log->info("目标稳定, 锁定角度 Yaw: {:.2f}, Pitch: {:.2f}", stable_send_yaw, stable_send_pitch);
          }
          
          // 使用锁定的角度进行瞄准
          send_yaw = stable_send_yaw;
          send_pitch = stable_send_pitch;

          // 9. 开火逻辑 (要求单发)
          auto now = std::chrono::steady_clock::now();
          if (shots_fired < SHOTS_PER_TARGET && (now - last_shot_time > SHOT_COOLDOWN)) {
            // (可以增加额外检查：gimbal.state().yaw 是否已接近 send_yaw)
            fire = true;
            shots_fired++;
            last_shot_time = now;
            log->debug("Fire! shot count: {}", shots_fired);
          }
        } else {
          // 目标不稳定时，只跟随，不射击
          send_yaw = base_yaw;
          send_pitch = compensated_pitch; // 即使不稳定也发送补偿后的角度
        }
      } else {
        // 视野中无目标
        control_gimbal = false;
        stable_counter = 0;
        shots_fired = 0;
        is_target_stable = false;
      }
    } else {
      // 非自瞄模式
      control_gimbal = false;
      stable_counter = 0;
      shots_fired = 0;
      is_target_stable = false;
    }

    // 10. 发送指令
    gimbal.send(control_gimbal, fire, send_yaw, send_pitch);

    // 11. (考核要求 3) Plotter输出
    plotter.plot(nlohmann::json{
      {"task2.send_yaw", send_yaw},
      {"task2.send_pitch", send_pitch},
      {"task2.shots_fired", shots_fired},
      {"task2.is_stable", is_target_stable}
    });

    cv::imshow("Task 2 - Press ESC to exit", img);
    
    if (cv::waitKey(1) == 27) { // 27 是 ESC 键
      exiter.exit(); // 触发 exiter 退出循环
    }

  }

  log->info("=== Task 2 结束 ===");
  return 0;
}//task_2.cpp