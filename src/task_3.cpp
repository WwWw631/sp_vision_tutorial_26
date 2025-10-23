#include <chrono>
#include <optional> // EKF追踪器需要
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"     // 任务三核心 EKF
#include "tools/trajectory.hpp" // 任务三需要弹道解算
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
  log->info("=== Task 3: 击打小陀螺 启动 ===");

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  // --- 任务三 状态变量 ---
  std::optional<auto_aim::Target> target_tracker; // EKF 追踪器
  
  // EKF 初始协方差 P0 (x, vx, y, vy, z, vz, a, w, r, l, h)
  // 需要根据 target.cpp 的 TODO 调优
  Eigen::VectorXd P0_dig(11);
  P0_dig << 1, 1e3, 1, 1e3, 1, 1e3, 1, 1e2, 1, 1, 1; // 示例值

  int shots_fired = 0;
  const int SHOTS_PER_TARGET = 10;
  auto last_shot_time = std::chrono::steady_clock::now();
  const auto SHOT_COOLDOWN = 200ms; // 射击间隔
  
  double last_w_for_reset = 0.0; // 用于检测转速变化
  const double GIMBAL_REACTION_TIME = 0.05; // 假设的云台+系统延迟 (50ms)
  const double FIRE_WINDOW_RAD = 0.08; // 提前量的开火窗口 (约 3 度)
  const int ARMOR_NUM = 4; // 假设目标是4装甲板

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
  camera.read(img, t);
    if (img.empty()) continue;

    q = gimbal.q(t);
    auto mode = gimbal.mode();
    auto gimbal_state = gimbal.state();
    float bullet_speed = gimbal_state.bullet_speed;
    if (bullet_speed < 10.f) bullet_speed = 20.f; 

    // 3. 初始化控制变量
    float send_yaw = 0.f;
    float send_pitch = 0.f;
    bool control_gimbal = false;
    bool fire = false;
    double ekf_w = 0.0; // EKF拟合的角速度

    if (mode == io::GimbalMode::AUTO_AIM) {
      solver.set_R_gimbal2world(q);
      auto armors = yolo.detect(img);

      if (armors.empty()) {
        // 目标丢失，只进行预测 (如果追踪器存在)
        if (target_tracker) {
          target_tracker->predict(t);
        }
        // (可以选择在丢失N帧后 reset 追踪器)
      } 
      else {
        // 发现目标
        control_gimbal = true;
        auto & best_armor = armors.front();
        solver.solve(best_armor);

        if (!target_tracker) {
          // 首次发现，初始化EKF
          target_tracker.emplace(best_armor, t, P0_dig, 0.2, ARMOR_NUM);
          log->info("EKF 追踪器初始化");
        } else {
          // 持续更新EKF
          target_tracker->predict(t);
          target_tracker->update(best_armor);
        }
      }

      // 如果EKF已激活并收敛
      if (target_tracker && target_tracker->convergened()) {
        auto ekf_state = target_tracker->ekf_x();
        ekf_w = ekf_state[7]; // 获取角速度 w

        // 检查转速是否变化，用于重置射击计数
        if (std::abs(ekf_w - last_w_for_reset) > 2.0) { // 变化阈值
          log->info("检测到转速变化，重置射击计数。 old_w: {:.2f}, new_w: {:.2f}", last_w_for_reset, ekf_w);
          shots_fired = 0;
          last_w_for_reset = ekf_w;
        }
        

        // --- 4. 瞄准中心 (Tip) ---
        Eigen::Vector3d center_xyz = {ekf_state[0], ekf_state[2], ekf_state[4]}; // x, y, z
        double xy_distance = std::sqrt(center_xyz.x() * center_xyz.x() + center_xyz.y() * center_xyz.y());
        
        send_yaw = std::atan2(center_xyz.y(), center_xyz.x());

        tools::Trajectory traj_calc(bullet_speed, xy_distance, center_xyz.z());
        send_pitch = traj_calc.unsolvable ? 0.f : -(float)traj_calc.pitch;
        
        // --- 5. 预测开火 ---
        double t_flight = xy_distance / bullet_speed;
        double t_pred = t_flight + GIMBAL_REACTION_TIME;

        bool should_fire = false;
        double current_base_angle = ekf_state[6]; // Armor 0 的当前角度

        double min_pred_error = 1e9; // 用于记录最小的预测误差

        for (int i = 0; i < ARMOR_NUM; i++) {
          double armor_angle_now = tools::limit_rad(current_base_angle + i * 2 * CV_PI / ARMOR_NUM);
          double armor_angle_pred = tools::limit_rad(armor_angle_now + ekf_w * t_pred);
          
          // 检查预测的角度是否在我们的瞄准线 (send_yaw) 附近
          double angle_error = tools::limit_rad(armor_angle_pred - send_yaw);

          // === 记录最小误差 ===
          if (std::abs(angle_error) < min_pred_error) {
            min_pred_error = std::abs(angle_error);
          }

          if (std::abs(angle_error) < FIRE_WINDOW_RAD) {
            should_fire = true;
            break;
          }
        }

        auto now = std::chrono::steady_clock::now();

        //Fire - YES/NO
        if (should_fire ) {
          fire = true;
          //shots_fired++;
          //last_shot_time = now;
          
          // 当开火时，打印 "FIRE" 日志
          log->info(
            "*** FIRE *** w={:.2f}, t_pred={:.3f}s, shots={}",
            ekf_w, t_pred, shots_fired
          );

        } else { 
          // 当不开火时，打印 "PRED" (预测) 日志
          log->info(
            "PRED: w={:.2f}, t_pred={:.3f}s, min_err={:.3f}rad, FIRE_WIN={:.3f}rad",
            ekf_w, t_pred, min_pred_error, FIRE_WINDOW_RAD
          );
        }
        
        } else {
        // EKF 未激活或未收敛
        control_gimbal = false; 
        }
    } 
    else {
      // 非自瞄模式，重置一切
      target_tracker.reset();
      shots_fired = 0;
      last_w_for_reset = 0;
      control_gimbal = false;
    }

    // 10. 发送指令
    gimbal.send(control_gimbal, fire, send_yaw, send_pitch);

    // 11. Plotter输出
    plotter.plot(nlohmann::json{
      {"task3.send_yaw", send_yaw},
      {"task3.send_pitch", send_pitch},
      {"task3.ekf_w", - ekf_w} // 输出拟合角速度
    });

    cv::imshow("Task 3 - Press ESC to exit", img);
    
    if (cv::waitKey(1) == 27) { // 27 是 ESC 键
      exiter.exit(); // 触发 exiter 退出循环
    }

  }

  log->info("=== Task 3 结束 ===");

  return 0;
}//task_3.cpp