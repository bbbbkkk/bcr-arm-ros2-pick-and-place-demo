#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


double durationToSec(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}


builtin_interfaces::msg::Duration secToDuration(double seconds)
{
  builtin_interfaces::msg::Duration duration;

  duration.sec = static_cast<int32_t>(seconds);
  duration.nanosec = static_cast<uint32_t>(
    (seconds - static_cast<double>(duration.sec)) * 1e9
  );

  return duration;
}


void slowDownTrajectory(
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  double scale)
{
  if (scale <= 0.0) {
    return;
  }

  auto & points = plan.trajectory_.joint_trajectory.points;

  for (auto & point : points) {
    double old_time = durationToSec(point.time_from_start);
    double new_time = old_time * scale;
    point.time_from_start = secToDuration(new_time);

    for (auto & velocity : point.velocities) {
      velocity = velocity / scale;
    }

    for (auto & acceleration : point.accelerations) {
      acceleration = acceleration / (scale * scale);
    }
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "reach_target_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  // 速度调快一点
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Pose Target 容忍度，避免 IK 太严格导致失败
  move_group.setGoalPositionTolerance(0.03);
  move_group.setGoalOrientationTolerance(0.35);

  move_group.startStateMonitor(5.0);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(node->get_logger(), "Planning group: %s", PLANNING_GROUP.c_str());

  auto shutdown_and_exit = [&](int code) -> int {
    executor.cancel();

    if (spinner.joinable()) {
      spinner.join();
    }

    rclcpp::shutdown();
    return code;
  };

  auto plan_and_execute_joint =
    [&](const std::map<std::string, double> & target, const std::string & name) -> bool {
      RCLCPP_INFO(node->get_logger(), "Planning to joint target: %s", name.c_str());

      move_group.setStartStateToCurrentState();

      bool target_set = move_group.setJointValueTarget(target);
      if (!target_set) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set joint target: %s", name.c_str());
        return false;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;

      bool plan_success = static_cast<bool>(move_group.plan(plan));
      if (!plan_success) {
        RCLCPP_ERROR(node->get_logger(), "Joint planning failed: %s", name.c_str());
        return false;
      }

      const auto & points_before = plan.trajectory_.joint_trajectory.points;

      if (!points_before.empty()) {
        double original_duration = durationToSec(points_before.back().time_from_start);
        RCLCPP_INFO(
          node->get_logger(),
          "Original joint trajectory duration: %.3f seconds",
          original_duration
        );
      }

      // 1.0 表示不额外放慢
      slowDownTrajectory(plan, 1.0);

      const auto & points_after = plan.trajectory_.joint_trajectory.points;

      if (!points_after.empty()) {
        double scaled_duration = durationToSec(points_after.back().time_from_start);
        RCLCPP_INFO(
          node->get_logger(),
          "Scaled joint trajectory duration: %.3f seconds",
          scaled_duration
        );
      }

      RCLCPP_INFO(node->get_logger(), "Joint planning succeeded. Executing: %s", name.c_str());

      auto execute_result = move_group.execute(plan);

      if (!static_cast<bool>(execute_result)) {
        RCLCPP_ERROR(node->get_logger(), "Joint execution failed: %s", name.c_str());
        return false;
      }

      RCLCPP_INFO(node->get_logger(), "Joint execution succeeded: %s", name.c_str());
      return true;
    };

  auto plan_and_execute_pose =
    [&](const geometry_msgs::msg::Pose & target_pose, const std::string & name) -> bool {
      RCLCPP_INFO(node->get_logger(), "Planning to pose target: %s", name.c_str());

      move_group.setStartStateToCurrentState();
      move_group.setPoseReferenceFrame(move_group.getPlanningFrame());
      move_group.setPoseTarget(target_pose, move_group.getEndEffectorLink());

      moveit::planning_interface::MoveGroupInterface::Plan plan;

      bool plan_success = static_cast<bool>(move_group.plan(plan));

      if (!plan_success) {
        RCLCPP_ERROR(node->get_logger(), "Pose planning failed: %s", name.c_str());
        move_group.clearPoseTargets();
        return false;
      }

      const auto & points_before = plan.trajectory_.joint_trajectory.points;

      if (!points_before.empty()) {
        double original_duration = durationToSec(points_before.back().time_from_start);
        RCLCPP_INFO(
          node->get_logger(),
          "Original pose trajectory duration: %.3f seconds",
          original_duration
        );
      }

      // 1.0 表示不额外放慢；如果又 aborted，可以改成 1.1 或 1.2
      slowDownTrajectory(plan, 1.0);

      const auto & points_after = plan.trajectory_.joint_trajectory.points;

      if (!points_after.empty()) {
        double scaled_duration = durationToSec(points_after.back().time_from_start);
        RCLCPP_INFO(
          node->get_logger(),
          "Scaled pose trajectory duration: %.3f seconds",
          scaled_duration
        );
      }

      RCLCPP_INFO(node->get_logger(), "Pose planning succeeded. Executing: %s", name.c_str());

      auto execute_result = move_group.execute(plan);

      move_group.clearPoseTargets();

      if (!static_cast<bool>(execute_result)) {
        RCLCPP_ERROR(node->get_logger(), "Pose execution failed: %s", name.c_str());
        return false;
      }

      RCLCPP_INFO(node->get_logger(), "Pose execution succeeded: %s", name.c_str());
      return true;
    };

  // ===============================
  // 1. 先进入 ready 姿态
  // ===============================
  std::map<std::string, double> ready_pose = {
    {"joint1", 0.0},
    {"joint2", -0.5},
    {"joint3", 0.8},
    {"joint4", 0.0},
    {"joint5", 0.7},
    {"joint6", 0.0},
    {"joint7", 0.0},
  };

  std::map<std::string, double> home_pose = {
    {"joint1", 0.0},
    {"joint2", 0.0},
    {"joint3", 0.0},
    {"joint4", 0.0},
    {"joint5", 0.0},
    {"joint6", 0.0},
    {"joint7", 0.0},
  };

  if (!plan_and_execute_joint(ready_pose, "ready")) {
    RCLCPP_ERROR(node->get_logger(), "ready failed, stop demo.");
    return shutdown_and_exit(1);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // ===============================
  // 2. 末端跨过 box_obstacle
  //
  // box_obstacle 大概位置：
  // x = 0.45, y = 0.0, z = 0.25
  // box 高度 0.40，顶部约 z = 0.45
  //
  // 这里 after_box 的 x 设置到 0.85，
  // 让末端明显到盒子后方。
  // ===============================

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

  // 盒子前方
  geometry_msgs::msg::Pose pre_over_box = current_pose;
  pre_over_box.position.x = 0.30;
  pre_over_box.position.y = -0.20;
  pre_over_box.position.z = 0.72;

  // 盒子正上方，高度更高，保证跨过盒子
  geometry_msgs::msg::Pose over_box = current_pose;
  over_box.position.x = 0.45;
  over_box.position.y = 0.00;
  over_box.position.z = 0.90;

  // 盒子后方高点，先跨过去
  geometry_msgs::msg::Pose after_box_high = current_pose;
  after_box_high.position.x = 0.85;
  after_box_high.position.y = 0.20;
  after_box_high.position.z = 0.78;

  // 盒子后方低点，下降一点，显示“到盒子后面”
  geometry_msgs::msg::Pose after_box_low = current_pose;
  after_box_low.position.x = 0.85;
  after_box_low.position.y = 0.20;
  after_box_low.position.z = 0.62;

  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> pose_sequence = {
    {"pre_over_box", pre_over_box},
    {"over_box", over_box},
    {"after_box_high", after_box_high},
    {"after_box_low", after_box_low},
  };

  for (const auto & target : pose_sequence) {
    const std::string & name = target.first;
    const auto & pose = target.second;

    bool success = plan_and_execute_pose(pose, name);

    if (!success) {
      RCLCPP_ERROR(node->get_logger(), "%s failed, stop pose demo.", name.c_str());
      return shutdown_and_exit(1);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // ===============================
  // 3. 回到 home
  // ===============================
  if (!plan_and_execute_joint(home_pose, "home")) {
    RCLCPP_ERROR(node->get_logger(), "home failed, stop demo.");
    return shutdown_and_exit(1);
  }

  RCLCPP_INFO(node->get_logger(), "Reach target obstacle crossing demo finished.");

  return shutdown_and_exit(0);
}