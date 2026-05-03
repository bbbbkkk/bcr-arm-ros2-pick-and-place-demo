// #include <chrono>
// #include <cstdint>
// #include <map>
// #include <memory>
// #include <string>
// #include <thread>
// #include <utility>
// #include <vector>

// #include <builtin_interfaces/msg/duration.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// double durationToSec(const builtin_interfaces::msg::Duration & duration)
// {
//   return static_cast<double>(duration.sec) +
//          static_cast<double>(duration.nanosec) * 1e-9;
// }

// builtin_interfaces::msg::Duration secToDuration(double seconds)
// {
//   builtin_interfaces::msg::Duration duration;
//   duration.sec = static_cast<int32_t>(seconds);
//   duration.nanosec = static_cast<uint32_t>(
//     (seconds - static_cast<double>(duration.sec)) * 1e9
//   );
//   return duration;
// }

// void slowDownTrajectory(
//   moveit::planning_interface::MoveGroupInterface::Plan & plan,
//   double scale)
// {
//   if (scale <= 0.0) return;

//   auto & points = plan.trajectory_.joint_trajectory.points;
//   for (auto & point : points) {
//     double old_time = durationToSec(point.time_from_start);
//     double new_time = old_time * scale;
//     point.time_from_start = secToDuration(new_time);
//     for (auto & velocity : point.velocities) velocity = velocity / scale;
//     for (auto & acceleration : point.accelerations) acceleration = acceleration / (scale * scale);
//   }
// }

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<rclcpp::Node>(
//     "reach_target_demo",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread spinner([&executor]() { executor.spin(); });

//   static const std::string PLANNING_GROUP = "arm";
//   moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

//   // 速度调快
//   move_group.setPlanningTime(5.0);
//   move_group.setNumPlanningAttempts(5);
//   move_group.setMaxVelocityScalingFactor(0.8);      // 快速
//   move_group.setMaxAccelerationScalingFactor(0.6);
//   move_group.setGoalPositionTolerance(0.01);
//   move_group.setGoalOrientationTolerance(0.1);
//   move_group.startStateMonitor(5.0);

//   RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   auto shutdown_and_exit = [&](int code) -> int {
//     executor.cancel();
//     if (spinner.joinable()) spinner.join();
//     rclcpp::shutdown();
//     return code;
//   };

//   auto plan_and_execute_pose =
//     [&](const geometry_msgs::msg::Pose & target_pose, const std::string & name) -> bool {
//       RCLCPP_INFO(node->get_logger(), "Moving to: %s [%.2f, %.2f, %.2f]", 
//         name.c_str(), target_pose.position.x, target_pose.position.y, target_pose.position.z);
      
//       move_group.setStartStateToCurrentState();
//       move_group.setPoseReferenceFrame(move_group.getPlanningFrame());
//       move_group.setPoseTarget(target_pose, move_group.getEndEffectorLink());

//       moveit::planning_interface::MoveGroupInterface::Plan plan;
//       if (!static_cast<bool>(move_group.plan(plan))) {
//         RCLCPP_ERROR(node->get_logger(), "Planning failed: %s", name.c_str());
//         move_group.clearPoseTargets();
//         return false;
//       }

//       // 正常速度，不放慢
//       if (!static_cast<bool>(move_group.execute(plan))) {
//         RCLCPP_ERROR(node->get_logger(), "Execution failed: %s", name.c_str());
//         move_group.clearPoseTargets();
//         return false;
//       }

//       move_group.clearPoseTargets();
//       RCLCPP_INFO(node->get_logger(), "✓ Reached: %s", name.c_str());
//       return true;
//     };

//   // 夹爪朝下姿态
//   auto orient_down = []() {
//     geometry_msgs::msg::Quaternion q;
//     q.x = 0.0;
//     q.y = 0.70710678;
//     q.z = 0.0;
//     q.w = 0.70710678;
//     return q;
//   }();

//   // ====================================================
//   // 场景坐标：
//   //   木块: (0.70, 0.50, 0.10), 顶部z=0.15
//   //   障碍物: (0.80, 0.00, 0.225), 顶部z=0.40, Y=[-0.275, 0.275]
//   //   放置区: (0.90, -0.50, 0.055), 顶部z=0.06
//   // ====================================================

//   // ===== 步骤1: 飞到木块正上方（停顿点1） =====
//   RCLCPP_INFO(node->get_logger(), "\n===== MOVING ABOVE BLOCK =====");

//   geometry_msgs::msg::Pose above_block;
//   above_block.position.x = 0.70;
//   above_block.position.y = 0.50;
//   above_block.position.z = 0.50;  // 木块上方安全高度
//   above_block.orientation = orient_down;

//   if (!plan_and_execute_pose(above_block, "ABOVE_BLOCK")) {
//     return shutdown_and_exit(1);
//   }

//   RCLCPP_INFO(node->get_logger(), ">>> PAUSED ABOVE BLOCK <<<");
//   std::this_thread::sleep_for(std::chrono::milliseconds(1500));

//   // ===== 步骤2: 绕过障碍物飞到放置区正上方（停顿点2） =====
//   RCLCPP_INFO(node->get_logger(), "\n===== MOVING ABOVE PLACE AREA =====");

//   geometry_msgs::msg::Pose above_place;
//   above_place.position.x = 0.90;
//   above_place.position.y = -0.50;
//   above_place.position.z = 0.45;  // 放置区上方安全高度
//   above_place.orientation = orient_down;

//   if (!plan_and_execute_pose(above_place, "ABOVE_PLACE")) {
//     return shutdown_and_exit(1);
//   }

//   RCLCPP_INFO(node->get_logger(), ">>> PAUSED ABOVE PLACE AREA <<<");
//   std::this_thread::sleep_for(std::chrono::milliseconds(1500));

//   RCLCPP_INFO(node->get_logger(), "\n===== DEMO COMPLETED =====");
//   return shutdown_and_exit(0);
// }

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "reach_target_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // 速度调快
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(8);
  move_group.setMaxVelocityScalingFactor(0.6);
  move_group.setMaxAccelerationScalingFactor(0.4);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.1);
  move_group.startStateMonitor(5.0);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // 打印当前末端位置，方便调试
  auto current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Current tool0 pose: [%.3f, %.3f, %.3f]",
    current_pose.position.x, current_pose.position.y, current_pose.position.z);

  auto shutdown_and_exit = [&](int code) -> int {
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return code;
  };

  auto plan_and_execute_pose =
    [&](const geometry_msgs::msg::Pose & target_pose, const std::string & name) -> bool {
      RCLCPP_INFO(node->get_logger(), "Moving to: %s [%.3f, %.3f, %.3f]", 
        name.c_str(), target_pose.position.x, target_pose.position.y, target_pose.position.z);
      
      move_group.setStartStateToCurrentState();
      move_group.setPoseReferenceFrame(move_group.getPlanningFrame());
      move_group.setPoseTarget(target_pose, move_group.getEndEffectorLink());

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (!static_cast<bool>(move_group.plan(plan))) {
        RCLCPP_ERROR(node->get_logger(), "Planning failed: %s", name.c_str());
        move_group.clearPoseTargets();
        return false;
      }

      if (!static_cast<bool>(move_group.execute(plan))) {
        RCLCPP_ERROR(node->get_logger(), "Execution failed: %s", name.c_str());
        move_group.clearPoseTargets();
        return false;
      }

      move_group.clearPoseTargets();
      RCLCPP_INFO(node->get_logger(), "✓ Reached: %s", name.c_str());
      return true;
    };

  // 根据你的URDF: world_to_base joint将base_link固定在(0, 0, 0)
  // 机械臂base_link高度从0到0.10m（圆柱半径0.18m）
  // 
  // 场景物体（world坐标系）：
  //   桌面顶z = 0.05m (pose.z = 0.025 + 半高0.025)
  //   木块中心(0.70, 0.50, 0.10)，顶部z = 0.15m (0.10 + 0.05半高)
  //   障碍物中心(0.80, 0.00, 0.225)，顶部z = 0.40m
  //   放置区中心(0.90, -0.50, 0.055)，顶部z = 0.06m

  auto create_gripper_down = []() {
    geometry_msgs::msg::Quaternion q;
    // 夹爪开口朝下（Z轴向下）
    q.x = 0.0;
    q.y = 0.70710678;   // 绕Y轴转180度
    q.z = 0.0;
    q.w = 0.70710678;
    return q;
  };

  auto gripper_down = create_gripper_down();

  // ===== 步骤1: 飞到木块正上方 =====
  RCLCPP_INFO(node->get_logger(), "\n===== STEP 1: MOVING ABOVE BLOCK =====");

  geometry_msgs::msg::Pose above_block;
  above_block.position.x = 0.70;      // 木块X中心
  above_block.position.y = 0.50;      // 木块Y中心
  above_block.position.z = 0.40;      // 木块上方25cm（木块顶0.15 + 0.25）
  above_block.orientation = gripper_down;

  RCLCPP_INFO(node->get_logger(), "Target ABOVE BLOCK: x=%.3f, y=%.3f, z=%.3f", 
    above_block.position.x, above_block.position.y, above_block.position.z);

  if (!plan_and_execute_pose(above_block, "ABOVE_BLOCK")) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move above block");
    return shutdown_and_exit(1);
  }

  // 验证当前位置
  current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Actual tool0 pose: [%.3f, %.3f, %.3f]",
    current_pose.position.x, current_pose.position.y, current_pose.position.z);
  
  RCLCPP_INFO(node->get_logger(), ">>> PAUSED ABOVE BLOCK (1.5s) <<<");
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // ===== 步骤2: 抬起绕障碍物 =====
  RCLCPP_INFO(node->get_logger(), "\n===== STEP 2: MOVING ABOVE PLACE AREA =====");

  geometry_msgs::msg::Pose above_place;
  above_place.position.x = 0.90;      // 放置区X中心
  above_place.position.y = -0.50;     // 放置区Y中心
  above_place.position.z = 0.40;      // 放置区上方34cm（放置区顶0.06 + 0.34）
  above_place.orientation = gripper_down;

  RCLCPP_INFO(node->get_logger(), "Target ABOVE PLACE: x=%.3f, y=%.3f, z=%.3f",
    above_place.position.x, above_place.position.y, above_place.position.z);

  if (!plan_and_execute_pose(above_place, "ABOVE_PLACE")) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move above place");
    return shutdown_and_exit(1);
  }

  // 验证当前位置
  current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Actual tool0 pose: [%.3f, %.3f, %.3f]",
    current_pose.position.x, current_pose.position.y, current_pose.position.z);
  
  RCLCPP_INFO(node->get_logger(), ">>> PAUSED ABOVE PLACE (1.5s) <<<");
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  RCLCPP_INFO(node->get_logger(), "\n===== DEMO COMPLETED =====");
  return shutdown_and_exit(0);
}