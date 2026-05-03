#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

using namespace std::chrono_literals;

class GripperCommander
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  explicit GripperCommander(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node_,
      "/gripper_controller/follow_joint_trajectory"
    );
  }

  bool send(
    double position,
    const std::string & name,
    double duration_sec = 1.0,
    bool allow_abort_as_success = false)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Sending gripper command [%s], position = %.3f",
      name.c_str(),
      position
    );

    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cannot find /gripper_controller/follow_joint_trajectory action server."
      );
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {
      "gripper_left_finger_joint",
      "gripper_right_finger_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position, position};
    point.time_from_start.sec = static_cast<int32_t>(duration_sec);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((duration_sec - static_cast<int32_t>(duration_sec)) * 1e9);

    goal.trajectory.points.push_back(point);

    auto goal_handle_future = client_->async_send_goal(goal);

    if (goal_handle_future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Timeout while sending gripper goal.");
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected.");
      return false;
    }

    auto result_future = client_->async_get_result(goal_handle);

    if (result_future.wait_for(20s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Timeout while waiting for gripper result.");
      return false;
    }

    auto result = result_future.get();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Gripper action [%s] did not succeed. result_code=%d",
        name.c_str(),
        static_cast<int>(result.code)
      );

      if (allow_abort_as_success) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Treat gripper action [%s] as acceptable and continue.",
          name.c_str()
        );
        return true;
      }

      return false;
    }

    if (result.result->error_code != 0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Gripper controller [%s] returned error_code=%d, error_string=%s",
        name.c_str(),
        result.result->error_code,
        result.result->error_string.c_str()
      );

      if (allow_abort_as_success) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Treat gripper controller error as acceptable and continue."
        );
        return true;
      }

      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Gripper command [%s] succeeded.", name.c_str());
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
};

static geometry_msgs::msg::Quaternion normalize_quaternion(
  const geometry_msgs::msg::Quaternion & q_in)
{
  geometry_msgs::msg::Quaternion q = q_in;

  const double norm = std::sqrt(
    q.x * q.x +
    q.y * q.y +
    q.z * q.z +
    q.w * q.w
  );

  if (norm < 1e-12 || !std::isfinite(norm)) {
    q.x = 1.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 0.0;
    return q;
  }

  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;

  return q;
}

static double quaternion_angular_distance(
  const geometry_msgs::msg::Quaternion & q1_in,
  const geometry_msgs::msg::Quaternion & q2_in)
{
  const auto q1 = normalize_quaternion(q1_in);
  const auto q2 = normalize_quaternion(q2_in);

  double dot =
    q1.x * q2.x +
    q1.y * q2.y +
    q1.z * q2.z +
    q1.w * q2.w;

  dot = std::abs(dot);
  dot = std::clamp(dot, 0.0, 1.0);

  return 2.0 * std::acos(dot);
}

static geometry_msgs::msg::Quaternion make_down_orientation(double yaw_rad)
{
  geometry_msgs::msg::Quaternion q;

  q.x = std::cos(yaw_rad * 0.5);
  q.y = std::sin(yaw_rad * 0.5);
  q.z = 0.0;
  q.w = 0.0;

  return normalize_quaternion(q);
}

static geometry_msgs::msg::Pose make_pose(
  double x,
  double y,
  double z,
  const geometry_msgs::msg::Quaternion & orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = normalize_quaternion(orientation);
  return pose;
}

static double get_trajectory_duration(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }

  const auto & last_point = trajectory.points.back();

  return static_cast<double>(last_point.time_from_start.sec) +
         static_cast<double>(last_point.time_from_start.nanosec) * 1e-9;
}

static void scale_joint_trajectory_time(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  double scale)
{
  if (scale <= 0.0) {
    return;
  }

  for (auto & point : trajectory.points) {
    const double old_time =
      static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1e-9;

    const double new_time = old_time * scale;

    point.time_from_start.sec = static_cast<int32_t>(std::floor(new_time));
    point.time_from_start.nanosec =
      static_cast<uint32_t>((new_time - std::floor(new_time)) * 1e9);

    for (auto & v : point.velocities) {
      v /= scale;
    }

    for (auto & a : point.accelerations) {
      a /= (scale * scale);
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  auto stop_and_exit = [&]() {
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
  };

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  GripperCommander gripper(node);

  move_group.setPlanningTime(15.0);
  move_group.setNumPlanningAttempts(30);

  move_group.setMaxVelocityScalingFactor(0.4);
  move_group.setMaxAccelerationScalingFactor(0.35);

  move_group.setGoalPositionTolerance(0.03);
  move_group.setGoalOrientationTolerance(0.80);

  const double TRAJECTORY_TIME_SCALE = 1.0;

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(node->get_logger(), "Planning group: %s", PLANNING_GROUP.c_str());

  auto plan_and_execute_joint =
    [&](const std::map<std::string, double> & target, const std::string & name) -> bool
    {
      RCLCPP_INFO(node->get_logger(), "Planning to joint target: %s", name.c_str());

      move_group.setStartStateToCurrentState();
      move_group.clearPoseTargets();
      move_group.setJointValueTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group.plan(plan));

      if (!success) {
        RCLCPP_ERROR(node->get_logger(), "Joint planning failed: %s", name.c_str());
        return false;
      }

      const double original_duration =
        get_trajectory_duration(plan.trajectory_.joint_trajectory);

      scale_joint_trajectory_time(plan.trajectory_.joint_trajectory, TRAJECTORY_TIME_SCALE);

      const double scaled_duration =
        get_trajectory_duration(plan.trajectory_.joint_trajectory);

      RCLCPP_INFO(
        node->get_logger(),
        "Joint planning succeeded: %s. Duration %.3f -> %.3f sec",
        name.c_str(),
        original_duration,
        scaled_duration
      );

      auto result = move_group.execute(plan);

      if (!static_cast<bool>(result)) {
        RCLCPP_ERROR(node->get_logger(), "Joint execution failed: %s", name.c_str());
        return false;
      }

      RCLCPP_INFO(node->get_logger(), "Joint execution succeeded: %s", name.c_str());
      return true;
    };

  auto plan_and_execute_pose =
    [&](const geometry_msgs::msg::Pose & target_pose, const std::string & name) -> bool
    {
      RCLCPP_INFO(node->get_logger(), "Planning to pose target: %s", name.c_str());

      move_group.setStartStateToCurrentState();
      move_group.clearPoseTargets();
      move_group.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group.plan(plan));

      if (!success) {
        RCLCPP_ERROR(node->get_logger(), "Pose planning failed: %s", name.c_str());
        move_group.clearPoseTargets();
        return false;
      }

      const double original_duration =
        get_trajectory_duration(plan.trajectory_.joint_trajectory);

      scale_joint_trajectory_time(plan.trajectory_.joint_trajectory, TRAJECTORY_TIME_SCALE);

      const double scaled_duration =
        get_trajectory_duration(plan.trajectory_.joint_trajectory);

      RCLCPP_INFO(
        node->get_logger(),
        "Pose planning succeeded: %s. Duration %.3f -> %.3f sec",
        name.c_str(),
        original_duration,
        scaled_duration
      );

      auto result = move_group.execute(plan);
      move_group.clearPoseTargets();

      if (!static_cast<bool>(result)) {
        RCLCPP_ERROR(node->get_logger(), "Pose execution failed: %s", name.c_str());
        return false;
      }

      RCLCPP_INFO(node->get_logger(), "Pose execution succeeded: %s", name.c_str());
      return true;
    };

  auto execute_cartesian_path =
    [&](const std::vector<geometry_msgs::msg::Pose> & waypoints,
        const std::string & name,
        double min_fraction = 0.80) -> bool
    {
      RCLCPP_INFO(node->get_logger(), "Computing Cartesian path: %s", name.c_str());

      move_group.setStartStateToCurrentState();

      moveit_msgs::msg::RobotTrajectory trajectory;

      const double eef_step = 0.005;
      const double jump_threshold = 0.0;

      double fraction = move_group.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory,
        true
      );

      RCLCPP_INFO(
        node->get_logger(),
        "Cartesian path [%s] fraction: %.2f",
        name.c_str(),
        fraction
      );

      if (fraction < min_fraction) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Cartesian path failed: %s, fraction = %.2f",
          name.c_str(),
          fraction
        );
        return false;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      const double original_duration =
        get_trajectory_duration(plan.trajectory_.joint_trajectory);

      scale_joint_trajectory_time(plan.trajectory_.joint_trajectory, TRAJECTORY_TIME_SCALE);

      const double scaled_duration =
        get_trajectory_duration(plan.trajectory_.joint_trajectory);

      RCLCPP_INFO(
        node->get_logger(),
        "Cartesian path ready: %s. Duration %.3f -> %.3f sec",
        name.c_str(),
        original_duration,
        scaled_duration
      );

      auto result = move_group.execute(plan);

      if (!static_cast<bool>(result)) {
        RCLCPP_ERROR(node->get_logger(), "Cartesian execution failed: %s", name.c_str());
        return false;
      }

      RCLCPP_INFO(node->get_logger(), "Cartesian execution succeeded: %s", name.c_str());
      return true;
    };

  auto enable_orientation_constraint =
    [&](const geometry_msgs::msg::Quaternion & q,
        double tolerance,
        const std::string & name)
    {
      const std::string eef_link = move_group.getEndEffectorLink();

      if (eef_link.empty()) {
        RCLCPP_WARN(
          node->get_logger(),
          "End effector link is empty. Skip orientation constraint [%s].",
          name.c_str()
        );
        return;
      }

      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.header.frame_id = move_group.getPlanningFrame();
      ocm.link_name = eef_link;
      ocm.orientation = normalize_quaternion(q);

      ocm.absolute_x_axis_tolerance = tolerance;
      ocm.absolute_y_axis_tolerance = tolerance;
      ocm.absolute_z_axis_tolerance = tolerance;
      ocm.weight = 1.0;

      moveit_msgs::msg::Constraints constraints;
      constraints.orientation_constraints.push_back(ocm);

      move_group.setPathConstraints(constraints);

      RCLCPP_INFO(
        node->get_logger(),
        "Orientation constraint [%s] enabled on link [%s], tolerance = %.3f rad.",
        name.c_str(),
        eef_link.c_str(),
        tolerance
      );
    };

  auto clear_orientation_constraint =
    [&]()
    {
      move_group.clearPathConstraints();
      RCLCPP_INFO(node->get_logger(), "Orientation constraint cleared.");
    };

  auto verify_current_orientation =
    [&](const geometry_msgs::msg::Quaternion & target_q,
        const std::string & name,
        double max_error_rad) -> bool
    {
      auto current_pose_stamped = move_group.getCurrentPose();

      const double error =
        quaternion_angular_distance(current_pose_stamped.pose.orientation, target_q);

      RCLCPP_INFO(
        node->get_logger(),
        "Orientation check [%s]: error = %.4f rad, max = %.4f rad",
        name.c_str(),
        error,
        max_error_rad
      );

      if (error > max_error_rad) {
        RCLCPP_WARN(
          node->get_logger(),
          "Orientation check warning [%s]. error = %.4f rad.",
          name.c_str(),
          error
        );
        return false;
      }

      return true;
    };

  // ==================== 夹爪参数 ====================
  const double GRIPPER_OPEN = 0.050;
  const double GRIPPER_GRASP = 0.019;
  const double GRIPPER_CLOSE = 0.000;

  std::map<std::string, double> ready_pose = {
    {"joint1", 0.0}, {"joint2", -0.1}, {"joint3", 0.25},
    {"joint4", 0.0}, {"joint5", 0.2},  {"joint6", 0.0}, {"joint7", 0.0}
  };

  std::map<std::string, double> home_pose = {
    {"joint1", 0.0}, {"joint2", 0.0}, {"joint3", 0.0},
    {"joint4", 0.0}, {"joint5", 0.0}, {"joint6", 0.0}, {"joint7", 0.0}
  };

  const double target_x = 0.85;
  const double target_y = 0.70;

  const double place_center_x = 1.05;
  const double place_center_y = -0.70;

  const double PLACE_OFFSET_X = -0.05;
  const double PLACE_OFFSET_Y = 0.02;

  const double place_x = place_center_x + PLACE_OFFSET_X;
  const double place_y = place_center_y + PLACE_OFFSET_Y;

  const double high_z = 0.82;
  const double grasp_z = 0.50;
  const double carry_z = 0.82;

  const double place_high_z = 0.82;
  const double place_hover_z = 0.68;

  // 原来 0.54 偏高，木块可能松爪后掉下去然后歪。
  // 如果压桌面或穿模，改成 0.520 或 0.525。
  const double place_release_z = 0.515;

  const double retreat_after_place_z = 0.75;

  const double away_distance = 0.03;

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 1.0;
  orientation.y = 0.0;
  orientation.z = 0.0;
  orientation.w = 0.0;
  orientation = normalize_quaternion(orientation);

  auto pre_grasp = make_pose(target_x, target_y, high_z, orientation);
  auto grasp_pose = make_pose(target_x, target_y, grasp_z, orientation);
  auto lift_vertical_pose = make_pose(target_x, target_y, carry_z, orientation);

  const double target_radius = std::sqrt(target_x * target_x + target_y * target_y);
  const double away_dir_x = (target_radius > 1e-6) ? target_x / target_radius : 1.0;
  const double away_dir_y = (target_radius > 1e-6) ? target_y / target_radius : 0.0;

  const double high_away_x = target_x + away_distance * away_dir_x;
  const double high_away_y = target_y + away_distance * away_dir_y;

  auto high_away_pose = make_pose(high_away_x, high_away_y, carry_z, orientation);

  // ==================== 流程 ====================
  if (!gripper.send(GRIPPER_CLOSE, "close_before_approach", 1.0, true)) {
    stop_and_exit();
    return 1;
  }

  const bool RUN_READY = false;

  if (RUN_READY) {
    if (!plan_and_execute_joint(ready_pose, "ready")) {
      stop_and_exit();
      return 1;
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Skip ready joint motion. Start from current state.");
    move_group.setStartStateToCurrentState();
  }

  std::this_thread::sleep_for(300ms);

  // 1. 木块上方
  if (!plan_and_execute_pose(pre_grasp, "pre_grasp")) {
    stop_and_exit();
    return 1;
  }

  // 2. 打开夹爪
  if (!gripper.send(GRIPPER_OPEN, "open_before_descend", 1.5, true)) {
    stop_and_exit();
    return 1;
  }

  std::this_thread::sleep_for(500ms);

  // 3. 垂直下降抓取
  if (!execute_cartesian_path({grasp_pose}, "descend_to_grasp", 0.80)) {
    stop_and_exit();
    return 1;
  }

  std::this_thread::sleep_for(800ms);

  // 4. 闭合夹爪
  if (!gripper.send(GRIPPER_GRASP, "grasp_object", 2.0, true)) {
    stop_and_exit();
    return 1;
  }

  std::this_thread::sleep_for(500ms);

  // 抓取后降速
  move_group.setMaxVelocityScalingFactor(0.25);
  move_group.setMaxAccelerationScalingFactor(0.20);
  clear_orientation_constraint();

  // 5. 竖直抬起
  if (!execute_cartesian_path({lift_vertical_pose}, "lift_vertical_after_grasp", 0.60)) {
    RCLCPP_WARN(node->get_logger(), "Cartesian vertical lift failed. Try normal pose planning.");

    if (!plan_and_execute_pose(lift_vertical_pose, "lift_vertical_after_grasp_pose")) {
      stop_and_exit();
      return 1;
    }
  }

  // 6. 轻微远离
  if (!plan_and_execute_pose(high_away_pose, "move_slightly_away_after_lift")) {
    RCLCPP_WARN(node->get_logger(), "Move slightly away failed. Skip this step and continue.");
  }

  // ============================================================
  // 7. 搬运到放置区上方
  // ============================================================
  auto current_after_lift_stamped = move_group.getCurrentPose();

  geometry_msgs::msg::Quaternion locked_carry_orientation =
    normalize_quaternion(current_after_lift_stamped.pose.orientation);

  geometry_msgs::msg::Pose place_transfer_high_pose =
    make_pose(place_x, place_y, place_high_z, locked_carry_orientation);

  move_group.setPlanningTime(15.0);
  move_group.setNumPlanningAttempts(30);
  move_group.setMaxVelocityScalingFactor(0.22);
  move_group.setMaxAccelerationScalingFactor(0.16);

  move_group.setGoalPositionTolerance(0.035);
  move_group.setGoalOrientationTolerance(0.35);

  bool transfer_success = false;

  // 第一次：中等姿态约束搬运
  enable_orientation_constraint(locked_carry_orientation, 0.35, "carry_orientation_medium");

  if (plan_and_execute_pose(place_transfer_high_pose, "move_to_place_high_keep_carry_orientation")) {
    transfer_success = true;
  } else {
    clear_orientation_constraint();

    RCLCPP_WARN(
      node->get_logger(),
      "Keep-carry transfer failed. Retry with looser orientation tolerance."
    );

    move_group.setGoalOrientationTolerance(0.55);
    enable_orientation_constraint(locked_carry_orientation, 0.55, "carry_orientation_loose");

    if (plan_and_execute_pose(place_transfer_high_pose, "move_to_place_high_keep_carry_orientation_loose")) {
      transfer_success = true;
    } else {
      clear_orientation_constraint();

      RCLCPP_WARN(
        node->get_logger(),
        "Loose keep-carry transfer failed. Try a higher place pose with loose goal."
      );

      geometry_msgs::msg::Pose place_transfer_higher_pose = place_transfer_high_pose;
      place_transfer_higher_pose.position.z = place_high_z + 0.10;

      move_group.setGoalOrientationTolerance(0.75);

      if (plan_and_execute_pose(place_transfer_higher_pose, "move_to_place_high_keep_carry_orientation_higher")) {
        transfer_success = true;
      }
    }
  }

  clear_orientation_constraint();

  if (!transfer_success) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move to place area.");
    stop_and_exit();
    return 1;
  }

  // ============================================================
  // 8. 放置区上方姿态处理
  // 重点修改：
  // 之前 error > 0.65 就直接退出。
  // 现在：如果姿态漂移较大，先尝试在当前位置软对齐。
  // 如果软对齐失败，不再退出，而是使用当前实际姿态下降。
  // ============================================================
  auto current_place_top_stamped = move_group.getCurrentPose();

  geometry_msgs::msg::Pose selected_place_high_pose = current_place_top_stamped.pose;
  selected_place_high_pose.position.x = place_x;
  selected_place_high_pose.position.y = place_y;
  selected_place_high_pose.orientation =
    normalize_quaternion(current_place_top_stamped.pose.orientation);

  geometry_msgs::msg::Quaternion selected_place_orientation =
    selected_place_high_pose.orientation;

  double carry_error =
    quaternion_angular_distance(selected_place_orientation, locked_carry_orientation);

  RCLCPP_INFO(
    node->get_logger(),
    "Place orientation compared with locked carry orientation: error = %.4f rad",
    carry_error
  );

  // 如果漂移比较大，尝试在放置区上方重新对齐到抓取后的姿态。
  if (carry_error > 0.45) {
    RCLCPP_WARN(
      node->get_logger(),
      "Place orientation drift is large. Try soft realign at place top."
    );

    geometry_msgs::msg::Pose realign_pose = selected_place_high_pose;
    realign_pose.orientation = locked_carry_orientation;

    move_group.setPlanningTime(8.0);
    move_group.setNumPlanningAttempts(20);
    move_group.setMaxVelocityScalingFactor(0.12);
    move_group.setMaxAccelerationScalingFactor(0.08);
    move_group.setGoalPositionTolerance(0.025);
    move_group.setGoalOrientationTolerance(0.35);

    enable_orientation_constraint(locked_carry_orientation, 0.45, "soft_realign_to_carry_orientation");

    if (plan_and_execute_pose(realign_pose, "soft_realign_to_carry_orientation_at_place")) {
      clear_orientation_constraint();

      auto after_realign_stamped = move_group.getCurrentPose();
      selected_place_high_pose = after_realign_stamped.pose;
      selected_place_high_pose.position.x = place_x;
      selected_place_high_pose.position.y = place_y;
      selected_place_high_pose.orientation =
        normalize_quaternion(after_realign_stamped.pose.orientation);

      selected_place_orientation = selected_place_high_pose.orientation;

      carry_error =
        quaternion_angular_distance(selected_place_orientation, locked_carry_orientation);

      RCLCPP_INFO(
        node->get_logger(),
        "Soft realign succeeded. New carry orientation error = %.4f rad",
        carry_error
      );
    } else {
      clear_orientation_constraint();

      RCLCPP_WARN(
        node->get_logger(),
        "Soft realign failed. Continue with current actual orientation to avoid another planning failure."
      );

      auto after_failed_realign_stamped = move_group.getCurrentPose();
      selected_place_high_pose = after_failed_realign_stamped.pose;
      selected_place_high_pose.position.x = place_x;
      selected_place_high_pose.position.y = place_y;
      selected_place_high_pose.orientation =
        normalize_quaternion(after_failed_realign_stamped.pose.orientation);

      selected_place_orientation = selected_place_high_pose.orientation;
    }
  }

  verify_current_orientation(
    selected_place_orientation,
    "selected_place_orientation_before_descend",
    0.20
  );

  // ============================================================
  // 9. 保持当前选定姿态，垂直下降放置
  // ============================================================
  geometry_msgs::msg::Pose place_hover_pose = selected_place_high_pose;
  place_hover_pose.position.z = place_hover_z;
  place_hover_pose.orientation = selected_place_orientation;

  geometry_msgs::msg::Pose place_release_pose = selected_place_high_pose;
  place_release_pose.position.z = place_release_z;
  place_release_pose.orientation = selected_place_orientation;

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(20);
  move_group.setMaxVelocityScalingFactor(0.16);
  move_group.setMaxAccelerationScalingFactor(0.10);
  move_group.setGoalPositionTolerance(0.02);
  move_group.setGoalOrientationTolerance(0.25);

  // 下降时保持当前实际姿态，避免末端在下降过程中继续扭木块。
  enable_orientation_constraint(
    selected_place_orientation,
    0.25,
    "place_descend_keep_selected_orientation"
  );

  if (!execute_cartesian_path({place_hover_pose}, "vertical_descend_to_place_hover_keep_orientation", 0.70)) {
    RCLCPP_WARN(
      node->get_logger(),
      "Vertical descend to place_hover failed. Retry with lower fraction requirement."
    );

    if (!execute_cartesian_path({place_hover_pose}, "vertical_descend_to_place_hover_retry", 0.45)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Vertical descend to place_hover failed after retry."
      );

      clear_orientation_constraint();
      stop_and_exit();
      return 1;
    }
  }

  if (!execute_cartesian_path({place_release_pose}, "vertical_descend_to_place_release_keep_orientation", 0.70)) {
    RCLCPP_WARN(
      node->get_logger(),
      "Vertical descend to place_release failed. Retry with lower fraction requirement."
    );

    if (!execute_cartesian_path({place_release_pose}, "vertical_descend_to_place_release_retry", 0.45)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Vertical descend to place_release failed after retry."
      );

      clear_orientation_constraint();
      stop_and_exit();
      return 1;
    }
  }

  std::this_thread::sleep_for(800ms);

  // 10. 慢速松爪
  if (!gripper.send(GRIPPER_OPEN, "release_object_slow", 2.5, true)) {
    clear_orientation_constraint();
    stop_and_exit();
    return 1;
  }

  std::this_thread::sleep_for(700ms);

  // 11. 保持同一姿态竖直抬起，避免夹爪横向刮到木块
  geometry_msgs::msg::Pose retreat_after_place_pose = place_release_pose;
  retreat_after_place_pose.position.z = retreat_after_place_z;
  retreat_after_place_pose.orientation = selected_place_orientation;

  if (!execute_cartesian_path({retreat_after_place_pose}, "vertical_retreat_after_place_keep_orientation", 0.60)) {
    RCLCPP_WARN(
      node->get_logger(),
      "Cartesian retreat_after_place failed. Skip retreat and go home."
    );
  }

  clear_orientation_constraint();

  // 12. 恢复速度参数
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(20);
  move_group.setGoalPositionTolerance(0.03);
  move_group.setGoalOrientationTolerance(0.80);
  move_group.setMaxVelocityScalingFactor(0.4);
  move_group.setMaxAccelerationScalingFactor(0.35);

  // 13. 回家
  if (!plan_and_execute_joint(home_pose, "home")) {
    stop_and_exit();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Pick-place flow demo finished.");
  stop_and_exit();
  return 0;
}