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

    // 给夹爪 action 加宽容差。
    // 物理仿真下夹爪可能因为摩擦、接触、控制误差，不能精确到达目标值。
    control_msgs::msg::JointTolerance left_goal_tolerance;
    left_goal_tolerance.name = "gripper_left_finger_joint";
    left_goal_tolerance.position = 0.015;
    left_goal_tolerance.velocity = 0.0;
    left_goal_tolerance.acceleration = 0.0;

    control_msgs::msg::JointTolerance right_goal_tolerance;
    right_goal_tolerance.name = "gripper_right_finger_joint";
    right_goal_tolerance.position = 0.015;
    right_goal_tolerance.velocity = 0.0;
    right_goal_tolerance.acceleration = 0.0;

    goal.goal_tolerance.clear();
    goal.goal_tolerance.push_back(left_goal_tolerance);
    goal.goal_tolerance.push_back(right_goal_tolerance);

    // 重点：这里不要只给 8 秒。
    // 你的日志已经明确显示 goal_time_tolerance 超了 8.01 秒。
    goal.goal_time_tolerance.sec = 60;
    goal.goal_time_tolerance.nanosec = 0;

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

    // Gazebo 物理仿真可能比真实时间慢很多。
    // 这里不要固定 20s，否则仿真时间 5s 的夹爪动作可能还没返回结果就被程序判定超时。
    // 给夹爪动作更长的 wall-clock 等待时间。
    const int result_timeout_sec =
      static_cast<int>(std::ceil(std::max(60.0, duration_sec * 8.0 + 20.0)));

    if (result_future.wait_for(std::chrono::seconds(result_timeout_sec)) !=
        std::future_status::ready)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Timeout while waiting for gripper result [%s]. waited=%d sec",
        name.c_str(),
        result_timeout_sec
      );

      client_->async_cancel_goal(goal_handle);
      return false;
    }

    auto result = result_future.get();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Gripper action [%s] did not succeed. result_code=%d, controller_error_code=%d, error_string=%s",
      name.c_str(),
      static_cast<int>(result.code),
      result.result ? result.result->error_code : 999999,
      result.result ? result.result->error_string.c_str() : "null result"
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

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(30);

  move_group.setMaxVelocityScalingFactor(0.4);
  move_group.setMaxAccelerationScalingFactor(0.35);

  move_group.setGoalPositionTolerance(0.03);
  move_group.setGoalOrientationTolerance(0.45);

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
        double min_fraction = 0.80,
        bool avoid_collisions = true) -> bool
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
        avoid_collisions
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

  // ==================== 夹爪参数 ====================
  const double GRIPPER_OPEN = 0.035;
  const double GRIPPER_GRASP = 0.015;
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
  const double grasp_z = 0.48;
  const double carry_z = 0.82;

  const double place_high_z = 0.82;
  const double place_hover_z = 0.68;
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
  clear_orientation_constraint();
  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();

  if (!gripper.send(GRIPPER_OPEN, "open_before_approach", 5.0, false)) {
  RCLCPP_ERROR(
    node->get_logger(),
    "open_before_approach failed. Stop because gripper did not open correctly."
  );
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
  std::this_thread::sleep_for(800ms);

  // 2. 夹爪已经在 approach 前打开，这里不再重复打开，避免物理状态卡住导致 action abort。
  RCLCPP_INFO(
    node->get_logger(),
    "Gripper already opened before approach. Skip open_before_descend."
  );

  std::this_thread::sleep_for(500ms);
  // 3. 垂直下降抓取
  if (!execute_cartesian_path({grasp_pose}, "descend_to_grasp", 0.75)) {
    stop_and_exit();
    return 1;
  }

  // 调试用：暂停观察 Gazebo 中夹爪夹持垫是否真的到木块两侧
  std::this_thread::sleep_for(5s);

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
  // 物理抓取后，夹爪和木块处于真实接触状态。
  // 这里先用 Gazebo 物理接触为准，MoveIt 的第一段竖直抬起不做 collision checking，
  // 避免夹爪 collision 刚闭合后被 MoveIt 判定为碰撞而规划失败。
  {
    auto lift_start_stamped = move_group.getCurrentPose();

    geometry_msgs::msg::Pose lift_start_pose = lift_start_stamped.pose;
    lift_start_pose.orientation = normalize_quaternion(lift_start_pose.orientation);

    std::vector<geometry_msgs::msg::Pose> lift_waypoints;

    const double start_z = lift_start_pose.position.z;

    std::vector<double> lift_z_steps = {
      start_z + 0.04,
      start_z + 0.08,
      start_z + 0.14,
      start_z + 0.22,
      carry_z
    };

    for (double z : lift_z_steps) {
      if (z <= start_z + 0.005) {
        continue;
      }

      if (z > carry_z) {
        z = carry_z;
      }

      geometry_msgs::msg::Pose p = lift_start_pose;
      p.position.z = z;
      p.orientation = lift_start_pose.orientation;

      if (lift_waypoints.empty() ||
          std::abs(lift_waypoints.back().position.z - p.position.z) > 0.005)
      {
        lift_waypoints.push_back(p);
      }

      if (std::abs(z - carry_z) < 0.005) {
        break;
      }
    }

    move_group.setMaxVelocityScalingFactor(0.12);
    move_group.setMaxAccelerationScalingFactor(0.06);

    if (!execute_cartesian_path(
          lift_waypoints,
          "lift_vertical_after_grasp_physical",
          0.45,
          false))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Physical vertical lift failed. Stop here instead of forcing a tilted lift."
      );

      stop_and_exit();
      return 1;
    }
  }

  // 6. 轻微远离
  if (!plan_and_execute_pose(high_away_pose, "move_slightly_away_after_lift")) {
    RCLCPP_WARN(node->get_logger(), "Move slightly away failed. Skip this step and continue.");
  }

  // ============================================================
  // 7. 放置阶段：到放置区上方，并让夹爪接近竖直向下
  // ============================================================

  const double place_gripper_center_x = place_x;
  const double place_gripper_center_y = place_y;
  const double place_approach_z = place_high_z + 0.10;

  bool place_approach_success = false;
  geometry_msgs::msg::Quaternion selected_place_orientation;

  const double PLACE_YAW_BASE = 0.0;

  std::vector<double> place_yaw_candidates = {
    PLACE_YAW_BASE
  };

  move_group.setPlanningTime(15.0);
  move_group.setNumPlanningAttempts(30);
  move_group.setMaxVelocityScalingFactor(0.18);
  move_group.setMaxAccelerationScalingFactor(0.12);
  move_group.setGoalPositionTolerance(0.020);
  move_group.setGoalOrientationTolerance(0.25);

  for (double yaw : place_yaw_candidates) {
    geometry_msgs::msg::Quaternion place_down_orientation =
      make_down_orientation(yaw);

    geometry_msgs::msg::Pose place_approach_pose =
      make_pose(
        place_gripper_center_x,
        place_gripper_center_y,
        place_approach_z,
        place_down_orientation
      );

    RCLCPP_INFO(
      node->get_logger(),
      "Try vertical-down place approach yaw = %.3f",
      yaw
    );

    clear_orientation_constraint();

    if (plan_and_execute_pose(place_approach_pose, "move_to_place_high_vertical_down")) {
      auto after_approach_stamped = move_group.getCurrentPose();

      const double orientation_error =
        quaternion_angular_distance(
          normalize_quaternion(after_approach_stamped.pose.orientation),
          place_down_orientation
        );

      RCLCPP_INFO(
        node->get_logger(),
        "Vertical-down approach orientation error = %.4f rad",
        orientation_error
      );

      if (orientation_error <= 0.35) {
        selected_place_orientation =
          normalize_quaternion(after_approach_stamped.pose.orientation);

        place_approach_success = true;

        RCLCPP_INFO(
          node->get_logger(),
          "Selected vertical-down place yaw = %.3f",
          yaw
        );

        break;
      } else {
        RCLCPP_WARN(
          node->get_logger(),
          "Approach reached, but orientation is still too tilted. Try next yaw."
        );
      }
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "Vertical-down place approach yaw %.3f failed. Try next yaw.",
        yaw
      );
    }
  }

  if (!place_approach_success) {
    RCLCPP_WARN(
      node->get_logger(),
      "Failed to reach place area with vertical-down gripper. "
      "Fallback: move to place high with current reachable orientation."
    );

    clear_orientation_constraint();

    auto fallback_start_stamped = move_group.getCurrentPose();

    geometry_msgs::msg::Quaternion fallback_orientation =
      normalize_quaternion(fallback_start_stamped.pose.orientation);

    geometry_msgs::msg::Pose fallback_place_approach_pose =
      make_pose(
        place_gripper_center_x,
        place_gripper_center_y,
        place_approach_z,
        fallback_orientation
      );

    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.25);
    move_group.setMaxAccelerationScalingFactor(0.18);
    move_group.setGoalPositionTolerance(0.040);
    move_group.setGoalOrientationTolerance(1.00);

    if (!plan_and_execute_pose(
          fallback_place_approach_pose,
          "move_to_place_high_fallback_current_orientation"))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Fallback move to place high also failed."
      );

      stop_and_exit();
      return 1;
    }

    auto after_fallback_stamped = move_group.getCurrentPose();

    selected_place_orientation =
      normalize_quaternion(after_fallback_stamped.pose.orientation);

    place_approach_success = true;

    RCLCPP_WARN(
      node->get_logger(),
      "Fallback reached place high. Continue with current reachable orientation."
    );
  }

  // ============================================================
  // 8. 高处精确对准放置 x/y
  // ============================================================

  auto before_xy_settle_stamped = move_group.getCurrentPose();

  geometry_msgs::msg::Pose place_xy_settle_pose = before_xy_settle_stamped.pose;
  place_xy_settle_pose.position.x = place_gripper_center_x;
  place_xy_settle_pose.position.y = place_gripper_center_y;
  place_xy_settle_pose.position.z = before_xy_settle_stamped.pose.position.z;
  place_xy_settle_pose.orientation = selected_place_orientation;

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(25);
  move_group.setMaxVelocityScalingFactor(0.12);
  move_group.setMaxAccelerationScalingFactor(0.08);
  move_group.setGoalPositionTolerance(0.010);
  move_group.setGoalOrientationTolerance(0.20);

  enable_orientation_constraint(
    selected_place_orientation,
    0.20,
    "xy_settle_keep_vertical_down_orientation"
  );

  if (!plan_and_execute_pose(place_xy_settle_pose, "xy_settle_above_place_before_vertical_descend")) {
    clear_orientation_constraint();

    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to settle x/y above place. Stop because vertical descend would become diagonal."
    );

    stop_and_exit();
    return 1;
  }

  clear_orientation_constraint();

  // ============================================================
  // 9. 重新读取真实位置，然后只改 z 做竖直下降
  // ============================================================

  auto vertical_start_stamped = move_group.getCurrentPose();

  geometry_msgs::msg::Pose vertical_start_pose = vertical_start_stamped.pose;
  vertical_start_pose.orientation =
    normalize_quaternion(vertical_start_stamped.pose.orientation);

  selected_place_orientation = vertical_start_pose.orientation;

  const double xy_error = std::sqrt(
    std::pow(vertical_start_pose.position.x - place_gripper_center_x, 2.0) +
    std::pow(vertical_start_pose.position.y - place_gripper_center_y, 2.0)
  );

  RCLCPP_INFO(
    node->get_logger(),
    "Before vertical descend: current x=%.4f y=%.4f, target gripper center x=%.4f y=%.4f, xy_error=%.4f",
    vertical_start_pose.position.x,
    vertical_start_pose.position.y,
    place_gripper_center_x,
    place_gripper_center_y,
    xy_error
  );

  if (xy_error > 0.025) {
    RCLCPP_ERROR(
      node->get_logger(),
      "XY error before vertical descend is too large: %.4f. Stop to avoid diagonal descend.",
      xy_error
    );

    stop_and_exit();
    return 1;
  }

  geometry_msgs::msg::Pose place_hover_pose = vertical_start_pose;
  place_hover_pose.position.z = place_hover_z;
  place_hover_pose.orientation = selected_place_orientation;

  geometry_msgs::msg::Pose place_release_pose = vertical_start_pose;
  place_release_pose.position.z = place_release_z;
  place_release_pose.orientation = selected_place_orientation;

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(20);
  move_group.setMaxVelocityScalingFactor(0.10);
  move_group.setMaxAccelerationScalingFactor(0.06);
  move_group.setGoalPositionTolerance(0.015);
  move_group.setGoalOrientationTolerance(0.18);

  enable_orientation_constraint(
    selected_place_orientation,
    0.18,
    "strict_vertical_descend_keep_vertical_down_orientation"
  );

  if (!execute_cartesian_path(
        {place_hover_pose, place_release_pose},
        "strict_vertical_descend_to_place_release",
        0.90))
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Strict vertical descend failed. Retry with lower fraction requirement."
    );

    if (!execute_cartesian_path(
          {place_hover_pose, place_release_pose},
          "strict_vertical_descend_to_place_release_retry",
          0.65))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Vertical descend failed after retry."
      );

      clear_orientation_constraint();
      stop_and_exit();
      return 1;
    }
  }

  clear_orientation_constraint();

  std::this_thread::sleep_for(500ms);

  // 10. 松开夹爪
  if (!gripper.send(GRIPPER_OPEN, "release_object", 1.5, true)) {
    stop_and_exit();
    return 1;
  }

  std::this_thread::sleep_for(500ms);

  // 11. 竖直抬起
  geometry_msgs::msg::Pose retreat_after_place_pose = place_release_pose;
  retreat_after_place_pose.position.z = retreat_after_place_z;
  retreat_after_place_pose.orientation = selected_place_orientation;

  enable_orientation_constraint(
    selected_place_orientation,
    0.18,
    "vertical_retreat_keep_vertical_down_orientation"
  );

  if (!execute_cartesian_path(
        {retreat_after_place_pose},
        "vertical_retreat_after_place",
        0.60))
  {
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