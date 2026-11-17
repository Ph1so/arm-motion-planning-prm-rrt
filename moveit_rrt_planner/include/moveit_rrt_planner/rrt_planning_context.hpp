#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

#include <random>
#include <limits>
#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace moveit_rrt_planner
{

struct TreeNode
{
  std::vector<double> q;
  int parent = -1;
};

class RRTPlanningContext : public planning_interface::PlanningContext
{
public:
  RRTPlanningContext(const std::string& name,
                     const std::string& group,
                     const moveit::core::RobotModelConstPtr& model,
                     const planning_scene::PlanningSceneConstPtr& scene)
  : planning_interface::PlanningContext(name, group)
  , model_(model)
  , scene_(scene)
  {}

  bool terminate() override { return false; }
  void clear() override {}

  void solve(planning_interface::MotionPlanResponse& res) override
  {
    const auto t0 = std::chrono::steady_clock::now();

    const auto* jmg = model_->getJointModelGroup(getGroupName());
    if (!jmg)
    {
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return;
    }

    moveit::core::RobotState start_state = scene_->getCurrentState();
    if (!request_.start_state.is_diff)
      moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
    start_state.update();

    std::vector<double> q_start, q_goal;
    start_state.copyJointGroupPositions(jmg, q_start);
    q_goal = q_start;

    if (request_.goal_constraints.empty() ||
        request_.goal_constraints[0].joint_constraints.empty())
    {
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return;
    }

    const auto& var_names = jmg->getVariableNames();
    std::unordered_map<std::string, std::size_t> name_to_idx;
    for (std::size_t i = 0; i < var_names.size(); ++i) name_to_idx[var_names[i]] = i;

    for (const auto& jc : request_.goal_constraints[0].joint_constraints)
    {
      auto it = name_to_idx.find(jc.joint_name);
      if (it != name_to_idx.end())
        q_goal[it->second] = jc.position;
    }

    std::vector<moveit::core::VariableBounds> v_bounds;
    v_bounds.reserve(var_names.size());
    for (const auto& n : var_names) v_bounds.push_back(model_->getVariableBounds(n));
    const auto var_count = v_bounds.size();

    auto clampToBounds = [&](std::vector<double>& q)
    {
      for (size_t i = 0; i < q.size() && i < var_count; ++i)
      {
        const auto& b = v_bounds[i];
        if (b.position_bounded_)
        {
          q[i] = std::min(std::max(q[i], b.min_position_), b.max_position_);
        }
      }
    };

    const double step = 0.08;
    const double goal_thresh = 0.04;
    const double goal_bias = 0.10;
    const int    max_iters = 5000;
    const int    segment_checks = 10;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    auto dist = [&](const std::vector<double>& a, const std::vector<double>& b)
    {
      double s = 0.0;
      for (size_t i = 0; i < a.size(); ++i) { double d = a[i]-b[i]; s += d*d; }
      return std::sqrt(s);
    };

    auto collisionFreeState = [&](const std::vector<double>& q) -> bool
    {
      moveit::core::RobotState st(start_state);
      st.setJointGroupPositions(jmg, q);
      st.update();
      return !scene_->isStateColliding(st, getGroupName());
    };

    auto collisionFreeSegment = [&](const std::vector<double>& q1,
                                    const std::vector<double>& q2) -> bool
    {
      for (int i = 0; i <= segment_checks; ++i)
      {
        double a = static_cast<double>(i)/segment_checks;
        std::vector<double> q(q1.size());
        for (size_t j = 0; j < q.size(); ++j) q[j] = (1.0-a)*q1[j] + a*q2[j];
        if (!collisionFreeState(q)) return false;
      }
      return true;
    };

    clampToBounds(q_start);
    clampToBounds(q_goal);

    if (!collisionFreeState(q_start))
    {
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION;
      return;
    }
    if (!collisionFreeState(q_goal))
    {
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION;
      return;
    }

    std::vector<TreeNode> tree;
    tree.push_back(TreeNode{q_start, -1});
    int goal_idx = -1;

    for (int it = 0; it < max_iters; ++it)
    {
      // Sample the goal with probability goal_bias, otherwise a uniform
      // random configuration inside the joint limits.
      std::vector<double> q_rand(q_start.size());
      if (uni01(rng) < goal_bias)
      {
        q_rand = q_goal;
      }
      else
      {
        for (size_t i = 0; i < q_rand.size() && i < var_count; ++i)
        {
          const auto& b = v_bounds[i];
          q_rand[i] = uni01(rng)*(b.max_position_ - b.min_position_) + b.min_position_;
        }
      }
      clampToBounds(q_rand);

      // Nearest node in the tree to the sample.
      int nn = 0;
      double best = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < tree.size(); i++)
      {
        double d = dist(tree[i].q, q_rand);
        if (d < best)
        {
          best = d;
          nn = static_cast<int>(i);
        }
      }

      // Steer from the nearest node toward the sample by at most 'step'.
      std::vector<double> q_new(q_start.size());
      if (best <= step)
      {
        q_new = q_rand;
      }
      else
      {
        double s = step / best;
        for (size_t i = 0; i < q_new.size(); ++i)
          q_new[i] = tree[nn].q[i] + s*(q_rand[i] - tree[nn].q[i]);
      }
      clampToBounds(q_new);

      // Discard the extension if the motion to it collides.
      if (!collisionFreeSegment(tree[nn].q, q_new)) continue;

      tree.push_back(TreeNode{q_new, nn});
      int new_idx = static_cast<int>(tree.size()) - 1;

      // Close enough to the goal with a clear straight line to it: done.
      if (dist(q_new, q_goal) < goal_thresh && collisionFreeSegment(q_new, q_goal))
      {
        tree.push_back(TreeNode{q_goal, new_idx});
        goal_idx = static_cast<int>(tree.size()) - 1;
        break;
      }
    }

    if (goal_idx < 0)
    {
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return;
    }

    // Walk parent links from the goal back to the root, then reverse.
    std::vector<std::vector<double>> path;
    for (int i = goal_idx; i != -1; i = tree[i].parent)
      path.push_back(tree[i].q);
    std::reverse(path.begin(), path.end());

    moveit::core::RobotState st(start_state);
    robot_trajectory::RobotTrajectory traj(model_, jmg);
    const double dt = 0.1;
    bool first = true;
    for (const auto& q : path)
    {
      st.setJointGroupPositions(jmg, q);
      st.update();
      traj.addSuffixWayPoint(st, first ? 0.0 : dt);
      first = false;
    }

    res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(traj);
    res.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  void solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    planning_interface::MotionPlanResponse simple;
    solve(simple);

    res.error_code = simple.error_code;
    if (simple.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) return;

    res.trajectory.push_back(simple.trajectory);
    res.description.push_back("plan");
    res.processing_time.push_back(simple.planning_time);
  }

private:
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningSceneConstPtr scene_;
};

}  // namespace moveit_rrt_planner
