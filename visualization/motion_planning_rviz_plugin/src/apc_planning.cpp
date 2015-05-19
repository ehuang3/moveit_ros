/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Georgia Tech Research Corporation
 *  All rights reserved.
 *
 *  Author(s): Eric Huang <ehuang@gatech.edu>
 *  Georgia Tech Socially Intelligent Machines Lab
 *  Under Direction of Prof. Andrea Thomaz <athomaz@cc.gatech.edu>
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <apc/planning.h>
#include <apc/exception.h>
#include <ros/ros.h>
#include <sstream>
#include <boost/xpressive/xpressive.hpp>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <moveit/warehouse/primitive_plan_storage.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>


bool apc_planning::_is_robot_moving_(const apc_msgs::PrimitiveAction& action)
{
    const double TOLERANCE = 1e-7;
    const trajectory_msgs::JointTrajectoryPoint& start = action.joint_trajectory.points.front();
    for (int i = 0; i < action.joint_trajectory.points.size(); i++) {
        const trajectory_msgs::JointTrajectoryPoint& current = action.joint_trajectory.points[i];
        for (int j = 0; j < current.positions.size(); j++)
            if (std::abs(start.positions[j] - current.positions[j]) > TOLERANCE)
                return true;
    }
    return false;
}

bool apc_planning::_is_object_moving_(const apc_msgs::PrimitiveAction& action)
{
    if (action.object_id.empty()) {
        return false;
    }
    if (action.object_trajectory.poses.size() == 0) {
        return false;
    }
    const double TOLERANCE = 1e-7;
    for (int i = 0; i < action.object_trajectory.poses.size(); i++) {
        const geometry_msgs::Pose& start = action.object_trajectory.poses.front();
        const geometry_msgs::Pose& current = action.object_trajectory.poses[i];
        Eigen::Affine3d T_start;
        Eigen::Affine3d T_current;
        tf::poseMsgToEigen(start, T_start);
        tf::poseMsgToEigen(current, T_current);
        if ((T_start.translation() - T_current.translation()).norm() > TOLERANCE)
            return true;
        Eigen::Quaterniond q_start(T_start.rotation());
        Eigen::Quaterniond q_current(T_current.rotation());
        double angle = q_start.angularDistance(q_current);
        if (std::abs(angle) > TOLERANCE)
            return true;
    }
    return false;
}

std::string apc_planning::_action_type_(const apc_msgs::PrimitiveAction& action)
{
    bool robot_moving  = _is_robot_moving_(action);
    bool object_moving = _is_object_moving_(action);
    bool grasping = action.grasp;
    bool has_object = !action.object_id.empty();
    // Compute action types.
    bool transit       =                                                   !has_object;
    bool pregrasp      = (( robot_moving && !grasping && !object_moving &&  has_object) ||
                          (!robot_moving && !grasping && !object_moving &&  has_object));
    bool grasp         =   !robot_moving &&  grasping && !object_moving &&  has_object;
    bool postgrasp     =    robot_moving &&  grasping                   &&  has_object;
    bool nonprehensile =    robot_moving && !grasping &&  object_moving &&  has_object;
    APC_ASSERT(transit ^ pregrasp ^ grasp ^ postgrasp ^ nonprehensile,
               "Requested impossible action type\n"
               "   transit: %d\n"
               "  pregrasp: %d\n"
               "     grasp: %d\n"
               " postgrasp: %d\n"
               "prehensile: %d\n"
               "\n"
               "   robot_moving: %d\n"
               "  object_moving: %d\n"
               "       grasping: %d\n"
               "     has_object: %d\n",
               transit, pregrasp, grasp, postgrasp, nonprehensile,
               robot_moving, object_moving, grasping, has_object);
    if (transit)
        return "transit";
    if (pregrasp)
        return "pregrasp";
    if (grasp)
        return "grasp";
    if (postgrasp)
        return "postgrasp";
    if (nonprehensile)
        return "nonprehensile";
}

bool apc_planning::is_action_transit(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "transit";
}

bool apc_planning::is_action_pregrasp(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "pregrasp";
}

bool apc_planning::is_action_grasp(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "grasp";
}

bool apc_planning::is_action_postgrasp(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "postgrasp";
}

bool apc_planning::is_action_nonprehensile(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "nonprehensile";
}

bool apc_planning::is_action_stationary(const apc_msgs::PrimitiveAction& action)
{
    return _is_robot_moving_(action);
}

void apc_planning::copyJointTrajectoryRestrictedToGroup(apc_msgs::PrimitiveAction& target,
                                                        const apc_msgs::PrimitiveAction& source,
                                                        const robot_state::RobotState& robot_state)
{
    APC_ASSERT(target.group_id != source.group_id,
               "Why are you copying the same group from source to target?");
    typedef std::vector<std::string> JointNames;
    JointNames target_names = robot_state.getJointModelGroup(target.group_id)->getVariableNames();
    JointNames source_names = source.joint_trajectory.joint_names;
    trajectory_msgs::JointTrajectory target_traj = target.joint_trajectory;
    trajectory_msgs::JointTrajectory source_traj = source.joint_trajectory;
    target_traj.joint_names.clear();
    target_traj.points.clear();
    target_traj.points.resize(source_traj.points.size());
    for (int i = 0; i < source_names.size(); i++) {
        if (std::find(target_names.begin(), target_names.end(), source_names[i]) != target_names.end()) {
            target_traj.joint_names.push_back(source_names[i]);
            for (int j = 0; j < source_traj.points.size(); j++) {
                target_traj.points[j].positions.push_back(source_traj.points[j].positions[i]);
            }
        }
    }
    APC_ASSERT(target_names.size() == target_traj.joint_names.size(),
               "Failed to find some variable name in source %s", source.action_name.c_str());
    target.joint_trajectory = target_traj;
}

void apc_planning::partitionPlanBySubgroups(apc_msgs::PrimitivePlan& _plan,
                                            const robot_state::RobotState& robot_state)
{
    const apc_msgs::PrimitivePlan input_plan = _plan;
    apc_msgs::PrimitivePlan output_plan;
    // Partition...
    for (int i = 0; i < input_plan.actions.size(); i++) {
        bool partition = false;
        kinematics::KinematicsBaseConstPtr solver =
            robot_state.getJointModelGroup(input_plan.actions[i].group_id)->getSolverInstance();
        if (solver) {
            partition = false;
        } else {
            // Check for subgroup disparity in kinematics sovlers.
            std::vector<const moveit::core::JointModelGroup*> subgroups;
            robot_state.getJointModelGroup(input_plan.actions[i].group_id)->getSubgroups(subgroups);
            int d = 0;
            for (int j = 0; j < subgroups.size(); j++) {
                solver = subgroups[j]->getSolverInstance();
                int s = solver ? 1 : -1;
                if (d == 0)
                    d = solver ? 1 : -1;
                else if (s != d) {
                    partition = true;
                    break;
                }
            }
        }

        if (partition) {
            int split_index = 0;
            std::vector<const moveit::core::JointModelGroup*> subgroups;
            robot_state.getJointModelGroup(input_plan.actions[i].group_id)->getSubgroups(subgroups);

            // First plan for the groups with a solver (i.e. the arm).
            for (int j = 0; j < subgroups.size(); j++) {
                solver = subgroups[j]->getSolverInstance();
                if (solver) {
                    // Clone the action.
                    apc_msgs::PrimitiveAction clone_action = input_plan.actions[i];
                    clone_action.group_id = subgroups[j]->getName();
                    copyJointTrajectoryRestrictedToGroup(clone_action,
                                                         input_plan.actions[i],
                                                         robot_state);
                    std::stringstream ss;
                    ss << clone_action.action_name << "_s" << split_index++;
                    clone_action.action_name = ss.str();
                    output_plan.actions.push_back(clone_action);
                }
            }

            // Next plan for groups without a solver (i.e. the hand).
            for (int j = 0; j < subgroups.size(); j++) {
                solver = subgroups[j]->getSolverInstance();
                if (!solver) {
                    // Clone the action.
                    apc_msgs::PrimitiveAction clone_action = input_plan.actions[i];
                    clone_action.group_id = subgroups[j]->getName();
                    copyJointTrajectoryRestrictedToGroup(clone_action,
                                                         input_plan.actions[i],
                                                         robot_state);
                    std::stringstream ss;
                    ss << clone_action.action_name << "_s" << split_index++;
                    clone_action.action_name = ss.str();
                    output_plan.actions.push_back(clone_action);
                }
            }
        } else {
            output_plan.actions.push_back(input_plan.actions[i]);
        }
    }
    output_plan.plan_name = input_plan.plan_name;
    APC_ASSERT(output_plan.actions.size() >= input_plan.actions.size(),
               "Failed to at least match input plan");
    _plan = output_plan;
}

void apc_planning::preprocessPlanBeforeExecution(apc_msgs::PrimitivePlan& plan,
                                                 const robot_state::RobotState& robot_state)
{
    // Ensure that only joints which correspond to motors are sent down to execute.
    for (int i = 0; i < plan.actions.size(); i++) {
        apc_msgs::PrimitiveAction& action = plan.actions[i];
        const std::vector<std::string>& A = robot_state.getJointModelGroup(action.group_id)->getActiveJointModelNames();
        const std::vector<std::string>& O = action.joint_trajectory.joint_names;
        trajectory_msgs::JointTrajectory        N;
        const trajectory_msgs::JointTrajectory& M = action.joint_trajectory;
        N.points.resize(M.points.size());
        for (int j = 0; j < O.size(); j++)
            if (std::find(A.begin(), A.end(), O[j]) != A.end()) {
                N.joint_names.push_back(O[j]);
                for (int k = 0; k < M.points.size(); k++)
                    N.points[k].positions.push_back(M.points[k].positions[j]);
            }
        action.joint_trajectory = N;
    }
}

void apc_planning::resetPlanJointTrajectories(apc_msgs::PrimitivePlan& plan)
{
    for (int i = 0; i < plan.actions.size(); i++) {
        trajectory_msgs::JointTrajectoryPoint back = plan.actions[i].joint_trajectory.points.back();
        plan.actions[i].joint_trajectory.points.resize(2);
        plan.actions[i].joint_trajectory.points[1] = back;
    }
}

void apc_planning::formatUniqueIndex(std::string& format, const std::vector<std::string>& existing)
{
    using namespace boost::xpressive;
    std::string expr = format;
    boost::replace_all(expr, "%i", "([0-9]+)");
    sregex rex = sregex::compile(expr);
    smatch what;
    int max = 0;
    for (int i = 0; i < existing.size(); i++) {
        if (regex_match(existing[i], what, rex)) {
            APC_ASSERT(what.size() != 1,
                       "Existing plan with name %s", what[0].str().c_str());
            int n = std::atoi(what[1].str().c_str());
            if (n >= max) {
                max = n + 1;
            }
        }
    }
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(3) << max;
    boost::replace_all(format, "%i", ss.str());
}

std::vector<std::string> apc_planning::getExistingPlanNames(QTreeWidget* plan_tree,
                                                            boost::shared_ptr<moveit_warehouse::PrimitivePlanStorage> primitive_plan_storage_)
{
    // Get all plan names from storage. FIXME We don't need to use this.
    // std::vector<std::string> stored_names;
    // primitive_plan_storage_->getKnownPrimitivePlans(plan_names);

    // Get all plan names from the current list.
    std::vector<std::string> active_names;
    for (int i = 0; i < plan_tree->topLevelItemCount(); i++) {
        active_names.push_back(plan_tree->topLevelItem(i)->text(0).toStdString());
    }
    return active_names;
}

void apc_planning::validatePlanningArguements(const apc_msgs::PrimitivePlan& plan)
{
    for (int i = 0; i < plan.actions.size(); i++) {
        const apc_msgs::PrimitiveAction& action = plan.actions[i];
        APC_ASSERT((!action.object_id.empty() && !action.object_key.empty()) ||
                   ( action.object_id.empty() &&  action.object_key.empty()),
                   "Failed ID-KEY check on action %s in plan %s\n"
                   "     object_id: %s\n"
                   "    object_key: %s",
                   action.action_name.c_str(), plan.plan_name.c_str(),
                   action.object_id.c_str(), action.object_key.c_str());
    }
}

void apc_planning::clampJointLimitsInPlan(apc_msgs::PrimitivePlan& plan,
                                          const robot_state::RobotState& robot_state)
{
    for (int i = 0; i < plan.actions.size(); i++) {
        apc_msgs::PrimitiveAction& action = plan.actions[i];
        APC_ASSERT(robot_state.getJointModelGroup(action.group_id)->getSubgroupNames().size() == 0,
                   "Failed to assert that no subgroups exist for action %s in plan %s",
                   action.action_name.c_str(), plan.plan_name.c_str());
        APC_ASSERT(action.joint_trajectory.points.size() > 0,
                   "Failed to find joint trajectory for action %s in plan %s",
                   action.action_name.c_str(), plan.plan_name.c_str());
        const std::vector<std::string>& joint_names = action.joint_trajectory.joint_names;
        for (int j = 0; j < joint_names.size(); j++) {
            double max_pos = robot_state.getRobotModel()->getVariableBounds(action.joint_trajectory.joint_names[j]).max_position_;
            double min_pos = robot_state.getRobotModel()->getVariableBounds(action.joint_trajectory.joint_names[j]).min_position_;
            trajectory_msgs::JointTrajectory& T = action.joint_trajectory;
            for (int k = 0; k < T.points.size(); k++) {
                double& q = T.points[k].positions[j];
                if (q < min_pos || q > max_pos) {
                    ROS_DEBUG_STREAM("Clamping " << joint_names[j]);
                    q = std::min(std::max(q, min_pos), max_pos);
                }
            }
        }
    }
}
