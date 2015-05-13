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
    apc_msgs::PrimitivePlan input_plan = _plan;
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
            ROS_WARN("Preparing to split action %s", input_plan.actions[i].action_name.c_str());

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
