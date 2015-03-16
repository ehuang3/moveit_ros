/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Georgia Tech Research Corporation
 *  All rights reserved.
 *
 *  Author(s): Eric Huang <ehuang@gatech.edu>
 *  Georgia Tech Humanoid Robotics Lab
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
#include <ros/package.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/window_manager_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <interactive_markers/tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/algorithm/string.hpp>
#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <apc_msgs/StoredScene.h>
#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::loadStateFromAction(robot_state::RobotState& state,
                                                  const apc_msgs::PrimitiveAction& action)
    {
        // Get the saved joint constraints.
        const trajectory_msgs::JointTrajectory& joint_trajectory = action.joint_trajectory;

        // Warn when the joint trajectory has more than one point!
        if (joint_trajectory.points.size() != 1)
            ROS_WARN("Loading state from an action with a dense trajectory!");

        // Copy the joints in the goal to the current state.
        for (int i = 0; i < joint_trajectory.joint_names.size(); i++)
        {
            state.setJointPositions(joint_trajectory.joint_names[i],
                                    &joint_trajectory.points[0].positions[i]);
        }

        // Add attached objects if they exist.
        computeAttachObjectToState(state,
                                   action.object_name,
                                   action.link_name,
                                   action.object_poses);

        // If this action was recorded with an attached object, default to
        // executing relative to object.
        if (!action.object_name.empty() && action.object_poses.size() > 0)
        {
            // Extract the object from the world.
            collision_detection::CollisionWorld::ObjectConstPtr object;
            {
                planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
                object = ps->getWorld()->getObject(action.object_name);
            }
            // If the object has not been loaded into the world, do nothing.
            if (!object)
            {
                ROS_ERROR("Failed to get object %s from world.",
                          action.object_name.c_str());
            }
            else
            {
                // Create link to object pose offsets from action.
                EigenSTL::vector_Affine3d poses(action.object_poses.size());
                for (int i = 0; i < action.object_poses.size(); i++)
                    tf::poseMsgToEigen(action.object_poses[i], poses[i]);
                // Back out the desired link transform in global coordinates.
                Eigen::Affine3d T_link = object->shape_poses_[0] * poses[0].inverse();
                // Create arguments for IK.
                const moveit::core::JointModelGroup* group = state.getJointModelGroup(action.group_name);
                geometry_msgs::Pose pose;
                tf::poseEigenToMsg(T_link, pose);
                // Compute IK to link.
                state.setFromIK(group, pose);
            }
        }

        // Update the dirty transforms, etc.
        state.update();
    }

    void MotionPlanningFrame::saveStateToAction(const robot_state::RobotState& state,
                                                apc_msgs::PrimitiveAction& action)
    {
        const moveit::core::JointModelGroup* joint_model_group = state.getJointModelGroup(action.group_name);
        if (!joint_model_group)
        {
            ROS_ERROR("Failed to get joint model group %s", action.group_name.c_str());
            return;
        }
        // Get the goal joint tolerance.
        double goal_joint_tolerance = move_group_->getGoalJointTolerance();
        // Construct goal constraints only for the joint model group.
        moveit_msgs::Constraints constraints = kinematic_constraints::constructGoalConstraints(state,
                                                                                               joint_model_group,
                                                                                               goal_joint_tolerance);
        // Copy joint angles and names to action.
        action.joint_trajectory.points.resize(1);
        for (int i = 0; i < constraints.joint_constraints.size(); i++)
        {
            action.joint_trajectory.joint_names.push_back(constraints.joint_constraints[i].joint_name);
            action.joint_trajectory.points[0].positions.push_back(constraints.joint_constraints[i].position);
        }
        // Save attached objects to the plan.
        computeAttachObjectToAction(state, action);
    }



}
