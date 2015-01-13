/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
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

/* Author: Eric Huang */

#include <moveit/warehouse/primitive_plan_storage.h>
#include <QMessageBox>

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <std_srvs/Empty.h>
#include <apc_msgs/FollowPrimitivePlanAction.h>
#include <actionlib/client/simple_action_client.h>

#include "ui_motion_planning_rviz_plugin_frame.h"



namespace moveit_rviz_plugin
{

    void MotionPlanningFrame::planGoalsButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanGoalsButtonClicked, this), "plan goals");
    }

    void MotionPlanningFrame::previewButtonClicked()
    {
        planning_display_->previewTrail();
    }

    void MotionPlanningFrame::computeLinearInterpPlan(const robot_state::RobotState& start,
                                                      apc_msgs::PrimitiveAction& goal)
    {
        std::string group = goal.group_name;
        const moveit::core::JointModelGroup* joint_group = start.getJointModelGroup(group);
        trajectory_msgs::JointTrajectoryPoint s, c, e;
        const std::vector<std::string>& joint_names = goal.joint_trajectory.joint_names;
        s = goal.joint_trajectory.points.front();
        for (int i = 0; i < joint_names.size(); i++)
            s.positions[i] = (start.getVariablePosition(joint_names[i]));
        e = goal.joint_trajectory.points.back();
        goal.joint_trajectory.points.clear();
        trajectory_msgs::JointTrajectory& T = goal.joint_trajectory;
        const int n = 15;
        c = s;
        for (int i = 0; i <= n; i++)
        {
            for (int j=0; j < c.positions.size(); j++)
                c.positions[j] = s.positions[j] + i * (e.positions[j] - s.positions[j]) / n;
            T.points.push_back(c);
        }
    }

    void MotionPlanningFrame::computePlanGoalsButtonClicked()
    {
        if (!move_group_)
            return;

        // Clear status
        ui_->result_label->setText("Planning...");

        // Reset the current plan.
        current_plan_.reset(new moveit::planning_interface::MoveGroup::Plan());

        // The primitive plan is used for actual execution on the robot. HACK
        // We reset it here.
        primitive_plan_.reset(new apc_msgs::PrimitivePlan);

        // Get the list of goals (waypoints) to follow.
        QListWidget* goals_list = ui_->active_goals_list;

        // Get the current start state.
        robot_state::RobotState start_state = *planning_display_->getQueryStartState();

        // The target goal state will be initialized to the start state.
        robot_state::RobotState goal_state = start_state;

        // For each item in the active goals list, configure for planning and then
        // append to the plan.
        for (int i = 0; i < goals_list->count(); i++)
        {
            // First get the plan represented by the item.
            apc_msgs::PrimitivePlan plan =
                getMessageFromUserData<apc_msgs::PrimitivePlan>(goals_list->item(i)->data(Qt::UserRole));

            // Loop through the actions in the plan.
            for (int j = 0; j < plan.actions.size(); j++)
            {
                // Get the last action from the user data because it is the goal state.
                apc_msgs::PrimitiveAction action = plan.actions.back();

                // Get the goal robot state from user data.
                getStateFromAction(goal_state, action);

                // HACK Reset move group so that I can plan with a different group... SMH.
                changePlanningGroupHelper(action.group_name);
                planning_display_->waitForAllMainLoopJobs(); // I hope there are no cyclic main job loops.

                // Set move group variables, like start and goal states, etc.
                configureForPlanning(start_state, goal_state);

                // Compute plan specific to the current primitive action.
                moveit::planning_interface::MoveGroup::Plan move_group_plan;

                // Make a planning service call. This will append any plans to the input.
                if (!move_group_->plan(move_group_plan))
                {
                    ui_->result_label->setText("Failed");
                    current_plan_.reset();
                    return;
                }

                // Copy plan over to primitive action.
                action.joint_trajectory = move_group_plan.trajectory_.joint_trajectory;
                action.duration = move_group_plan.planning_time_;

                // TODO Eef trajectory support.

                // TODO Dense trajectory support.

                // HACK If we are planning for the hands, overwrite plan with linear interpolation.
                if (action.group_name == "crichton_left_hand" ||
                    action.group_name == "crichton_right_hand")
                {
                    computeLinearInterpPlan(start_state, action);
                    move_group_plan.trajectory_.joint_trajectory = action.joint_trajectory;
                }

                // Append plan onto the current plan.
                move_group_->appendPlan(*current_plan_, move_group_plan);

                // Append action onto primitive plan.
                primitive_plan_->actions.push_back(action);

                // Start the next plan from this goal.
                start_state = goal_state;
            }
        }

        // Success
        ui_->execute_button->setEnabled(true);
        ui_->result_label->setText(QString("Time: ").append(
                                       QString::number(current_plan_->planning_time_,'f',3)));

        // Copy trajectory over to display.
        {
            // Get a robot model.
            const robot_model::RobotModelConstPtr& robot_model = planning_display_->getRobotModel();

            // Construct a new robot trajectory.
            robot_trajectory::RobotTrajectoryPtr display_trajectory(new robot_trajectory::RobotTrajectory(robot_model, ""));

            // Copy current plan over to robot trajectory.
            display_trajectory->setRobotTrajectoryMsg(planning_display_->getPlanningSceneRO()->getCurrentState(),
                                                      current_plan_->start_state_,
                                                      current_plan_->trajectory_);
            // Swap the plan trajectory into our planning display.
            planning_display_->setTrajectoryToDisplay(display_trajectory);

            // Display trail. FIXME This doesn't accomplish anything actually.
            previewButtonClicked();
        }

    }

}
