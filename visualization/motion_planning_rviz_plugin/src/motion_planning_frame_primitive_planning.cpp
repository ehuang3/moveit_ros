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

#include <apc_msgs/GetMotionPlan.h>

#include "ui_motion_planning_rviz_plugin_frame.h"



namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::pickAndPlaceButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePickAndPlaceButtonClicked, this), "motion plan");
    }

    void MotionPlanningFrame::planGoalsButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanGoalsButtonClicked, this), "plan goals");
    }

    void MotionPlanningFrame::previewButtonClicked()
    {
        planning_display_->previewTrail();
    }

    void MotionPlanningFrame::copyTrajectoryToDisplay(const moveit_msgs::RobotState& start_state,
                                                      const apc_msgs::PrimitivePlan& plan)
    {
        // Get a robot model.
        const robot_model::RobotModelConstPtr& robot_model = planning_display_->getRobotModel();

        // Construct a new robot trajectory.
        robot_trajectory::RobotTrajectoryPtr display_trajectory(new robot_trajectory::RobotTrajectory(robot_model, ""));

        // Accumulate joint trajectory over entire plan.
        trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
        for (int i = 0; i < plan.actions.size(); i++)
            appendToTrajectory(trajectory, plan.actions[i].joint_trajectory);

        // Copy current plan over to robot trajectory.
        display_trajectory->setRobotTrajectoryMsg(planning_display_->getPlanningSceneRO()->getCurrentState(),
                                                  // FIXME start_state,
                                                  trajectory);

        // Swap the plan trajectory into our planning display.
        planning_display_->setTrajectoryToDisplay(display_trajectory);

        // Display trail. FIXME This doesn't accomplish anything actually.
        previewButtonClicked();
    }

    void MotionPlanningFrame::appendToTrajectory(trajectory_msgs::JointTrajectory& first,
                                                 const trajectory_msgs::JointTrajectory& second)
    {
        // If the first plan is empty, copy the second over to the
        // first.
        if (first.joint_names.size() == 0)
        {
            first = second;
            return;
        }
        // Build a map from joint names to joint indexes and of
        // uniqueness of joint names between the two trajectory.
        int j_index = 0;                       // New joint index.
        std::map<std::string, int>  j_map;     // Map from joint name to new index.
        std::map<std::string, bool> j1_unique; // Uniqueness of joint to first trajectory.
        std::map<std::string, bool> j2_unique; // Uniqueness of joint to second trajectory.
        std::vector<std::string>    j_names;   // New order of joint names.
        const std::vector<std::string>& j1 = first.joint_names;
        for (int j = 0; j < j1.size(); j++)
        {
            if (j_map.count(j1[j]) == 0)
                j_map[j1[j]] = j_index++;
            j1_unique[j1[j]] = true;
            j2_unique[j1[j]] = false;
            j_names.push_back(j1[j]);
        }
        const std::vector<std::string>& j2 = second.joint_names;
        for (int j = 0; j < j2.size(); j++)
        {
            if (j_map.count(j2[j]) == 0)
            {
                j_map[j2[j]] = j_index++;
                j2_unique[j2[j]] = true;
                j_names.push_back(j2[j]);
            }
            j1_unique[j2[j]] = false;
        }
        // If velocities and forces are unset, fill them in for the first trajectory.
        int n_dof_first = first.joint_names.size();
        for (int i = 0; i < first.points.size(); i++)
        {
            if (!first.points[i].velocities.size())
                first.points[i].velocities.resize(n_dof_first);
            if (!first.points[i].accelerations.size())
                first.points[i].accelerations.resize(n_dof_first);
        }
        std::map<std::string, int> j2_map; // Map from joint name to joint index in second start state.
        const trajectory_msgs::JointTrajectoryPoint& j2_start = second.points[0];
        for (int i = 0; i < second.joint_names.size(); i++)
            j2_map[second.joint_names[i]] = i;
        // Append the new joint names to the first plan.
        first.joint_names = j_names;
        // Fill in missing joints from the second plan to the first plan.
        typedef std::map<std::string, bool>::const_iterator UniqueJointIterator;
        std::vector<trajectory_msgs::JointTrajectoryPoint>& p1 = first.points;
        if (j2_unique.size() > 0)
            for (int i = 0; i < p1.size(); i++)
                for (UniqueJointIterator iter = j2_unique.begin(); iter != j2_unique.end(); ++iter)
                    if (iter->second)
                    {
                        p1[i].positions.push_back(j2_start.positions[j2_map[iter->first]]);
                        p1[i].velocities.push_back(0);
                        p1[i].accelerations.push_back(0);
                        // p1[i].effort.push_back(0); // The planner does not return effort!
                    }
        // Append the second plan to the first plan.
        const std::vector<trajectory_msgs::JointTrajectoryPoint>& p2 = second.points;
        trajectory_msgs::JointTrajectoryPoint point = p1.back();
        const ros::Duration p1_time = point.time_from_start;
        for (int i = 0; i < p2.size(); i++)
        {
            for (int j = 0; j < j2.size(); j++)
            {
                point.positions[j_map[j2[j]]] = p2[i].positions[j];
                if (p2[i].velocities.size() > 0)
                    point.velocities[j_map[j2[j]]] = p2[i].velocities[j];
                else
                    point.velocities[j_map[j2[j]]] = 0;
                if (p2[i].accelerations.size() > 0)
                    point.accelerations[j_map[j2[j]]] = p2[i].accelerations[j];
                else
                    point.accelerations[j_map[j2[j]]] = 0;
                // point.effort[j_map[j2[j]]] = p2[i].effort[j]; // The planner does not return effort!
            }
            point.time_from_start = p1_time + p2[i].time_from_start;
            p1.push_back(point);
        }
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

    void MotionPlanningFrame::computePickAndPlaceButtonClicked()
    {
        // Create a motion plan service object.
        apc_msgs::GetMotionPlan srv;

        // Get the planning scene message.
        const planning_scene_monitor::LockedPlanningSceneRO& locked_scene = planning_display_->getPlanningSceneRO();
        if (!locked_scene)
        {
            ROS_ERROR("Failed to acquire locked planning scene");
            return;
        }
        locked_scene->getPlanningSceneMsg(srv.request.scene);

        // Get the robot start state.
        robot_state::robotStateToRobotStateMsg(*planning_display_->getQueryStartState(), srv.request.start_state);

        // Get the robot goal state.
        robot_state::robotStateToRobotStateMsg(*planning_display_->getQueryGoalState(), srv.request.goal_state);

        // TODO Get the object goal states.

        // Call motion planning service and wait.
        if(motion_plan_client_.call(srv))
        {
            // ROS_INFO_STREAM("Response: " << srv.response);
        }
        else
        {
            ROS_ERROR("Failed to call service GetMotionPlan");
            return;
        }

        // Copy trajectory over to display.
        copyTrajectoryToDisplay(srv.request.start_state, srv.response.plan);

        // Copy trajectory into active goals list.
    }

}
