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
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include "ui_motion_planning_rviz_plugin_frame.h"
#include <eigen_conversions/eigen_msg.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <apc_msgs/GetMotionPlan.h>


namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::loadPlanToPreview(const moveit_msgs::RobotState& start_state,
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

        return;
    }

    void MotionPlanningFrame::computePlanButtonClicked()
    {
        // Reset last computed plan.
        primitive_plan_.reset(new apc_msgs::PrimitivePlan);

        // Get the list of active goals (waypoints) to follow.
        QListWidget* active_actions = ui_->active_actions_list;

        // Create an empty plan.
        apc_msgs::PrimitivePlan plan;

        // Appeach each active action to the plan.
        for (int i = 0; i < active_actions->count(); i++)
        {
            // Get the plan stored in the active action item.
            apc_msgs::PrimitivePlan stored_plan =
                getMessageFromUserData<apc_msgs::PrimitivePlan>(active_actions->item(i)->data(Qt::UserRole));

            // Append each action stored in the active action item.
            for (int j = 0; j < stored_plan.actions.size(); j++)
                plan.actions.push_back(stored_plan.actions[j]);
        }

        // Compute trajectory and save on success.
        if (computePlan(plan))
            *primitive_plan_ = plan;
        else
        {
            ROS_ERROR("Failed to compute TRAJOPT plan for active actions");
        }

        // Copy trajectory over to display.
        {
            moveit_msgs::RobotState start_state;
            robot_state::robotStateToRobotStateMsg(*planning_display_->getQueryStartState(), start_state);
            loadPlanToPreview(start_state, plan);
        }
    }



    bool MotionPlanningFrame::computePlan(apc_msgs::PrimitivePlan& plan)
    {
        // Create a motion plan service object.
        apc_msgs::GetMotionPlan srv;

        ROS_INFO("Computing motion plan using trajectory optimization");

        // Get the planning scene message.
        {
            const planning_scene_monitor::LockedPlanningSceneRO& locked_scene = planning_display_->getPlanningSceneRO();
            locked_scene->getPlanningSceneMsg(srv.request.scene);
        }

        // Set start state to the current state.
        robot_state::RobotState start_state = *planning_display_->getQueryStartState();
        {
            const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
            start_state = ps->getCurrentState();
        }
        planning_display_->setQueryStartState(start_state);

        // Create goal state.
        robot_state::RobotState goal_state = *planning_display_->getQueryGoalState();

        // Run plan through trajectory optimization.
        for (int i = 0; i < plan.actions.size(); i++)
        {
            // Construct robot goal state.
            // FIXME loadStateFromAction(goal_state, plan.actions[i]);

            // Write planning arguments into service message.
            robot_state::robotStateToRobotStateMsg(start_state, srv.request.start_state);
            robot_state::robotStateToRobotStateMsg(goal_state,  srv.request.goal_state);

            // Call motion planning service and wait.
            if(!motion_plan_client_.call(srv))
            {
                ROS_ERROR("Failed to call service GetMotionPlan");
                return false;
            }

            if (!srv.response.valid)
            {
                ROS_ERROR("Failed to compute valid TRAJOPT plan");
                return false;
            }

            // Write plan to output.
            plan.actions[i].joint_trajectory = srv.response.plan.actions[0].joint_trajectory;

            // Set next start state to this goal state.
            start_state = goal_state;
        }

        return true;
    }

    void MotionPlanningFrame::computeExecuteButtonClicked()
    {
        if (!execute_client_)
        {
            typedef actionlib::SimpleActionClient<apc_msgs::FollowPrimitivePlanAction> ActionClient;

            // Create an action client that spins its own thread.
            execute_client_.reset(new ActionClient("/crichton/trajectory_bridge", true));
        }

        if (!execute_client_->isServerConnected())
            if (!execute_client_->waitForServer(ros::Duration(1.0)))
            {
                ROS_ERROR("Unable to connect to action server.");
                return;
            }
            else
                planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::initExecuteProgressLabel, this));

        // Get goal.
        apc_msgs::FollowPrimitivePlanGoal goal;
        if (!primitive_plan_)
        {
            ROS_ERROR("No plan to execute");
            return;
        }
        goal.plan = *primitive_plan_;

        // Send the goal.
        execute_client_->sendGoal(goal,
                                  boost::bind(&MotionPlanningFrame::executeDoneCallback, this, _1, _2),
                                  boost::bind(&MotionPlanningFrame::executeActiveCallback, this),
                                  boost::bind(&MotionPlanningFrame::executeFeedbackCallback, this, _1));
    }

    void MotionPlanningFrame::executeActiveCallback()
    {
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateExecuteActive, this));
    }

    void MotionPlanningFrame::executeFeedbackCallback(const apc_msgs::FollowPrimitivePlanFeedbackConstPtr& feedback)
    {
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateExecuteFeedback,
                                                      this,
                                                      *feedback));
    }

    void MotionPlanningFrame::executeDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                  const apc_msgs::FollowPrimitivePlanResultConstPtr& result)
    {
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateExecuteDone,
                                                      this,
                                                      state,
                                                      *result));
    }

}
