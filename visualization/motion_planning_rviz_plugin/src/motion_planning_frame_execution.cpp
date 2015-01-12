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

    void setLabelColor(QLabel* label, std::string color)
    {
        label->setStyleSheet(QString("QLabel { color : %1; }").arg(color.c_str()));
    }

    void MotionPlanningFrame::initExecutePlans()
    {
        ui_->executeProgressLabel->setText("READY");
        setLabelColor(ui_->executeProgressLabel, "rgb(0,170,0)");
    }

    void MotionPlanningFrame::stopExecutionButtonClicked()
    {
        ROS_INFO("Cancelling all goals on server");
        execute_client_->cancelAllGoals();
    }

    void MotionPlanningFrame::executePlansButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecutePlansButtonClicked, this),
                                            "execute plans");
    }

    void MotionPlanningFrame::computeExecutePlansButtonClicked()
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
                planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::initExecutePlans, this));

        // Get goal.
        apc_msgs::FollowPrimitivePlanGoal goal;
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

    void MotionPlanningFrame::updateExecuteActive()
    {
        ROS_INFO("Goal is active!");
        ui_->executeProgressLabel->setText(QString("Executing"));
        setLabelColor(ui_->executeProgressLabel, "rgb(170,170,0)");
        int min = ui_->executeProgressBar->minimum();
        ui_->executeProgressBar->setValue(min);
        ui_->executeProgressBar->update();
    }

    void MotionPlanningFrame::updateExecuteFeedback(const apc_msgs::FollowPrimitivePlanFeedback& feedback)
    {
        ROS_INFO("%s (%.2f)", feedback.progress_string.c_str(), feedback.progress);
        int min = ui_->executeProgressBar->minimum();
        int max = ui_->executeProgressBar->maximum();
        ui_->executeProgressBar->setValue(feedback.progress * (max - min) + min);
        ui_->executeProgressBar->update();
    }

    void MotionPlanningFrame::updateExecuteDone(const actionlib::SimpleClientGoalState& state,
                                                const apc_msgs::FollowPrimitivePlanResult& result)
    {
        // Check for success.
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Yay! The primtive plan was executed");
            ui_->executeProgressLabel->setText(QString("READY"));
            setLabelColor(ui_->executeProgressLabel, "rgb(0,170,0)");
        }
        else
        {
            // ui_->executeProgressLabel->setText(QString("%1").arg(state.getText().c_str()));
            ui_->executeProgressLabel->setText(QString("FAILED"));
            setLabelColor(ui_->executeProgressLabel, "rgb(170,0,0)");
        }
    }

}
