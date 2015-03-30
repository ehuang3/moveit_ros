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


namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::connectTeleopSlots()
    {
        connect( ui_->plan_button,    SIGNAL( clicked() ), this, SLOT( planButtonClicked() ));
        connect( ui_->execute_button, SIGNAL( clicked() ), this, SLOT( executeButtonClicked() ));
        connect( ui_->preview_button, SIGNAL( clicked() ), this, SLOT( previewButtonClicked() ));
        connect( ui_->stop_button,    SIGNAL( clicked() ), this, SLOT( stopButtonClicked() ));
    }

    void MotionPlanningFrame::planButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanButtonClicked, this), "plan goals");
    }

    void MotionPlanningFrame::previewButtonClicked()
    {
        planning_display_->previewTrail();
    }

    void MotionPlanningFrame::executeButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this),
                                            "execute plans");
    }

    void MotionPlanningFrame::stopButtonClicked()
    {
        ROS_INFO("Cancelling all goals on server");
        execute_client_->cancelAllGoals();
    }

    void setLabelColor(QLabel* label, std::string color)
    {
        label->setStyleSheet(QString("QLabel { color : %1; background-color: rgb(212, 208, 200); }").arg(color.c_str()));
    }

    void MotionPlanningFrame::initExecuteProgressLabel()
    {
        ui_->executeProgressLabel->setText("READY");
        setLabelColor(ui_->executeProgressLabel, "rgb(0,170,0)");
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
