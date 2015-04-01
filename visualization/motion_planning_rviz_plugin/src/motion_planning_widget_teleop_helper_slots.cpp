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


namespace moveit_rviz_plugin
{

    void MotionPlanningFrame::connectTeleopHelperSlots()
    {
        connect( ui_->start_to_current_button, SIGNAL( clicked() ), this, SLOT( setStartToCurrentButtonClicked() ));
        connect( ui_->goal_to_current_button,  SIGNAL( clicked() ), this, SLOT( setGoalToCurrentButtonClicked() ));
        connect( ui_->relative_to_object_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->relative_to_pose_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->eef_trajectory_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->dense_trajectory_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->monitor_contact_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->monitor_profile_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->cartesian_interpolate_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->group_combo_box, SIGNAL( activated(int) ), this, SLOT( groupComboBoxActivated(int) ));
        connect( ui_->joint_combo_box, SIGNAL( activated(int) ), this, SLOT( jointComboBoxActivated(int) ));
    }

    void MotionPlanningFrame::updateGroupComboBox()
    {
        // If the group combo box is empty, add a list of groups.
        if (ui_->group_combo_box->count() == 0)
        {
            const std::vector<std::string>& groups =
                planning_display_->getRobotModel()->getJointModelGroupNames();
            for (int i = 0; i < groups.size(); i++)
                ui_->group_combo_box->addItem(QString(groups[i].c_str()));
        }
        std::string group = planning_display_->getCurrentPlanningGroup();
        int index = ui_->group_combo_box->findText(QString::fromStdString(group));
        ui_->group_combo_box->setCurrentIndex(index);
    }

    void MotionPlanningFrame::groupComboBoxActivated(int index)
    {
        std::string group = ui_->group_combo_box->currentText().toStdString();
        if (group.empty())
            return;
        planning_display_->changePlanningGroup(group);
    }

    void MotionPlanningFrame::jointComboBoxActivated(int index)
    {
    }

    void MotionPlanningFrame::setStartToCurrentButtonClicked()
    {
        robot_state::RobotState start = *planning_display_->getQueryStartState();
        updateQueryStateHelper(start, "<current>");
        planning_display_->setQueryStartState(start);
    }

    void MotionPlanningFrame::setGoalToCurrentButtonClicked()
    {
        robot_state::RobotState goal = *planning_display_->getQueryGoalState();
        updateQueryStateHelper(goal, "<current>");
        planning_display_->setQueryGoalState(goal);
    }

    void MotionPlanningFrame::optionsCheckBoxClicked()
    {
        // Save the clicked options.
        QList<QListWidgetItem*> items = ui_->active_actions_list->selectedItems();
        saveOptionsFromView(items);
    }

}
