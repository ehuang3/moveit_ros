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
        connect( ui_->padlock_button,     SIGNAL( clicked() ), this, SLOT( padlockButtonClicked() ));
        connect( ui_->start_radio_button, SIGNAL( clicked() ), this, SLOT( startRadioButtonClicked() ));
        connect( ui_->goal_radio_button,  SIGNAL( clicked() ), this, SLOT( goalRadioButtonClicked() ));
        connect( ui_->group_combo_box, SIGNAL( activated(int) ), this, SLOT( groupComboBoxActivated(int) ));

        connect( ui_->start_to_current_button, SIGNAL( clicked() ), this, SLOT( setStartToCurrentButtonClicked() ));
        connect( ui_->goal_to_current_button,  SIGNAL( clicked() ), this, SLOT( setGoalToCurrentButtonClicked() ));
        connect( ui_->monitor_contact_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->monitor_profile_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        connect( ui_->cartesian_interpolate_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));

    }

    void MotionPlanningFrame::padlockButtonClicked()
    {
        // Update text to reflect locked state.
        const QString lock = QString::fromUtf8("\uF023");
        const QString unlock = QString::fromUtf8("\uF13E");
        bool checked = ui_->padlock_button->isChecked();
        ui_->padlock_button->setText(checked ? lock : unlock);

        // Toggle start and goal buttons.
        ui_->start_radio_button->setEnabled(!checked);
        ui_->goal_radio_button->setEnabled(!checked);

        // Update query visualizations.
        planning_display_->query_start_state_property_->setBool(!checked);
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers,
                                                        planning_display_, true),
                                            "publishInteractiveMarkers");
    }

    void MotionPlanningFrame::startRadioButtonClicked()
    {
        // Update query visualizations.
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers,
                                                        planning_display_, true),
                                            "publishInteractiveMarkers");
    }

    void MotionPlanningFrame::goalRadioButtonClicked()
    {
        // Update query visualizations.
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers,
                                                        planning_display_, true),
                                            "publishInteractiveMarkers");
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

    void MotionPlanningFrame::updateFrameComboBox()
    {
        QComboBox* frame = ui_->frame_combobox;
        // Clear combo box.
        frame->clear();
        // Add blank space.
        frame->addItem("");
        // Add highlighted item.
        if (ui_->bin_contents_table_widget->selectedItems().size() > 0)
            frame->addItem(ui_->bin_contents_table_widget->selectedItems()[0]->text());
        // Add generic "bin".
        frame->addItem("bin");
        // Add specific bins.
        for (char a = 'A'; a <= 'L'; a++)
            frame->addItem(QString("bin_%1").arg(a));
        // Add objects.
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        for (int i = 0; i < bin_contents->rowCount(); i++)
            if (frame->findText(bin_contents->item(i, 1)->text(), Qt::MatchExactly) == -1)
                frame->addItem(bin_contents->item(i, 1)->text());
    }

    void MotionPlanningFrame::frameComboBoxActivated(int)
    {
        // QListWidget* active_actions_widget = ui_->active_actions_list;
        // if (active_actions_widget->selectedItems().count() != 1)
        // {
        //     ROS_WARN("Cannot determine action from active action selection");
        //     return;
        // }

        // if (!checked)
        // {

        // }

        // QTableWidget* bin_contents_widget = ui_->bin_contents_table_widget;

        // if (bin_contents_widget->selectedItems().count() != 1)
        // {
        //     ROS_WARN("Cannot determine frame from bin content selection");
        //     return;
        // }
    }

    void MotionPlanningFrame::updateObjectComboBox()
    {
        QComboBox* object = ui_->object_combobox;
        // Clear combo box.
        object->clear();
        // Add blank space.
        object->addItem("");
        // Add highlighted item if it is an object.
        if (ui_->bin_contents_table_widget->selectedItems().size() > 0 &&
            ui_->bin_contents_table_widget->selectedItems()[0]->column() == 1)
            object->addItem(ui_->bin_contents_table_widget->selectedItems()[0]->text());
        // Add objects.
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        for (int i = 0; i < bin_contents->rowCount(); i++)
            if (object->findText(bin_contents->item(i, 1)->text(), Qt::MatchExactly) == -1) // No duplicates.
                object->addItem(bin_contents->item(i, 1)->text());
    }

    void MotionPlanningFrame::objectComboBoxActivated(int)
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
