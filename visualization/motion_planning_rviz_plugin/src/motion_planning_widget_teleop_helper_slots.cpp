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
        connect( ui_->display_current_button,     SIGNAL( toggled(bool) ), this, SLOT( displayCurrentButtonToggled(bool) ));
        connect( ui_->display_start_button,     SIGNAL( toggled(bool) ), this, SLOT( displayStartButtonToggled(bool) ));
        connect( ui_->display_goal_button,     SIGNAL( toggled(bool) ), this, SLOT( displayGoalButtonToggled(bool) ));
        connect( ui_->display_eef_button,     SIGNAL( toggled(bool) ), this, SLOT( displayEefButtonToggled(bool) ));
        connect( ui_->display_joints_button,     SIGNAL( toggled(bool) ), this, SLOT( displayJointsButtonToggled(bool) ));
        connect( ui_->display_shelf_button,     SIGNAL( toggled(bool) ), this, SLOT( displayKivaPodButtonToggled(bool) ));
        connect( ui_->display_objects_button,     SIGNAL( toggled(bool) ), this, SLOT( displayObjectsButtonToggled(bool) ));

        connect( ui_->current_to_start_button, SIGNAL( clicked() ), this, SLOT( setCurrentToStartButtonClicked() ));
        connect( ui_->current_to_goal_button,  SIGNAL( clicked() ), this, SLOT( setCurrentToGoalButtonClicked() ));
        connect( ui_->start_to_goal_button, SIGNAL( clicked() ), this, SLOT( setStartToGoalButtonClicked() ));
        connect( ui_->goal_to_start_button,  SIGNAL( clicked() ), this, SLOT( setGoalToStartButtonClicked() ));

        connect( ui_->padlock_button,     SIGNAL( toggled(bool) ), this, SLOT( padlockButtonToggled(bool) ));
        connect( ui_->start_radio_button, SIGNAL( clicked() ), this, SLOT( startRadioButtonClicked() ));
        connect( ui_->goal_radio_button,  SIGNAL( clicked() ), this, SLOT( goalRadioButtonClicked() ));
        connect( ui_->group_combo_box, SIGNAL( activated(int) ), this, SLOT( groupComboBoxActivated(int) ));
        connect( ui_->frame_combobox,  SIGNAL( activated(const QString&) ),
                 this, SLOT( frameComboBoxActivated(const QString&) ));
        connect( ui_->object_combobox,  SIGNAL( currentIndexChanged(const QString&) ),
                 this, SLOT( objectComboBoxCurrentIndexChanged(const QString&) ));
        connect( ui_->grasp_checkbox,  SIGNAL( toggled(bool) ), this, SLOT( graspCheckBoxToggled(bool) ));

        // connect( ui_->monitor_contact_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        // connect( ui_->monitor_profile_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
        // connect( ui_->interpolate_cartesian_checkbox, SIGNAL( clicked() ), this, SLOT( optionsCheckBoxClicked() ));
    }

    void MotionPlanningFrame::displayCurrentButtonToggled(bool checked)
    {
        planning_display_->setSceneRobotVisualEnabled(checked);
    }

    void MotionPlanningFrame::displayStartButtonToggled(bool checked)
    {
        planning_display_->setQueryStartVisualEnabled(checked);
    }

    void MotionPlanningFrame::displayGoalButtonToggled(bool checked)
    {
        planning_display_->setQueryGoalVisualEnabled(checked);
    }

    void MotionPlanningFrame::displayEefButtonToggled(bool checked)
    {
        planning_display_->setEefMarkersActive(checked);
    }

    void MotionPlanningFrame::displayJointsButtonToggled(bool checked)
    {
        planning_display_->setJointMarkersActive(checked);
    }

    void MotionPlanningFrame::displayKivaPodButtonToggled(bool checked)
    {
        planning_display_->setObjectVisibility("kiva_pod", checked);
        planning_display_->queueRenderSceneGeometry();
    }

    void MotionPlanningFrame::displayObjectsButtonToggled(bool checked)
    {
        std::vector<std::string> object_keys = planning_display_->getPlanningSceneRO()->getWorld()->getObjectIds();
        for (int i = 0; i < object_keys.size(); i++) {
            if (object_keys[i] == "kiva_pod")
                continue;
            planning_display_->setObjectVisibility(object_keys[i], checked);
        }
        planning_display_->queueRenderSceneGeometry();
    }

    void MotionPlanningFrame::setCurrentToStartButtonClicked()
    {
        const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
        robot_state::RobotState start = *planning_display_->getQueryStartState();
        robot_state::RobotState current = ps->getCurrentState();
        start.setVariablePositions(current.getVariablePositions());
        planning_display_->setQueryStartState(start);
    }

    void MotionPlanningFrame::setCurrentToGoalButtonClicked()
    {
        const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
        robot_state::RobotState goal = *planning_display_->getQueryGoalState();
        robot_state::RobotState current = ps->getCurrentState();
        goal.setVariablePositions(current.getVariablePositions());
        planning_display_->setQueryGoalState(goal);
    }

    void MotionPlanningFrame::setStartToGoalButtonClicked()
    {
        robot_state::RobotState start = *planning_display_->getQueryStartState();
        robot_state::RobotState goal = *planning_display_->getQueryGoalState();
        goal.setVariablePositions(start.getVariablePositions());
        planning_display_->setQueryGoalState(goal);
    }

    void MotionPlanningFrame::setGoalToStartButtonClicked()
    {
        robot_state::RobotState start = *planning_display_->getQueryStartState();
        robot_state::RobotState goal = *planning_display_->getQueryGoalState();
        start.setVariablePositions(goal.getVariablePositions());
        planning_display_->setQueryStartState(start);
    }

    void MotionPlanningFrame::padlockButtonToggled(bool checked)
    {
        // Update text to reflect locked state.
        const QString lock = QString::fromUtf8("\uF023");
        const QString unlock = QString::fromUtf8("\uF13E");
        ui_->padlock_button->setText(checked ? lock : unlock);

        // Snap start to goal on unlock.
        if (!checked)
            planning_display_->setQueryStartState(*planning_display_->getQueryGoalState());

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

    void MotionPlanningFrame::updateGroupComboBoxFromAction(const apc_msgs::PrimitiveAction& action)
    {
        planning_display_->changePlanningGroup(action.group_id);
    }

    void MotionPlanningFrame::updateFrameComboBox()
    {
        QComboBox* frame = ui_->frame_combobox;
        // Save current text.
        QString text = frame->currentText();
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
        // Restore text.
        frame->setCurrentIndex(frame->findText(text));
    }

    void MotionPlanningFrame::frameComboBoxActivated(const QString& text)
    {
        // If the frame is an object, attach that object for
        // convinience.
        std::string frame = text.toStdString();
        if (frame.empty() || frame.find("bin") == 0)
            return;
        int index = ui_->object_combobox->findText(text);
        ui_->object_combobox->setCurrentIndex(index);
    }

    void MotionPlanningFrame::updateFrameComboBoxFromAction(const apc_msgs::PrimitiveAction& action)
    {
        QComboBox* frame = ui_->frame_combobox;
        int index = frame->findText(QString::fromStdString(action.frame_id));
        if (index != -1)
            frame->setCurrentIndex(index);
    }

    void MotionPlanningFrame::updateObjectComboBox()
    {
        QComboBox* object = ui_->object_combobox;
        // Save current text.
        QString text = object->currentText();
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
        // Restore text.
        object->setCurrentIndex(object->findText(text));
    }

    void MotionPlanningFrame::objectComboBoxCurrentIndexChanged(const QString& text)
    {
        std::string object_id = text.toStdString();

        ROS_DEBUG("Object ID index changed to %s", object_id.c_str());

        // If the selected object is void, uncheck the grasp checkbox and disable it.
        if (object_id.empty()) {
            ui_->grasp_checkbox->setChecked(false);
            ui_->grasp_checkbox->setEnabled(false);
        }
        // If the selected object is not void, enable the grasp checkbox.
        else {
            ui_->grasp_checkbox->setEnabled(true);
        }
    }

    void MotionPlanningFrame::updateObjectComboBoxFromAction(const apc_msgs::PrimitiveAction& action)
    {
        QComboBox* object = ui_->object_combobox;
        int index = object->findText(QString::fromStdString(action.object_id));
        if (index != -1)
            object->setCurrentIndex(index);
    }

    void MotionPlanningFrame::updateOptionsCheckBoxesFromAction(const apc_msgs::PrimitiveAction& action)
    {
        ui_->monitor_contact_checkbox->setChecked(action.monitor_contact);
        ui_->monitor_profile_checkbox->setChecked(action.monitor_haptic_profile);
        ui_->interpolate_cartesian_checkbox->setChecked(action.interpolate_cartesian);
        ui_->grasp_checkbox->setChecked(action.grasp);
    }

    void MotionPlanningFrame::updateLockedStateFromAction(const apc_msgs::PrimitiveAction& action)
    {
        ui_->padlock_button->setChecked(action.eef_locked);
    }

    void MotionPlanningFrame::graspCheckBoxToggled(bool grasp) {
        // If grasp is disabled, remove any attached objects from the robot state.
        if (!grasp) {
            robot_state::RobotState start_state = *planning_display_->getQueryStartState();
            robot_state::RobotState goal_state = *planning_display_->getQueryGoalState();
            start_state.clearAttachedBodies();
            goal_state.clearAttachedBodies();
            planning_display_->setQueryStartState(start_state);
            planning_display_->setQueryGoalState(goal_state);
            // Enable object combobox when grasp is unchecked.
            ui_->object_combobox->setEnabled(true);
        }
        // If grasp is enabled, attach the object to the robot state.
        else {
            std::string object_id = ui_->object_combobox->currentText().toStdString();
            robot_state::RobotState start_state = *getQueryStartState();
            robot_state::RobotState end_state = *getQueryGoalState();
            KeyPoseMap world_state = this->computeWorldKeyPoseMap();
            std::string group_id = planning_display_->getCurrentPlanningGroup();
            try {
                computeAttachNearestObjectToStateMatchingId(object_id, group_id, world_state, start_state);
                computeAttachNearestObjectToStateMatchingId(object_id, group_id, world_state, end_state);
            } catch (std::exception& error) {
                ROS_ERROR("Caught exception in %s", error.what());
            }
            planning_display_->setQueryStartState(start_state);
            planning_display_->setQueryGoalState(end_state);
            // Disable object combobox when grasp is checked.
            ui_->object_combobox->setEnabled(false);
        }
    }



}
