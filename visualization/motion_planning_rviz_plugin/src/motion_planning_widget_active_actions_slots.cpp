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

    void MotionPlanningFrame::connectActiveActionsSlots()
    {
        connect( ui_->insert_action_button,  SIGNAL( clicked() ), this, SLOT( insertActionButtonClicked() ));
        connect( ui_->delete_action_button,  SIGNAL( clicked() ), this, SLOT( deleteActionButtonClicked() ));
        connect( ui_->replace_action_button, SIGNAL( clicked() ), this, SLOT( replaceActionButtonClicked() ));
        connect( ui_->rename_actions_button, SIGNAL( clicked() ), this, SLOT( renameAllActionsButtonClicked() ));
        connect( ui_->active_actions_list,  SIGNAL( itemSelectionChanged() ),
                 this, SLOT( activeActionsSelectionChanged() ));
        connect( ui_->active_actions_list,  SIGNAL( itemClicked(QListWidgetItem*) ),
                 this, SLOT( activeActionsItemClicked(QListWidgetItem*) ));
        connect( ui_->active_actions_list,  SIGNAL( itemDoubleClicked(QListWidgetItem*) ),
                 this, SLOT( activeActionsItemDoubleClicked(QListWidgetItem*) ));
    }

    void MotionPlanningFrame::insertActionButtonClicked()
    {
        if (!planning_display_->getPlanningSceneMonitor())
        {
            ROS_ERROR("Error: No query start or goal state to save");
            return;
        }

        // Get the list of active actions (waypoints).
        QListWidget* active_actions_list = ui_->active_actions_list;

        // Save the current action to a new item.
        QListWidgetItem* item = new QListWidgetItem;
        item->setFlags(item->flags() | Qt::ItemIsEditable);

        // Save state and settings to action.
        apc_msgs::PrimitiveAction action;
        saveStartAndGoalToAction(action);
        saveFormatToAction(action);
        saveFrameToAction(action);
        saveObjectToAction(action);
        saveOptionsToAction(action);

        std::cout << "==================== Action ====================\n"
                  << action
                  << "================================================\n";

        // Store action in variant.
        QVariant data;
        saveActionToData(action, data);
        item->setData(Qt::UserRole, data);

        // Set item name to action name.
        item->setText(QString::fromStdString(action.action_name));

        // Get the currently selected row.
        int row = active_actions_list->currentRow();

        // Unselect item.
        if (active_actions_list->currentItem())
            active_actions_list->currentItem()->setSelected(false);

        // Insert the item into the list.
        active_actions_list->insertItem(row + 1, item);

        // Set item as active.
        active_actions_list->setCurrentItem(item);

        // Load all actions into waypoint display.
        QList<QListWidgetItem*> actions = ui_->active_actions_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::deleteActionButtonClicked()
    {
        // Get the list of active actions (waypoints).
        QListWidget* active_actions_list = ui_->active_actions_list;

        // If there are no items to delete, return.
        if (active_actions_list->count() == 0)
            return;

        // Take and delete item.
        int row = active_actions_list->currentRow();
        QListWidgetItem* item = active_actions_list->takeItem(row);
        if (item)
            delete item;

        // Set the active action to the next in line if available or else previous.
        if (row == active_actions_list->count())
            active_actions_list->setCurrentRow(std::max(row - 1, 0));

        // Load all actions into waypoint display.
        QList<QListWidgetItem*> actions = ui_->active_actions_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::replaceActionButtonClicked()
    {
        ROS_WARN("replaceActionButtonClicked");
    }

    void MotionPlanningFrame::renameAllActionsButtonClicked()
    {
        ROS_WARN("renameAllActionsButtonClicked");
    }

    void MotionPlanningFrame::activeActionsSelectionChanged()
    {
        // HACK Because QWidget does not provide a 'clicked' signal, we check here for the no selection case.
        if (ui_->active_actions_list->selectedItems().empty())
            activeActionsListClicked(QModelIndex());
    }

    void MotionPlanningFrame::activeActionsListClicked(const QModelIndex& index)
    {
        // Load all actions into waypoint display.
        QList<QListWidgetItem*> actions = ui_->active_actions_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::activeActionsItemClicked(QListWidgetItem* item)
    {
        // Load the selected items into waypoint display.
        QList<QListWidgetItem*> actions = ui_->active_actions_list->selectedItems();
        loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::activeActionsItemDoubleClicked(QListWidgetItem* item)
    {
        apc_msgs::PrimitiveAction action;
        loadActionFromData(action, item->data(Qt::UserRole));

        // Load the currently selected item into the query start and goal state.
        loadStartAndGoalFromAction(action);

        // Load the action into the GUI.
        updateGroupComboBoxFromAction(action);
        updateFrameComboBoxFromAction(action);
        updateObjectComboBoxFromAction(action);
        updateOptionsCheckBoxesFromAction(action);
    }
}
