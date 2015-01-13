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

    void MotionPlanningFrame::insertGoalButtonClicked()
    {
        // Get the list of active goals (waypoints).
        QListWidget* active_goals_list = ui_->active_goals_list;

        // Save the current goal to a new item.
        QListWidgetItem* item = new QListWidgetItem;
        saveGoalToItem(item);

        // Get the currently selected row.
        int row = active_goals_list->currentRow();

        // Unselect item.
        if (active_goals_list->currentItem())
            active_goals_list->currentItem()->setSelected(false);

        // Insert the item into the list.
        active_goals_list->insertItem(row + 1, item);

        // Set item as active.
        active_goals_list->setCurrentItem(item);

        // Set the current options into the goal.
        QList<QListWidgetItem*> opt;
        opt.append(item);
        saveOptionsFromView(opt);

        // Load all goals into waypoint display.
        QList<QListWidgetItem*> goals = ui_->active_goals_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(goals);
    }

    void MotionPlanningFrame::deleteGoalButtonClicked()
    {
        // Get the list of active goals (waypoints).
        QListWidget* active_goals_list = ui_->active_goals_list;

        // If there are no items to delete, return.
        if (active_goals_list->count() == 0)
            return;

        // Take and delete item.
        int row = active_goals_list->currentRow();
        QListWidgetItem* item = active_goals_list->takeItem(row);
        if (item)
            delete item;

        // Set the active goal to the next in line if available or else previous.
        if (row == active_goals_list->count())
            active_goals_list->setCurrentRow(std::max(row - 1, 0));

        // Load all goals into waypoint display.
        QList<QListWidgetItem*> goals = ui_->active_goals_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(goals);
    }

    void MotionPlanningFrame::activeGoalSelectionChanged()
    {
        // HACK Because QWidget does not provide a 'clicked' signal, we check here for the no selection case.
        if (ui_->active_goals_list->selectedItems().empty())
            activeGoalListClicked(QModelIndex());
    }

    void MotionPlanningFrame::activeGoalListClicked(const QModelIndex& index)
    {
        // Load the options from all items, but make it non-editable.
        QList<QListWidgetItem*> options = ui_->active_goals_list->findItems(".*", Qt::MatchRegExp);
        loadOptionsToView(options, false);

        // Load all goals into waypoint display.
        QList<QListWidgetItem*> goals = ui_->active_goals_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(goals);
    }

    void MotionPlanningFrame::activeGoalItemClicked(QListWidgetItem* item)
    {
        // Load the options from selected items.
        QList<QListWidgetItem*> options = ui_->active_goals_list->selectedItems();
        loadOptionsToView(options, true);

        // Load the selected items into waypoint display.
        QList<QListWidgetItem*> goals = ui_->active_goals_list->selectedItems();
        loadWaypointsToDisplay(goals);
    }

    void MotionPlanningFrame::activeGoalItemDoubleClicked(QListWidgetItem* item)
    {
        // Load the currently selected item into the goal query state.
        loadGoalFromItem(item);

        // Load the options from double clicked items.
        QList<QListWidgetItem*> option;
        option.append(item);
        loadOptionsToView(option, true);
    }

    void MotionPlanningFrame::storedPlanTreeClicked(const QModelIndex& index)
    {
        // Load the options from selected items, but make them non-editable.
        QList<QTreeWidgetItem*> options = ui_->stored_plans_tree->selectedItems();
        loadOptionsToView(options, false);

        // Load selected stored tree plans into waypoint display.
        QList<QTreeWidgetItem*> goals = ui_->stored_plans_tree->selectedItems();
        loadWaypointsToDisplay(goals);
    }

    void MotionPlanningFrame::storedPlanItemClicked(QTreeWidgetItem* item, int col)
    {
        // Load the options from selected items.
        QList<QTreeWidgetItem*> options = ui_->stored_plans_tree->selectedItems();
        loadOptionsToView(options, false);

        // Load the selected items into waypoint display.
        QList<QTreeWidgetItem*> goals = ui_->stored_plans_tree->selectedItems();
        loadWaypointsToDisplay(goals);
    }

    void MotionPlanningFrame::storedPlanItemDoubleClicked(QTreeWidgetItem* item, int col)
    {
        // Load robot into goal state.
        loadGoalFromItem(item);

        // Do nothing special.
        storedPlanItemClicked(item, col);
    }

    void MotionPlanningFrame::activeToStoredPlansButtonClicked()
    {
        // Get the list of active goals (waypoints).
        QListWidget* active_goals = ui_->active_goals_list;

        // Do nothing if there are no items.
        if (active_goals->count() == 0)
            return;

        // Get all hightlighted items, or all items if none are highlighted.
        QList<QListWidgetItem*> items = active_goals->selectedItems();
        if (items.count() == 0)
            for (int i = 0; i < active_goals->count(); i++)
                items.push_back(active_goals->item(i));

        // Construct a top level tree item.
        QTreeWidgetItem* root = new QTreeWidgetItem;
        for (int i = 0; i < items.count(); i++)
        {
            QTreeWidgetItem* child = new QTreeWidgetItem;
            child->setData(0, Qt::UserRole, items[i]->data(Qt::UserRole));
            child->setText(0, items[i]->text());
            root->addChild(child);
        }

        // Tree of stored plans.
        QTreeWidget* stored_plans = ui_->stored_plans_tree;

        // Build a descriptive name.
        QString name = QString("plan: %1").arg(stored_plans->topLevelItemCount());

        // Set plan name to display.
        root->setText(0, name);

        // Add to tree list.
        stored_plans->addTopLevelItem(root);
    }

    void MotionPlanningFrame::storedToActiveGoalsButtonClicked()
    {
        // Tree of stored plans.
        QTreeWidget* stored_plans = ui_->stored_plans_tree;

        // The selected items.
        QList<QTreeWidgetItem*> items = stored_plans->selectedItems();

        // If no items are selected, do nothing.
        if (items.empty())
            return;

        // Check to see if the list has toplevel items and children.
        bool mixed = false;
        bool toplevel = false;
        for (int i = 0; i < items.count(); i++)
            if (i == 0)
                toplevel = (-1 != stored_plans->indexOfTopLevelItem(items[i]));
            else if (-1 == stored_plans->indexOfTopLevelItem(items[i]) && !mixed && !toplevel)
                mixed = true;
            else if (!mixed && toplevel)
                mixed = true;

        // List of active goals.
        QListWidget* active_goals = ui_->active_goals_list;

        // Active goal's currently selected row or else the last row.
        int row = active_goals->currentRow() == -1 ? active_goals->count()-1 : active_goals->currentRow();

        // For loop to copy over the selected items.
        for (int i = 0; i < items.count(); i++)
        {
            // Create list of data.
            std::vector<QVariant> data;

            // If the list is not mixed and the item is toplevel, add it's children's data.
            if (!mixed && toplevel)
                for (int j = 0; j < items[i]->childCount(); j++)
                    data.push_back(items[i]->child(j)->data(0, Qt::UserRole));
            // Else if the list is mixed and items are not toplevel, add it's data.
            else if (-1 == stored_plans->indexOfTopLevelItem(items[i]))
                data.push_back(items[i]->data(0, Qt::UserRole));

            // Copy data over to goals list.
            for (int j = 0; j < data.size(); j++)
            {
                // Create a new list item.
                QListWidgetItem* item = new QListWidgetItem;

                // Get the plan message from the stored data.
                apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[j]);

                // Get the display name from the plan.
                QString text = plan.actions[0].action_name.c_str();

                // Set the into the new item.
                item->setData(Qt::UserRole, data[j]);

                // Set the display name into the item.
                item->setText(text);

                // Insert item into the goals list.
                active_goals->insertItem(++row, item);
            }
        }

        // Load all goals into waypoint display.
        QList<QListWidgetItem*> goals = ui_->active_goals_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(goals);
    }

    void MotionPlanningFrame::savePlansButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSavePlansButtonClicked, this), "save plans");
    }

    void MotionPlanningFrame::loadPlansButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeLoadPlansButtonClicked, this), "load plans");
    }

    void MotionPlanningFrame::deleteStoredPlanButtonClicked()
    {
        // Get the stored plans.
        QTreeWidget* stored_plans = ui_->stored_plans_tree;

        // If there are no plans, do nothing.
        if (stored_plans->topLevelItemCount() == 0)
            return;

        // If the selected item is a toplevel item, remove the plan.
        QTreeWidgetItem* current_item  = stored_plans->currentItem();
        int              index         = stored_plans->indexOfTopLevelItem(current_item);
        QTreeWidgetItem* toplevel_item = stored_plans->topLevelItem(index);
        if (current_item == toplevel_item)
            delete stored_plans->takeTopLevelItem(index);
    }

    void MotionPlanningFrame::optionsCheckBoxClicked()
    {
        // Save the clicked options.
        QList<QListWidgetItem*> items = ui_->active_goals_list->selectedItems();
        saveOptionsFromView(items);
    }

}
