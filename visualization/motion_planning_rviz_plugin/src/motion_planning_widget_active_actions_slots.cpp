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
    void MotionPlanningFrame::insertGoalButtonClicked()
    {
        // Get the list of active goals (waypoints).
        QListWidget* active_goals_list = ui_->active_goals_list;

        // Save the current goal to a new item.
        QListWidgetItem* item = new QListWidgetItem;
        item->setFlags(item->flags() | Qt::ItemIsEditable);
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
}
