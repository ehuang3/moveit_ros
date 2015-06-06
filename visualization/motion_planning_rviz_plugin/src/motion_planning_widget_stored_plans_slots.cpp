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
#include <moveit/warehouse/primitive_plan_storage.h>
#include <apc/exception.h>


namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::connectStoredPlansSlots()
    {
        connect( ui_->active_to_stored_button, SIGNAL( clicked() ), this, SLOT( activeToStoredButtonClicked() ));
        connect( ui_->stored_to_active_button, SIGNAL( clicked() ), this, SLOT( storedToActiveButtonClicked() ));

        // connect( ui_->stored_plans_tree, SIGNAL( clicked(const QModelIndex&) ), this,
        //          SLOT( storedPlansTreeClicked(const QModelIndex&) ));
        connect( ui_->stored_plans_tree, SIGNAL( itemClicked(QTreeWidgetItem*, int) ), this,
                 SLOT( storedPlansItemClicked(QTreeWidgetItem*, int) ));
        connect( ui_->stored_plans_tree, SIGNAL( itemDoubleClicked(QTreeWidgetItem*, int) ), this,
                 SLOT( storedPlansItemDoubleClicked(QTreeWidgetItem*, int) ));
        connect( ui_->plan_database_name_combobox, SIGNAL( currentIndexChanged(const QString&) ), this,
                 SLOT( storedPlansDatabaseNameChanged(const QString&) ));

        connect( ui_->save_plans_button, SIGNAL( clicked() ), this, SLOT( savePlansButtonClicked() ));
        connect( ui_->load_plans_button, SIGNAL( clicked() ), this, SLOT( loadPlansButtonClicked() ));
        connect( ui_->delete_plan_button, SIGNAL( clicked() ), this, SLOT( deletePlanButtonClicked() ));

        connect( planning_display_, SIGNAL( planningSceneDisplayLoaded() ), this, SLOT( databaseConnectButtonClicked() ) );
    }

    void MotionPlanningFrame::storedPlansTreeClicked(const QModelIndex& index)
    {
        // Load selected stored tree plans into waypoint display.
        // QList<QTreeWidgetItem*> actions = ui_->stored_plans_tree->selectedItems();
        // loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::storedPlansItemClicked(QTreeWidgetItem* item, int col)
    {
        // Load the selected items into waypoint display.
        QList<QTreeWidgetItem*> actions = ui_->stored_plans_tree->selectedItems();
        loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::storedPlansItemDoubleClicked(QTreeWidgetItem* item, int col)
    {
        // Do nothing special.
        storedPlansItemClicked(item, col);
    }

    void MotionPlanningFrame::activeToStoredButtonClicked()
    {
        // Get the list of active actions (waypoints).
        QListWidget* active_actions = ui_->active_actions_list;

        // Do nothing if there are no items.
        if (active_actions->count() == 0)
            return;

        // Get all hightlighted items, or all items if none are
        // highlighted.
        QList<QListWidgetItem*> items = active_actions->selectedItems();
        if (items.count() == 0)
            for (int i = 0; i < active_actions->count(); i++)
                items.push_back(active_actions->item(i));

        // Convert items data into plan.
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            data.push_back(items[i]->data(Qt::UserRole));
        apc_msgs::PrimitivePlan plan;
        loadActionFromData(plan.actions, data);

        // Build a descriptive name.
        try {
            saveFormatToPlan(plan);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Caught error in %s", error.what());
            return;
        }

        // Store plan in data.
        QVariant plan_data;
        setMessageToUserData<apc_msgs::PrimitivePlan>(plan_data, plan);

        // Store plan in top level tree item and store individual
        // actions in child elements.
        QTreeWidgetItem* root = new QTreeWidgetItem;
        root->setFlags(root->flags() | Qt::ItemIsEditable);
        root->setData(0, Qt::UserRole, plan_data);
        root->setText(0, QString::fromStdString(plan.plan_name));
        for (int i = 0; i < items.count(); i++)
        {
            QTreeWidgetItem* child = new QTreeWidgetItem;
            child->setFlags(child->flags() | Qt::ItemIsEditable);
            child->setData(0, Qt::UserRole, items[i]->data(Qt::UserRole));
            child->setText(0, items[i]->text());
            root->addChild(child);
        }

        // Tree of stored plans.
        QTreeWidget* stored_plans = ui_->stored_plans_tree;

        // Add to tree list.
        stored_plans->addTopLevelItem(root);
    }

    void MotionPlanningFrame::storedToActiveButtonClicked()
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

        if (mixed)
            ROS_WARN("Mixed toplevel and children");

        // List of active actions.
        QListWidget* active_actions = ui_->active_actions_list;

        // Active action's currently selected row or else the last row.
        int row = active_actions->currentRow() == -1 ? active_actions->count()-1 : active_actions->currentRow();

        // For loop to copy over the selected items.
        for (int i = 0; i < items.count(); i++)
        {
            // The selected actions.
            std::vector<apc_msgs::PrimitiveAction> actions;

            // If the list is not mixed and the item is toplevel, add it's children's data.
            if (!mixed && toplevel)
                for (int j = 0; j < items[i]->childCount(); j++)
                    actions.push_back(getMessageFromUserData<apc_msgs::PrimitiveAction>(items[i]->child(j)->data(0, Qt::UserRole)));
            // Else if the list is mixed or items are not toplevel, add selected item's data.
            else if (-1 == stored_plans->indexOfTopLevelItem(items[i]))
                actions.push_back(getMessageFromUserData<apc_msgs::PrimitiveAction>(items[i]->data(0, Qt::UserRole)));

            // Copy data over to actions list.
            for (int j = 0; j < actions.size(); j++)
            {
                // Create a new list item.
                QListWidgetItem* item = new QListWidgetItem;
                item->setFlags(item->flags() | Qt::ItemIsEditable);

                // Get the display name from the plan.
                QString text = actions[j].action_name.c_str();

                // Convert action to data.
                QVariant data;
                saveActionToData(actions[j], data);

                // Set the into the new item.
                item->setData(Qt::UserRole, data);

                // Set the display name into the item.
                item->setText(text);

                // Insert item into the actions list.
                active_actions->insertItem(++row, item);
            }
        }

        // Load all actions into waypoint display.
        QList<QListWidgetItem*> actions = ui_->active_actions_list->findItems(".*", Qt::MatchRegExp);
        loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::savePlansButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSavePlansButtonClicked, this), "save plans");
    }

    void MotionPlanningFrame::loadPlansButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeLoadPlansButtonClicked, this), "load plans");
    }

    void MotionPlanningFrame::deletePlanButtonClicked()
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

    void MotionPlanningFrame::storedPlansDatabaseNameChanged(const QString& text)
    {
        if (!primitive_plan_storage_)
        {
            ROS_ERROR("No planning database loaded!");
            return;
        }

        ROS_INFO_STREAM("Loading plan database: " << text.toStdString());

        // Load the database.
        primitive_plan_storage_->setDatabaseSuffix(text.toStdString());
        primitive_plan_storage_->loadDatabase();

        // Load the stored plans into the tree view.
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeLoadPlansButtonClicked, this), "load plans");
    }

}
