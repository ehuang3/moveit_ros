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
#include <QtGui/QMessageBox>


namespace moveit_rviz_plugin
{

    void MotionPlanningFrame::computeSavePlansButtonClicked()
    {
        if (primitive_plan_storage_)
        {
            // The stored plans in the view.
            QTreeWidget* stored_plans = ui_->stored_plans_tree;

            // A primitive plan message.
            apc_msgs::PrimitivePlan msg;

            // List of plans in the database.
            std::vector<std::string> database_names;
            primitive_plan_storage_->getKnownPrimitivePlans(database_names);

            // Drop all old plans from the database. FIXME This might be very inefficient?
            for (int i = 0; i < database_names.size(); i++)
                try
                {
                    primitive_plan_storage_->removePrimitivePlan(database_names[i]);
                }
                catch (std::runtime_error &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }

            // Construct primitive plans.
            for (int i = 0; i < stored_plans->topLevelItemCount(); i++)
            {
                // Get the toplevel item.
                QTreeWidgetItem* root = stored_plans->topLevelItem(i);

                // Build primitive plan message.
                msg.plan_name = root->text(0).toStdString(); // TODO Unique plan names.
                msg.actions.clear();
                for (int j = 0; j < root->childCount(); j++)
                {
                    // Get the child.
                    QTreeWidgetItem* child = root->child(j);

                    // Get the message stored in the child.
                    apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(child->data(0, Qt::UserRole));

                    // Append primitive actions in child plan to primitive plan.
                    for (int k = 0; k < plan.actions.size(); k++)
                        msg.actions.push_back(plan.actions[k]);
                }

                // Store message in database.
                try
                {
                    primitive_plan_storage_->addPrimitivePlan(msg, msg.plan_name);
                }
                catch (std::runtime_error &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }
            }
        }
    }

    void MotionPlanningFrame::computeLoadPlansButtonClicked()
    {
        if (!primitive_plan_storage_)
            return;

        // Names of the stored plans.
        std::vector<std::string> plan_names;

        // Get all the stored plans.
        try
        {
            primitive_plan_storage_->getKnownPrimitivePlans(plan_names);
        }
        catch (std::runtime_error &ex)
        {
            QMessageBox::warning(this, "Cannot query the database", QString("Error: ").append(ex.what()));
            return;
        }

        // Clear the current stored plan tree.
        ui_->stored_plans_tree->clear();

        // Add plans to the tree!
        for (int i = 0; i < plan_names.size(); i++)
        {
            moveit_warehouse::PrimitivePlanWithMetadata plan;
            bool got_plan = false;
            try
            {
                got_plan = primitive_plan_storage_->getPrimitivePlan(plan, plan_names[i]);
            }
            catch(std::runtime_error &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
            if (!got_plan)
                continue;

            // Create tree widget item to hold the plan.
            QTreeWidgetItem* root = new QTreeWidgetItem;
            root->setFlags(root->flags() | Qt::ItemIsEditable);

            root->setText(0, QString(plan->plan_name.c_str()));

            for (int j = 0; j < plan->actions.size(); j++)
            {
                QTreeWidgetItem* child = new QTreeWidgetItem;
                child->setFlags(child->flags() | Qt::ItemIsEditable);

                // Set the child's name.
                child->setText(0, QString(plan->actions[j].action_name.c_str()));

                // Construct a plan to hold the action.
                apc_msgs::PrimitivePlan child_plan;
                child_plan.actions.push_back(plan->actions[j]);

                // Store primitive action in the data.
                QVariant data;
                setMessageToUserData<apc_msgs::PrimitivePlan>(data, child_plan);

                // Store data into the child node.
                child->setData(0, Qt::UserRole, data);

                // Append to root.
                root->addChild(child);
            }

            ui_->stored_plans_tree->addTopLevelItem(root);
        }
    }

}
