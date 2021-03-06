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
#include <apc/planning.h>


namespace moveit_rviz_plugin
{

    void MotionPlanningFrame::computeSavePlansButtonClicked()
    {
        if (primitive_plan_storage_)
        {
            // The stored plans in the view.
            QTreeWidget* stored_plans = ui_->stored_plans_tree;

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

                // Get the primitive plan.
                apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(root->data(0, Qt::UserRole));

                // Build primitive plan message.
                plan.plan_name = root->text(0).toStdString(); // TODO Unique plan names.

                // Store message in database.
                try
                {
                    primitive_plan_storage_->addPrimitivePlan(plan, plan.plan_name);
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

            // Store overall plan in root node.
            QVariant data;
            setMessageToUserData<apc_msgs::PrimitivePlan>(data, *plan);
            root->setData(0, Qt::UserRole, data);

            for (int j = 0; j < plan->actions.size(); j++)
            {
                QTreeWidgetItem* child = new QTreeWidgetItem;
                child->setFlags(child->flags() | Qt::ItemIsEditable);

                // Set the child's name.
                child->setText(0, QString(plan->actions[j].action_name.c_str()));

                // Store primitive action in the data.
                setMessageToUserData<apc_msgs::PrimitiveAction>(data, plan->actions[j]);

                // Store data into the child node.
                child->setData(0, Qt::UserRole, data);

                // Append to root.
                root->addChild(child);
            }

            ui_->stored_plans_tree->addTopLevelItem(root);
        }
    }

    void MotionPlanningFrame::setStoredPlansList(const std::vector<apc_msgs::PrimitivePlan>& plans)
    {
        ui_->stored_plans_tree->clear();

        for (int i = 0; i < plans.size(); i++)
        {
            const apc_msgs::PrimitivePlan& plan = plans[i];

            // Create tree widget item to hold the plan.
            QTreeWidgetItem* root = new QTreeWidgetItem;
            root->setFlags(root->flags() | Qt::ItemIsEditable);

            root->setText(0, QString(plan.plan_name.c_str()));

            // Store overall plan in root node.
            QVariant data;
            setMessageToUserData<apc_msgs::PrimitivePlan>(data, plan);
            root->setData(0, Qt::UserRole, data);

            for (int j = 0; j < plan.actions.size(); j++)
            {
                QTreeWidgetItem* child = new QTreeWidgetItem;
                child->setFlags(child->flags() | Qt::ItemIsEditable);

                // Set the child's name.
                child->setText(0, QString(plan.actions[j].action_name.c_str()));

                // Store primitive action in the data.
                setMessageToUserData<apc_msgs::PrimitiveAction>(data, plan.actions[j]);

                // Store data into the child node.
                child->setData(0, Qt::UserRole, data);

                // Append to root.
                root->addChild(child);
            }

            ui_->stored_plans_tree->addTopLevelItem(root);
        }
    }

    void MotionPlanningFrame::getStoredPlansList(std::vector<apc_msgs::PrimitivePlan>& plans)
    {
        plans.clear();
        QTreeWidget* stored_plans = ui_->stored_plans_tree;
        for (int i = 0; i < stored_plans->topLevelItemCount(); i++) {
            apc_msgs::PrimitivePlan plan =
                getMessageFromUserData<apc_msgs::PrimitivePlan>(stored_plans->
                                                                topLevelItem(i)->data(0, Qt::UserRole));
            plans.push_back(plan);
        }
    }

    void MotionPlanningFrame::overwriteStoredPlans(const std::vector<apc_msgs::PrimitivePlan>& write_plans)
    {
        // Sort the overwriting plans.
        typedef std::vector<apc_msgs::PrimitivePlan> PlanList;
        PlanList write = write_plans;
        std::sort(write.begin(), write.end(), apc_planning::less_than_plan_name());
        // Get stored plans list.
        PlanList stored;
        getStoredPlansList(stored);
        std::sort(stored.begin(), stored.end(), apc_planning::less_than_plan_name());
        // replace the plans with the same names.
        int i = 0, j = 0;
        while (i < write.size() && j < stored.size()) {
            std::string ni = write[i].plan_name;
            std::string nj = stored[j].plan_name;
            if (ni < nj)
                i++;
            if (ni == nj)
                stored[j++] = write[i++];
            if (ni > nj)
                j++;
        }
        // write to plan list.
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::setStoredPlansList, this, stored));
    }

}
