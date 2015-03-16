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

    QString toDebug(const QByteArray & line) {

        QString s;
        uchar c;

        for ( int i=0 ; i < line.size() ; i++ ){
            c = line[i];
            if ( c >= 0x20 and c <= 126 ) {
                s.append(c);
            } else {
                s.append(QString("<%1>").arg(c, 2, 16, QChar('0')));
            }
        }
        return s;
    }

    void MotionPlanningFrame::getStateFromAction(robot_state::RobotState& robot,
                                                 const apc_msgs::PrimitiveAction& action)
    {
        // Get the group name.
        std::string group = action.group_name;

        // Convert goal message into joint state message.
        sensor_msgs::JointState state;
        state.name = action.joint_trajectory.joint_names;

        // Get the joint constraints. Note we only extract the first one!
        state.position = action.joint_trajectory.points[0].positions;

        // Merge the goal message into the robot state. This keeps the previous values.
        robot.setVariableValues(state);

        // Update the dirty transforms, etc.
        robot.update();
    }

    void MotionPlanningFrame::saveGoalToItem(QListWidgetItem* item)
    {
        const robot_state::RobotState& state = *planning_display_->getQueryGoalState();
        saveGoalToItem(state, item);
    }

    void MotionPlanningFrame::saveGoalToItem(const robot_state::RobotState& state,
                                             QListWidgetItem* item)
    {
        if (!item)
            return;

        // TODO Save the end-effector relative poses and the joint angles.

        // Get current planning group (set of active joints).
        std::string group = planning_display_->getCurrentPlanningGroup();

        // Get the active joint models in that group.
        const moveit::core::JointModelGroup* joint_model_group = state.getJointModelGroup(group);

        // Get the goal joint tolerance.
        double goal_joint_tolerance = move_group_->getGoalJointTolerance();

        // Create a move group goal message.
        apc_msgs::PrimitivePlan goal;
        goal.actions.resize(1);

        // Set the goal group name.
        goal.actions[0].group_name = group;

        // Construct goal constraints only for the joint model group.
        moveit_msgs::Constraints constraints = kinematic_constraints::constructGoalConstraints(state,
                                                                                               joint_model_group,
                                                                                               goal_joint_tolerance);

        // Copy constraints to goal.
        goal.actions[0].joint_trajectory.points.resize(1);
        for (int i = 0; i < constraints.joint_constraints.size(); i++)
        {
            goal.actions[0].joint_trajectory.joint_names.push_back(constraints.joint_constraints[i].joint_name);
            goal.actions[0].joint_trajectory.points[0].positions.push_back(constraints.joint_constraints[i].position);
        }

        // Save attached objects to the plan.
        computeAttachObjectToPlan(goal, state);

        // TODO Build a descriptive name.
        static int count = 0;
        QString display_name = QString("%1: %2").arg(group.c_str()).arg(count++);

        goal.actions[0].action_name = display_name.toStdString();

        // Get the data in the item.
        QVariant data = item->data(Qt::UserRole);

        // Store the goal into the data.
        setMessageToUserData<apc_msgs::PrimitivePlan>(data, goal);

        // Save the serialized data back into the item.
        item->setData(Qt::UserRole, data);

        // Set the display name of the item.
        item->setText(display_name);
    }

    void MotionPlanningFrame::loadGoalFromItem(QListWidgetItem* item)
    {
        if (!item)
            return;

        // Get the saved binary data.
        QVariant data = item->data(Qt::UserRole);

        // Load goal from data.
        loadGoalFromData(data);
    }

    void MotionPlanningFrame::loadGoalFromItem(QTreeWidgetItem* item)
    {
        if (!item)
            return;

        // Get the saved binary data.
        QVariant data = item->data(0, Qt::UserRole);

        // Load goal from data.
        loadGoalFromData(data);
    }

    void MotionPlanningFrame::loadGoalFromData(const QVariant& data)
    {
        // Deserialize byte array into move group goal.
        apc_msgs::PrimitivePlan goal = getMessageFromUserData<apc_msgs::PrimitivePlan>(data);

        // Get the new planning group.
        std::string group = goal.actions[0].group_name;

        // Update the planning group.
        planning_display_->changePlanningGroup(group);

        // Get the current goal state.
        robot_state::RobotState current_state = *planning_display_->getQueryGoalState();

        // Get the saved joint constraints.
        const trajectory_msgs::JointTrajectory& joint_trajectory = goal.actions[0].joint_trajectory;

        // Copy the joints in the goal to the current state.
        for (int i = 0; i < joint_trajectory.joint_names.size(); i++)
        {
            current_state.setJointPositions(joint_trajectory.joint_names[i],
                                            &joint_trajectory.points[0].positions[i]);
        }

        // Add attached objects if they exist.
        computeAttachObjectToState(current_state,
                                   goal.actions[0].object_name,
                                   goal.actions[0].link_name,
                                   goal.actions[0].object_poses);

        // Set the joints related to the current group.
        planning_display_->setQueryGoalState(current_state);
    }

    void MotionPlanningFrame::loadWaypointsToDisplay(QList<QListWidgetItem*> items)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            data.push_back(items[i]->data(Qt::UserRole));

        loadWaypointsToDisplay(data);
    }

    void MotionPlanningFrame::loadWaypointsToDisplay(QList<QTreeWidgetItem*> items)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            if (items[i]->childCount() > 0)
                for (int j = 0; j < items[i]->childCount(); j++)
                    data.push_back(items[i]->child(j)->data(0, Qt::UserRole));
            else
                data.push_back(items[i]->data(0, Qt::UserRole));

        loadWaypointsToDisplay(data);
    }

    void MotionPlanningFrame::loadWaypointsToDisplay(std::vector<QVariant>& data)
    {
        // Delete all existing waypoints!
        planning_display_->clearDisplayWaypoints();

        // TODO Construct the group name.
        std::string group;

        // TODO Construct the link names.
        std::vector<std::string> link_names;

        // TODO Set the focus.
        int focus = 0;

        // Construct a list of new waypoints.
        robot_state::RobotState waypoint(planning_display_->getPlanningSceneRO()->getCurrentState());
        for (int i = 0; i < data.size(); i++)
        {
            // Get the ith goal message in the data vector.
            apc_msgs::PrimitivePlan goal = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[i]);

            // Get the group name.
            group = goal.actions[0].group_name;

            // Get the joint trajectory. Note we only extract the first one!
            const trajectory_msgs::JointTrajectory& joint_trajectory = goal.actions[0].joint_trajectory;

            // Convert goal message into joint state message.
            sensor_msgs::JointState state;
            for (int j = 0; j < joint_trajectory.joint_names.size(); j++)
            {
                state.name.push_back(joint_trajectory.joint_names[j]);
                state.position.push_back(joint_trajectory.points[0].positions[j]);
            }

            // Attach object to state, if applicable.
            computeAttachObjectToState(waypoint,
                                       goal.actions[0].object_name,
                                       goal.actions[0].link_name,
                                       goal.actions[0].object_poses);

            // Merge the goal message into the robot state. This keeps the previous values.
            waypoint.setVariableValues(state);

            // Update the dirty transforms, etc.
            waypoint.update();

            // Add waypoint.
            planning_display_->addDisplayWaypoint(waypoint, group, link_names, focus);
        }
    }

    void MotionPlanningFrame::loadOptionsToView(QList<QListWidgetItem*> items, bool enable)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            data.push_back(items[i]->data(Qt::UserRole));
        loadOptionsToView(data, enable);
    }

    void MotionPlanningFrame::loadOptionsToView(QList<QTreeWidgetItem*> items, bool enable)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            if (items[i]->childCount() > 0)
                for (int j = 0; j < items[i]->childCount(); j++)
                    data.push_back(items[i]->child(j)->data(0, Qt::UserRole));
            else
                data.push_back(items[i]->data(0, Qt::UserRole));
        loadOptionsToView(data, enable);
    }

    void MotionPlanningFrame::loadOptionsToView(std::vector<QVariant>& data, bool enable)
    {
        // The number of checkboxes.
        const int num_checkboxes = 7;

        // Array of checkbox pointers.
        class QCheckBox* checkbox[num_checkboxes] = { ui_->relative_to_object_checkbox,
                                                      ui_->relative_to_pose_checkbox,
                                                      ui_->eef_trajectory_checkbox,
                                                      ui_->dense_trajectory_checkbox,
                                                      ui_->monitor_contact_checkbox,
                                                      ui_->monitor_profile_checkbox,
                                                      ui_->cartesian_interpolate_checkbox };

        // Clear and disable all the options.
        for (int i = 0; i < num_checkboxes; i++)
        {
            checkbox[i]->setChecked(false);
            checkbox[i]->setEnabled(false);
            checkbox[i]->setTristate(false);
        }

        // Exit if there's nothing to display.
        if (data.size() == 0)
            return;

        // Set all options to enabled or disabled.
        for (int i = 0; i < num_checkboxes; i++)
        {
            checkbox[i]->setEnabled(enable);
        }

        // Copy the options over to view.
        bool init = false;
        for (int i = 0; i < data.size(); i++)
        {
            // Get the thing out of the data.
            apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[i]);
            for (int j = 0; j < plan.actions.size(); j++)
            {
                const apc_msgs::PrimitiveAction& action = plan.actions[j];
                unsigned char options[num_checkboxes] = { action.relative_to_object,
                                                          action.relative_to_previous_pose,
                                                          action.use_eef_trajectory,
                                                          action.dense_trajectory,
                                                          action.stop_on_contact,
                                                          action.use_haptic_profile,
                                                          action.interpolate_cartesian };
                // Set the tristate of the checkbox.
                for (int k = 0; k < num_checkboxes; k++)
                    setTristateCheckBox(checkbox[k], options[k], init);
                init = true;
            }
        }
    }

    void MotionPlanningFrame::setTristateCheckBox(QCheckBox* checkbox, bool b, bool init)
    {
        if (!init)
            checkbox->setChecked(b);
        else if ((int) checkbox->checkState() != 2 * (int) b)
        {
            checkbox->setTristate();
            checkbox->setChecked(Qt::PartiallyChecked);
            checkbox->setEnabled(false);
        }
        checkbox->update();
    }

    void MotionPlanningFrame::saveOptionsFromView(QList<QListWidgetItem*> items)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            data.push_back(items[i]->data(Qt::UserRole));

        saveOptionsFromView(data);

        for (int i = 0; i < items.count(); i++)
            items[i]->setData(Qt::UserRole, data[i]);
    }

    void MotionPlanningFrame::saveOptionsFromView(std::vector<QVariant>& data)
    {
        if (!ui_->relative_to_object_checkbox->isEnabled())
            return;

        for (int i = 0; i < data.size(); i++)
        {
            // Get message from data.
            apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[i]);

            // Fill message options.
            for (int j = 0; j < plan.actions.size(); j++)
            {
                apc_msgs::PrimitiveAction& action = plan.actions[i];
                QCheckBox* checkbox[7] = { ui_->relative_to_object_checkbox,
                                           ui_->relative_to_pose_checkbox,
                                           ui_->eef_trajectory_checkbox,
                                           ui_->dense_trajectory_checkbox,
                                           ui_->monitor_contact_checkbox,
                                           ui_->monitor_profile_checkbox,
                                           ui_->cartesian_interpolate_checkbox };
                unsigned char* options[7] = { &action.relative_to_object,
                                              &action.relative_to_previous_pose,
                                              &action.use_eef_trajectory,
                                              &action.dense_trajectory,
                                              &action.stop_on_contact,
                                              &action.use_haptic_profile,
                                              &action.interpolate_cartesian };
                for (int k = 0; k < 7; k++)
                    if (checkbox[k]->checkState() == Qt::PartiallyChecked)
                        ROS_WARN("Skipping partially checked checkbox: %s", checkbox[k]->text().toStdString().c_str());
                    else
                        *options[k] = checkbox[k]->isChecked();
            }

            // Set message back into user data!
            setMessageToUserData<apc_msgs::PrimitivePlan>(data[i], plan);
        }
    }

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
