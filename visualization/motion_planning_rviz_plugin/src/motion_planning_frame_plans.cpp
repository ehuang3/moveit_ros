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

template< typename Message > QByteArray MotionPlanningFrame::serializeMessage(const Message& msg)
{
    // Stored data.
    QByteArray string;

    // Get the length of serialization.
    uint32_t serial_size = ros::serialization::serializationLength(msg);

    // Construct a middle man for the serialization?
    boost::scoped_array<uint8_t> buffer(new uint8_t[serial_size]);

    // Create a serialization object.
    ros::serialization::OStream stream(buffer.get(), serial_size);

    // Serialize.
    ros::serialization::serialize(stream, msg);

    // Clear QByteArray.
    string.clear();

    // Copy to QByteArray.
    for (uint32_t i = 0; i < serial_size; i++)
    {
        char b = (char) buffer.get()[i];
        string.append(b);
    }

    return string;
}

template< typename Message > Message MotionPlanningFrame::deserializeMessage(const QByteArray& string)
{
    // Deserialized message.
    Message msg;

    // Get the length of serialization.
    uint32_t serial_size = (uint32_t) (string.size() / sizeof(char));

    // Construct a middle man for the deserialization?
    boost::scoped_array<uint8_t> buffer(new uint8_t[serial_size]);

    // Copy to buffer.
    for (uint32_t i = 0; i < serial_size; i++)
    {
        buffer.get()[i] = (uint8_t) string.at(i);
    }

    // Create a deserialization object.
    ros::serialization::IStream stream(buffer.get(), serial_size);

    // Deserialize.
    ros::serialization::Serializer<Message>::read(stream, msg);

    return msg;
}

template< typename Message > Message MotionPlanningFrame::getMessageFromUserData(const QVariant& data)
{
    // The message we are getting.
    Message msg;

    // Check if the data can be convertered.
    if (!data.canConvert(QVariant::Map))
    {
        ROS_ERROR("Failed to convert stored data to QVariantMap.");
        return msg;
    }

    QVariantMap map = data.toMap();

    // Auto-generate key for message type. May not be human readable.
    QString key = "DUMB_KEY"; // typeid(Message).name();

    if (!map.contains(key))
    {
        ROS_ERROR("Stored data does not contain %s.", key.toStdString().c_str());
        return msg;
    }

    QVariant msg_data = map[key];

    // Check if the data can be convertered.
    if (!msg_data.canConvert(QVariant::ByteArray))
    {
        ROS_ERROR("Failed to convert stored data to QByteArray.");
        return msg;
    }

    // Convert the variant to a byte array.
    QByteArray byte_array = msg_data.toByteArray();

    // Deserialize message.
    msg = deserializeMessage<Message>(byte_array);

    return msg;
}

template< typename Message > void MotionPlanningFrame::setMessageToUserData(QVariant& data, const Message& msg)
{
    // Ensure the data is a QVariantMap.
    if (!data.canConvert(QVariant::Map))
        data = QVariantMap();

    // Get the underlying map.
    QVariantMap map = data.toMap();

    // Create a byte array.
    QByteArray byte_array;

    // Serialize the goal message into a byte array.
    byte_array = serializeMessage<Message>(msg);

    // Auto-generate key for message type. May not be human readable.
    QString key = "DUMB_KEY"; // typeid(Message).name();

    // Set the goal message into the map.
    map[key] = byte_array;

    // Set the map back into the data.
    data = map;
}

void MotionPlanningFrame::getRobotStateFromUserData(const QVariant& data,
                                                    robot_state::RobotState& robot)
{
    // Get the ith goal message in the data vector.
    apc_msgs::PrimitiveAction goal = getMessageFromUserData<apc_msgs::PrimitiveAction>(data);

    // Get the group name.
    std::string group = goal.group_name;

    // Convert goal message into joint state message.
    sensor_msgs::JointState state;
    state.name = goal.joint_trajectory.joint_names;
    // Get the joint constraints. Note we only extract the first one!
    state.position = goal.joint_trajectory.points[0].positions;

    // Merge the goal message into the robot state. This keeps the previous values.
    robot.setVariableValues(state);

    // Update the dirty transforms, etc.
    robot.update();
}



void MotionPlanningFrame::saveGoalAsItem(QListWidgetItem* item)
{
    const robot_state::RobotState& state = *planning_display_->getQueryGoalState();
    saveGoalAsItem(state, item);
}

void MotionPlanningFrame::saveGoalAsItem(const robot_state::RobotState& state,
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
    apc_msgs::PrimitiveAction goal;

    // Set the goal group name.
    goal.group_name = group;

    // Construct goal constraints only for the joint model group.
    moveit_msgs::Constraints constraints = kinematic_constraints::constructGoalConstraints(state,
                                                                                           joint_model_group,
                                                                                           goal_joint_tolerance);

    // Copy constraints to goal.
    goal.joint_trajectory.points.resize(1);
    for (int i = 0; i < constraints.joint_constraints.size(); i++)
    {
        goal.joint_trajectory.joint_names.push_back(constraints.joint_constraints[i].joint_name);
        goal.joint_trajectory.points[0].positions.push_back(constraints.joint_constraints[i].position);
    }

    // TODO Build a descriptive name.
    static int count = 0;
    QString display_name = QString("%1: %2").arg(group.c_str()).arg(count++);

    goal.action_name = display_name.toStdString();

    // Get the data in the item.
    QVariant data = item->data(Qt::UserRole);

    // Store the goal into the data.
    setMessageToUserData<apc_msgs::PrimitiveAction>(data, goal);

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

    // Deserialize byte array into move group goal.
    apc_msgs::PrimitiveAction goal = getMessageFromUserData<apc_msgs::PrimitiveAction>(data);

    // Get the new planning group.
    std::string group = goal.group_name;

    // Update the planning group.
    planning_display_->changePlanningGroup(group);

    // Get the current goal state.
    robot_state::RobotState current_state = *planning_display_->getQueryGoalState();

    // Get the saved joint constraints.
    const trajectory_msgs::JointTrajectory& joint_trajectory = goal.joint_trajectory;

    // Copy the joints in the goal to the current state.
    for (int i = 0; i < joint_trajectory.joint_names.size(); i++)
    {
        current_state.setJointPositions(joint_trajectory.joint_names[i],
                                        &joint_trajectory.points[0].positions[i]);
    }

    // Set the joints related to the current group.
    planning_display_->setQueryGoalState(current_state);
}

void MotionPlanningFrame::updateDisplayWaypoints(QListWidget* list)
{
    std::vector<QVariant> data;
    for (int i = 0; i < list->count(); i++)
        data.push_back(list->item(i)->data(Qt::UserRole));

    updateDisplayWaypoints(data);

    for (int i = 0; i < list->count(); i++)
        list->item(i)->setData(Qt::UserRole, data[i]);
}

void MotionPlanningFrame::updateDisplayWaypoints(QTreeWidget* tree)
{

}

void MotionPlanningFrame::updateDisplayWaypoints(std::vector<QVariant>& data)
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
        apc_msgs::PrimitiveAction goal = getMessageFromUserData<apc_msgs::PrimitiveAction>(data[i]);

        // Get the group name.
        group = goal.group_name;

        // Get the joint trajectory. Note we only extract the first one!
        const trajectory_msgs::JointTrajectory& joint_trajectory = goal.joint_trajectory;

        // Convert goal message into joint state message.
        sensor_msgs::JointState state;
        for (int j = 0; j < joint_trajectory.joint_names.size(); j++)
        {
            state.name.push_back(joint_trajectory.joint_names[j]);
            state.position.push_back(joint_trajectory.points[0].positions[j]);
        }

        // Merge the goal message into the robot state. This keeps the previous values.
        waypoint.setVariableValues(state);

        // Update the dirty transforms, etc.
        waypoint.update();

        // Add waypoint.
        planning_display_->addDisplayWaypoint(waypoint, group, link_names, focus);
    }
}

void MotionPlanningFrame::pushButtonClicked()
{
    // Get the list of active goals (waypoints).
    QListWidget* active_goals_list = ui_->active_goals_list;

    // Save the current goal to a new item.
    QListWidgetItem* item = new QListWidgetItem;
    saveGoalAsItem(item);

    // Get the currently selected row.
    int row = active_goals_list->currentRow();

    // Unselect item.
    if (active_goals_list->currentItem())
        active_goals_list->currentItem()->setSelected(false);

    // Insert the item into the list.
    active_goals_list->insertItem(row + 1, item);

    // Set item as active.
    active_goals_list->setCurrentItem(item);

    // update rendering.
    updateDisplayWaypoints(active_goals_list);
}

void MotionPlanningFrame::activeGoalItemDoubleClicked(QListWidgetItem* item)
{
    // Load the currently selected item into the goal query state.
    loadGoalFromItem(item);

    // Update rendering. FIXME Not needed here!
    // updateDisplayWaypoints(ui_->active_goals_list);
}

void MotionPlanningFrame::popButtonClicked()
{
    // Get the list of active goals (waypoints).
    QListWidget* active_goals_list = ui_->active_goals_list;

    if (active_goals_list->count() == 0)
        return;

    int row = active_goals_list->currentRow();

    QListWidgetItem* item = active_goals_list->takeItem(row);

    if (item)
        delete item;

    active_goals_list->setCurrentRow(std::max(row - 1, 0));

    // Update rendering.
    updateDisplayWaypoints(active_goals_list);
}

void MotionPlanningFrame::previewButtonClicked()
{
    planning_display_->previewTrail();
}

void MotionPlanningFrame::savePlansButtonClicked()
{
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSavePlansButtonClicked, this), "save plans");
}

void MotionPlanningFrame::loadPlansButtonClicked()
{
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeLoadPlansButtonClicked, this), "load plans");
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

    // TODO Build a descriptive name.
    static int count = 0;
    QString display_name = QString("plan: %1").arg(count++);

    root->setText(0, display_name);

    // Add to tree list.
    QTreeWidget* stored_plans = ui_->stored_plans_tree;
    stored_plans->addTopLevelItem(root);
}

void MotionPlanningFrame::storedToActiveGoalsButtonClicked()
{
    // Tree of stored plans.
    QTreeWidget* stored_plans = ui_->stored_plans_tree;

    // The selected stored plan.
    QTreeWidgetItem* plan_item = stored_plans->currentItem();

    if (!plan_item)
        return;

    // List of active goals.
    QListWidget* active_goals = ui_->active_goals_list;

    // Active goal's selected row or else the last row.
    int row = active_goals->currentRow() == -1 ? active_goals->count()-1 : active_goals->currentRow();

    // Append stored plans to active goals.
    for (int i = 0; i < plan_item->childCount(); i++)
    {
        QListWidgetItem* item = new QListWidgetItem;

        apc_msgs::PrimitiveAction goal =
            getMessageFromUserData<apc_msgs::PrimitiveAction>(plan_item->child(i)->data(0, Qt::UserRole));

        item->setText(QString(goal.action_name.c_str()));

        item->setData(Qt::UserRole, plan_item->child(i)->data(0, Qt::UserRole));

        active_goals->insertItem(++row, item);
    }

    // FIXME Doesn't work...
    // planning_display_->addBackgroundJob(
    //     boost::bind(static_cast<void (*)(QListWidget*)>(&MotionPlanningFrame::updateDisplayWaypoints),
    //                 this,
    //                 ui_->active_goals_list),
    //     "update display waypoints");
    updateDisplayWaypoints(active_goals);
}

void MotionPlanningFrame::computeSavePlansButtonClicked()
{
    if (primitive_plan_storage_)
    {
        // TODO
        QTreeWidget* stored_plans = ui_->stored_plans_tree;

        // A primitive plan message.
        apc_msgs::PrimitivePlan msg;

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

                // Append primitive action to primitive plan.
                msg.actions.push_back(getMessageFromUserData<apc_msgs::PrimitiveAction>(child->data(0, Qt::UserRole)));
            }

            // Store message in database.
            try
            {
                primitive_plan_storage_->removePrimitivePlan(msg.plan_name);
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

        root->setText(0, QString(plan->plan_name.c_str()));

        for (int j = 0; j < plan->actions.size(); j++)
        {
            QTreeWidgetItem* child = new QTreeWidgetItem;

            // Set the child's name.
            child->setText(0, QString(plan->actions[j].action_name.c_str()));

            // Store primitive action in the data.
            QVariant data;
            setMessageToUserData<apc_msgs::PrimitiveAction>(data, plan->actions[j]);

            // Store data into the child node.
            child->setData(0, Qt::UserRole, data);

            // Append to root.
            root->addChild(child);
        }

        ui_->stored_plans_tree->addTopLevelItem(root);
    }
}

// HACK Define the template instantiations here because the template definitions are in this cpp file.
template QByteArray MotionPlanningFrame::serializeMessage<apc_msgs::PrimitiveAction>(const apc_msgs::PrimitiveAction& msg);
template QByteArray MotionPlanningFrame::serializeMessage<apc_msgs::PrimitivePlan>(const apc_msgs::PrimitivePlan& msg);
template apc_msgs::PrimitiveAction MotionPlanningFrame::deserializeMessage<apc_msgs::PrimitiveAction>(const QByteArray& string);
template apc_msgs::PrimitivePlan MotionPlanningFrame::deserializeMessage<apc_msgs::PrimitivePlan>(const QByteArray& string);
template apc_msgs::PrimitiveAction MotionPlanningFrame::getMessageFromUserData<apc_msgs::PrimitiveAction>(const QVariant& data);
template apc_msgs::PrimitivePlan MotionPlanningFrame::getMessageFromUserData<apc_msgs::PrimitivePlan>(const QVariant& data);
template void MotionPlanningFrame::setMessageToUserData<apc_msgs::PrimitiveAction>(QVariant& data, const apc_msgs::PrimitiveAction& msg);
template void MotionPlanningFrame::setMessageToUserData<apc_msgs::PrimitivePlan>(QVariant& data, const apc_msgs::PrimitivePlan& msg);
}
