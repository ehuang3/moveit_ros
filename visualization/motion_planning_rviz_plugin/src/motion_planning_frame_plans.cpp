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

void MotionPlanningFrame::serializeGoalMsg(const moveit_msgs::MoveGroupGoal& goal, QByteArray& string)
{
    // Get the length of serialization.
    uint32_t serial_size = ros::serialization::serializationLength(goal);

    // Construct a middle man for the serialization?
    boost::scoped_array<uint8_t> buffer(new uint8_t[serial_size]);

    // Create a serialization object.
    ros::serialization::OStream stream(buffer.get(), serial_size);

    // Serialize.
    ros::serialization::serialize(stream, goal);

    // Clear QByteArray.
    string.clear();

    // Copy to QByteArray.
    for (uint32_t i = 0; i < serial_size; i++)
    {
        char b = (char) buffer.get()[i];
        string.append(b);
    }
}

void MotionPlanningFrame::deserializeGoalMsg(const QByteArray& string, moveit_msgs::MoveGroupGoal& goal)
{
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
    ros::serialization::Serializer<moveit_msgs::MoveGroupGoal>::read(stream, goal);
}

void MotionPlanningFrame::saveGoalAsItem(QListWidgetItem* item)
{
    if (!item)
        return;

    // TODO Save the end-effector relative poses and the joint angles.

    // Get current planning group (set of active joints).
    std::string group = planning_display_->getCurrentPlanningGroup();

    // Get the current query goal state.
    robot_state::RobotState state = *planning_display_->getQueryGoalState();

    // Get the active joint models in that group.
    const moveit::core::JointModelGroup* joint_model_group = state.getJointModelGroup(group);

    // Get the goal joint tolerance.
    double goal_joint_tolerance = move_group_->getGoalJointTolerance();

    // Create a move group goal message.
    moveit_msgs::MoveGroupGoal goal;

    // Set the goal group name.
    goal.request.group_name = group;

    // Construct goal constraints only for the joint model group.
    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(state,
                                                                                       joint_model_group,
                                                                                       goal_joint_tolerance);

    // Serialize the goal message.
    QByteArray byte_array;
    serializeGoalMsg(goal, byte_array);

    // Save the serialized message into the item.
    item->setData(Qt::UserRole, byte_array);

    // TODO Build a descriptive name.
    static int count = 0;
    QString display_name = QString("%1: %2").arg(group.c_str()).arg(count++);

    // Set the display name of the item.
    item->setText(display_name);
}

void MotionPlanningFrame::loadGoalFromItem(QListWidgetItem* item)
{
    if (!item)
        return;

    // Get the saved binary data.
    QVariant data = item->data(Qt::UserRole);

    // Check if the data can be convertered.
    if (!data.canConvert(QVariant::ByteArray))
    {
        ROS_ERROR("Failed to convert stored goal to query goal state.");
        return;
    }

    // Convert the variant to a byte array.
    QByteArray byte_array = data.toByteArray();

    // Deserialize byte array into move group goal.
    moveit_msgs::MoveGroupGoal goal;
    deserializeGoalMsg(byte_array, goal);

    // Get the new planning group.
    std::string group = goal.request.group_name;

    // Update the planning group.
    planning_display_->changePlanningGroup(group);

    // Get the current goal state.
    robot_state::RobotState current_state = *planning_display_->getQueryGoalState();

    // Get the saved joint constraints.
    const std::vector<moveit_msgs::JointConstraint>& joint_constraints =
        goal.request.goal_constraints[0].joint_constraints;

    // Copy the joints in the goal to the current state.
    for (int i = 0; i < joint_constraints.size(); i++)
    {
        current_state.setJointPositions(joint_constraints[i].joint_name,
                                        &joint_constraints[i].position);
    }

    // Set the joints related to the current group.
    planning_display_->setQueryGoalState(current_state);
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

    // Insert the item into the list.
    active_goals_list->insertItem(row + 1, item);

    // Set item as active.
    active_goals_list->setCurrentItem(item);
}

void MotionPlanningFrame::activeGoalChanged(QListWidgetItem* current, QListWidgetItem* previous)
{
    if (current == previous)
        return;

    // Save the active goal query state into the previously selected item.
    saveGoalAsItem(previous);

    // Load the currently selected item into the goal query state.
    loadGoalFromItem(current);
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
}

void MotionPlanningFrame::previewButtonClicked()
{
    planning_display_->previewTrail();
}

void MotionPlanningFrame::savePlansButtonClicked()
{

}

void MotionPlanningFrame::loadPlansButtonClicked()
{

}

void MotionPlanningFrame::activeToStoredPlansButtonClicked()
{

}

void MotionPlanningFrame::storedToActiveGoalsButtonClicked()
{

}

}
