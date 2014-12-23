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

void MotionPlanningFrame::serializeGoalMsg(const moveit_msgs::RobotState& goal, QByteArray& string)
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

void MotionPlanningFrame::deserializeGoalMsg(const QByteArray& string, moveit_msgs::RobotState& goal)
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
    ros::serialization::Serializer<moveit_msgs::RobotState>::read(stream, goal);
}

void MotionPlanningFrame::pushButtonClicked()
{
    QListWidget* active_goals_list = ui_->active_goals_list;


    // Get the current robot state associated with the active interaction.
    planning_display_->getQueryGoalState();

    // Test serialization.
    moveit_msgs::RobotState goal;
    goal.joint_state.position.resize(1);
    goal.joint_state.position[0] = 10;

    std::cout << goal << std::endl;

    QByteArray string;
    serializeGoalMsg(goal, string);
    deserializeGoalMsg(string, goal);

    std::cout << goal << std::endl;


}

void MotionPlanningFrame::popButtonClicked()
{

}

void MotionPlanningFrame::previewButtonClicked()
{

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
