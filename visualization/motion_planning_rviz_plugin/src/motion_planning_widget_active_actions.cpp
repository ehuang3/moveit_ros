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
#include <eigen_conversions/eigen_msg.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <sstream>
#include <boost/algorithm/string.hpp>


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

    void MotionPlanningFrame::saveStartAndGoalToAction(apc_msgs::PrimitiveAction& action)
    {
        // Set action group.
        action.group_name = planning_display_->getCurrentPlanningGroup();
        // Clear joint trajectory.
        action.joint_trajectory.joint_names.clear();
        action.joint_trajectory.points.clear();
        // Save start and end points to joint trajectory.
        const robot_state::RobotState& start_state = *planning_display_->getQueryStartState();
        const robot_state::RobotState& goal_state  = *planning_display_->getQueryGoalState();
        // If start and goal state are locked together, use the goal
        // state as the start.
        if (ui_->padlock_button->isChecked())
            appendStateToAction(action, goal_state);
        else
            appendStateToAction(action, start_state);
        appendStateToAction(action, goal_state);
    }

    void MotionPlanningFrame::appendStateToAction(apc_msgs::PrimitiveAction& action,
                                                  const robot_state::RobotState& state)
    {
        if (action.group_name.empty())
        {
            ROS_ERROR("No group provided in action");
        }
        const moveit::core::JointModelGroup* joint_model_group = state.getJointModelGroup(action.group_name);
        if (!joint_model_group)
        {
            ROS_ERROR("Failed to get joint model group %s", action.group_name.c_str());
            return;
        }
        // Get the goal joint tolerance.
        double goal_joint_tolerance = move_group_->getGoalJointTolerance();
        // Construct goal constraints only for the joint model group.
        moveit_msgs::Constraints constraints = kinematic_constraints::constructGoalConstraints(state,
                                                                                               joint_model_group,
                                                                                               goal_joint_tolerance);
        // Warn the user if the number of joints in the joint group
        // doesn't match previous points in the trajectory.
        int num_state_joints  = joint_model_group->getJointModelNames().size();
        int num_action_joints = action.joint_trajectory.joint_names.size();
        if (num_state_joints != num_action_joints)
        {
            ROS_ERROR("Mismatch in number of joints when appending state to action");
            return;
        }
        if (num_action_joints == 0)
            action.joint_trajectory.joint_names = joint_model_group->getJointModelNames();
        else
            for (int i = 0; i < joint_model_group->getJointModelNames().size(); i++)
                // if (std::find(action.joint_trajectory.joint_names.begin(),
                //               action.joint_trajectory.joint_names.end(),
                //               joint_model_group->getJointModelNames()[i]) == action.joint_trajectory.joint_names.end())
                if (joint_model_group->getJointModelNames()[i] != action.joint_trajectory.joint_names[i])
                {
                    ROS_ERROR("Mismatch in joint names (or joint order) when appending state to action");
                    return;
                }

        trajectory_msgs::JointTrajectoryPoint point;
        state.copyJointGroupPositions(joint_model_group, point.positions);
        action.joint_trajectory.points.push_back(point);
    }

    void MotionPlanningFrame::saveFormatToAction(apc_msgs::PrimitiveAction& action, const std::string& format)
    {
        if (action.group_name.empty())
            action.group_name = planning_display_->getCurrentPlanningGroup();
        // Construct %a token.
        std::string a = action.group_name;
        // Construct %g token.
        std::string g;
        std::string token;
        std::istringstream iss(action.group_name);
        while (std::getline(iss, token, '_'))
            g.push_back(token[0]);
        // Construct %i token.
        std::stringstream ss;
        ss << ui_->active_actions_list->count();
        std::string i = ss.str();
        // Substitute args into format.
        std::string name = format;
        boost::replace_all(name, "%a", a);
        boost::replace_all(name, "%g", g);
        boost::replace_all(name, "%i", i);
        action.action_name = name;
    }

    std::string MotionPlanningFrame::computeNearestBin(std::string group,
                                                       robot_state::RobotState& state)
    {
        const boost::shared_ptr<const srdf::Model> &srdf = planning_display_->getRobotModel()->getSRDF();
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group);
        if (!jmg)
        {
            ROS_ERROR("Failed to find group: %s", group.c_str());
            return "";
        }
        // Find active end-effector.
        const std::vector<srdf::Model::EndEffector> &eef = srdf->getEndEffectors();
        std::vector<srdf::Model::EndEffector> active_eef;
        for (int i = 0; i < eef.size(); i++)
            if (jmg->hasLinkModel(eef[i].parent_link_) ||
                jmg->getName() == eef[i].parent_group_)
                active_eef.push_back(eef[i]);
        if (active_eef.size() != 1)
        {
            ROS_ERROR("No or more than one end-effector found");
            return "";
        }
        // Get world transform of end-effector parent link.
        Eigen::Affine3d T_eef = state.getGlobalLinkTransform(active_eef[0].parent_link_);
        // Find closest bin to parent link.
        if (!_kiva_pod)
        {
            ROS_ERROR("KIVA Pod not loaded!");
            return "";
        }
        Eigen::Affine3d T_pod;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            Eigen::Affine3d T_pod = ps->getWorld()->getObject("kiva_pod")->shape_poses_[0];
        }
        double min_dist = 1e9;
        std::string bin;
        for (char c = 'A'; c <= 'L'; c++)
        {
            std::string current_bin = std::string("bin_") + c;
            Eigen::Affine3d T_bin = T_pod * _kiva_pod->getGlobalTransform(current_bin);
            if (min_dist > (T_bin.translation() - T_eef.translation()).norm())
            {
                min_dist = (T_bin.translation() - T_eef.translation()).norm();
                bin = current_bin;
            }
        }
        return bin;
    }

    Eigen::Isometry3d MotionPlanningFrame::computeFrame(const std::string& frame,
                                                        const std::string& group,
                                                        robot_state::RobotState& state)
    {
        if (frame.empty())
        {
            ROS_WARN("Attempted to look up empty frame!");
            return Eigen::Isometry3d::Identity();
        }
        // Compute the frame lookup information.
        std::string object_name = frame;
        std::string bin_name = "";
        if (frame.find("bin") == 0)
        {
            object_name = "kiva_pod";
            bin_name = frame;
            if (frame == "bin")
                bin_name = computeNearestBin(group, state);
            if (bin_name.empty())
            {
                ROS_ERROR("Failed to compute frame");
                return Eigen::Isometry3d::Identity();
            }
        }

        // Lookup the frame.
        Eigen::Isometry3d T_frame = Eigen::Isometry3d::Identity();
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            if (!ps->getWorld()->getObject(object_name))
            {
                ROS_ERROR("Failed to get %s from world", object_name.c_str());
                return T_frame;
            }
            Eigen::Affine3d T_object = ps->getWorld()->getObject(object_name)->shape_poses_[0];
            if (!bin_name.empty() && !_kiva_pod)
            {
                ROS_ERROR("Failed to get KIVA Pod kinematics");
                return T_frame;
            }
            if (bin_name)
                T_frame = T_object * _kiva_pod->getGlobalTransform(bin_name);
            else
                T_frame = T_object;
        }
        return T_frame;
    }

    void MotionPlanningFrame::saveFrameToAction(apc_msgs::PrimitiveAction& action, const std::string& frame)
    {
        if (frame.empty())
            return;
        if (action.group_name.empty())
            action.group_name = planning_display_->getCurrentPlanningGroup();
        // Handle start and goal.
        robot_state::RobotState start_state = planning_display_->getQueryStartState();
        robot_state::RobotState goal_state  = planning_display_->getQueryGoalState();
        if (ui_->padlock_button->isChecked())
            start_state = goal_state;

        // Get frame location of "bin", "bin_%alpha", or "<object>".
        Eigen::Isometry3d T_frame = computeFrame(frame, action.group_name, start_state);

        // Find active end-effector.
        const boost::shared_ptr<const srdf::Model> &srdf = planning_display_->getRobotModel()->getSRDF();
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group);
        if (!jmg)
        {
            ROS_ERROR("Failed to find group: %s", group.c_str());
            return "";
        }

        std::string eef_link;
        const std::vector<srdf::Model::EndEffector> &eef = srdf->getEndEffectors();
        std::vector<srdf::Model::EndEffector> active_eef;
        for (int i = 0; i < eef.size(); i++)
            if (jmg->hasLinkModel(eef[i].parent_link_) ||
                jmg->getName() == eef[i].parent_group_)
                active_eef.push_back(eef[i]);
        if (active_eef.size() == 0) // Use last link as "eef".
            eef_link = jmg->getLinkModelNames().back();
        else if (active_eef.size() == 1) // Use last link as "eef".
            eef_link = active_eef[0].parent_link_;
        else if (active_eef.size() > 1)
        {
            ROS_ERROR("More than one end-effector found");
            return "";
        }
        // Get relative transform from frame to start eef.
        Eigen::Affine3d T_start_inv = start_state.getGlobalLinkTransform(eef_link).inverse();
        Eigen::Affine3d T_start_frame = T_start_inv * T_frame;
        Eigen::Affine3d T_goal_inv = goal_state.getGlobalLinkTransform(eef_link).inverse();
        Eigen::Affine3d T_goal_frame = T_start_inv * T_frame;
        // Write to action.
        action.eef_trajectory.poses.resize(2);
        tf::poseEigenToMsg(T_start_frame, action.eef_trajectory.poses[0]);
        tf::poseEigenToMsg(T_start_frame, action.eef_trajectory.poses[1]);
    }

    void MotionPlanningFrame::loadStateFromAction(robot_state::RobotState& state,
                                                  const apc_msgs::PrimitiveAction& action)
    {
        // Get the saved joint constraints.
        const trajectory_msgs::JointTrajectory& joint_trajectory = action.joint_trajectory;

        // Warn when the joint trajectory has more than one point!
        if (joint_trajectory.points.size() != 1)
            ROS_WARN("Loading state from an action with a dense trajectory!");

        // Get the number of points in this trajectory.
        int num_points = joint_trajectory.points.size();

        // Copy the joints in the goal to the current state.
        for (int i = 0; i < joint_trajectory.joint_names.size(); i++)
        {
            state.setJointPositions(joint_trajectory.joint_names[i],
                                    &joint_trajectory.points[num_points - 1].positions[i]);
        }

        // Add attached objects if they exist.
        computeAttachObjectToState(state,
                                   action.object_name,
                                   action.link_name,
                                   action.object_poses);

        // If this action was recorded with an attached object, default to
        // executing relative to object.
        if (!action.object_name.empty() && action.object_poses.size() > 0)
        {
            // Extract the object from the world.
            collision_detection::CollisionWorld::ObjectConstPtr object;
            {
                planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
                object = ps->getWorld()->getObject(action.object_name);
            }
            // If the object has not been loaded into the world, do nothing.
            if (!object)
            {
                ROS_ERROR("Failed to get object %s from world.",
                          action.object_name.c_str());
            }
            else
            {
                // Create link to object pose offsets from action.
                EigenSTL::vector_Affine3d poses(action.object_poses.size());
                for (int i = 0; i < action.object_poses.size(); i++)
                    tf::poseMsgToEigen(action.object_poses[i], poses[i]);
                // Back out the desired link transform in global coordinates.
                Eigen::Affine3d T_link = object->shape_poses_[0] * poses[0].inverse();
                // Create arguments for IK.
                const moveit::core::JointModelGroup* group = state.getJointModelGroup(action.group_name);
                geometry_msgs::Pose pose;
                tf::poseEigenToMsg(T_link, pose);
                // Compute IK to link.
                state.setFromIK(group, pose);
            }
        }

        // Update the dirty transforms, etc.
        state.update();
    }

    void MotionPlanningFrame::saveStateToAction(const robot_state::RobotState& state,
                                                apc_msgs::PrimitiveAction& action)
    {
        const moveit::core::JointModelGroup* joint_model_group = state.getJointModelGroup(action.group_name);
        if (!joint_model_group)
        {
            ROS_ERROR("Failed to get joint model group %s", action.group_name.c_str());
            return;
        }
        // Get the goal joint tolerance.
        double goal_joint_tolerance = move_group_->getGoalJointTolerance();
        // Construct goal constraints only for the joint model group.
        moveit_msgs::Constraints constraints = kinematic_constraints::constructGoalConstraints(state,
                                                                                               joint_model_group,
                                                                                               goal_joint_tolerance);
        // Copy joint angles and names to action.
        action.joint_trajectory.points.resize(1);
        for (int i = 0; i < constraints.joint_constraints.size(); i++)
        {
            action.joint_trajectory.joint_names.push_back(constraints.joint_constraints[i].joint_name);
            action.joint_trajectory.points[0].positions.push_back(constraints.joint_constraints[i].position);
        }
        // Save attached objects to the plan.
        computeAttachObjectToAction(state, action);
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

    void MotionPlanningFrame::saveActionToItem(QListWidgetItem* item)
    {
        const robot_state::RobotState& state = *planning_display_->getQueryGoalState();
        saveActionToItem(state, item);
    }

    void MotionPlanningFrame::saveActionToItem(const robot_state::RobotState& state,
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

        // Create a move group action message.
        apc_msgs::PrimitivePlan action;
        action.actions.resize(1);

        // Set the action group name.
        action.actions[0].group_name = group;

        // Save state information to action.
        saveStateToAction(state, action.actions[0]);

        // TODO Build a descriptive name.
        static int count = 0;
        QString display_name = QString("%1: %2").arg(group.c_str()).arg(count++);

        action.actions[0].action_name = display_name.toStdString();

        // Get the data in the item.
        QVariant data = item->data(Qt::UserRole);

        // Store the action into the data.
        setMessageToUserData<apc_msgs::PrimitivePlan>(data, action);

        // Save the serialized data back into the item.
        item->setData(Qt::UserRole, data);

        // Set the display name of the item.
        item->setText(display_name);
    }

    void MotionPlanningFrame::loadActionFromItem(QListWidgetItem* item)
    {
        if (!item)
            return;

        // Get the saved binary data.
        QVariant data = item->data(Qt::UserRole);

        // Load action from data.
        loadActionFromData(data);
    }

    void MotionPlanningFrame::loadActionFromItem(QTreeWidgetItem* item)
    {
        if (!item)
            return;

        // Get the saved binary data.
        QVariant data = item->data(0, Qt::UserRole);

        // Load action from data.
        loadActionFromData(data);
    }

    void MotionPlanningFrame::loadActionFromData(const QVariant& data)
    {
        // Deserialize byte array into move group action.
        apc_msgs::PrimitivePlan action = getMessageFromUserData<apc_msgs::PrimitivePlan>(data);

        // Get the new planning group.
        std::string group = action.actions[0].group_name;

        // Update the planning group.
        planning_display_->changePlanningGroup(group);

        // Get the current action state.
        robot_state::RobotState current_state = *planning_display_->getQueryGoalState();

        // Load state from the first action.
        loadStateFromAction(current_state, action.actions[0]);

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
            // Get the ith action message in the data vector.
            apc_msgs::PrimitivePlan action = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[i]);

            // Get the group name.
            group = action.actions[0].group_name;

            // Load action into state.
            loadStateFromAction(waypoint, action.actions[0]);

            // Update the dirty transforms, etc.
            waypoint.update();

            // Add waypoint.
            planning_display_->addDisplayWaypoint(waypoint, group, link_names, focus);
        }
    }

}
