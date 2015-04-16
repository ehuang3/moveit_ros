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

    robot_state::RobotStateConstPtr MotionPlanningFrame::getQueryStartState()
    {
        return getQueryStartState(ui_->padlock_button->isChecked());
    }

    robot_state::RobotStateConstPtr MotionPlanningFrame::getQueryGoalState()
    {
        return getQueryGoalState(ui_->padlock_button->isChecked());
    }

    robot_state::RobotStateConstPtr MotionPlanningFrame::getQueryStartState(bool locked)
    {
        if (!locked)
            return planning_display_->getQueryStartState();
        else
            return planning_display_->getQueryGoalState();
    }

    robot_state::RobotStateConstPtr MotionPlanningFrame::getQueryGoalState(bool locked)
    {
        return planning_display_->getQueryGoalState();
    }

    bool MotionPlanningFrame::computeStartAndGoalEefLockedState(const apc_msgs::PrimitiveAction& action)
    {
        int n = action.eef_trajectory.poses.size();
        if (n == 0)
        {
            ROS_ERROR("Failed to compute start and goal locked state");
            return false;
        }
        geometry_msgs::Pose start, goal;
        start = action.eef_trajectory.poses[0];
        goal = action.eef_trajectory.poses[n-1];
        const double tol = 1e-7;

#define APC_EXPECT_NEAR(x, y, tol)                  \
        if (std::abs(x-y) > tol)                    \
            return false;                           \

        APC_EXPECT_NEAR(start.position.x, goal.position.x, tol);
        APC_EXPECT_NEAR(start.position.y, goal.position.y, tol);
        APC_EXPECT_NEAR(start.position.z, goal.position.z, tol);
        APC_EXPECT_NEAR(start.orientation.w, goal.orientation.w, tol);
        APC_EXPECT_NEAR(start.orientation.x, goal.orientation.x, tol);
        APC_EXPECT_NEAR(start.orientation.y, goal.orientation.y, tol);
        APC_EXPECT_NEAR(start.orientation.z, goal.orientation.z, tol);

#undef APC_EXPECT_NEAR

        return true;
    }

    void MotionPlanningFrame::saveLockedStateToAction(apc_msgs::PrimitiveAction& action)
    {
        action.eef_locked = ui_->padlock_button->isChecked();
    }

    void MotionPlanningFrame::saveStartAndGoalToAction(apc_msgs::PrimitiveAction& action)
    {
        // Set action group.
        action.group_id = planning_display_->getCurrentPlanningGroup();
        // Clear joint trajectory.
        action.joint_trajectory.joint_names.clear();
        action.joint_trajectory.points.clear();
        // If start and goal state are locked together, we will use
        // the goal state as the start.
        const robot_state::RobotState& start_state = *getQueryStartState();
        const robot_state::RobotState& goal_state  = *getQueryGoalState();
        // Save start and end points to joint trajectory.
        appendStateToAction(action, start_state);
        appendStateToAction(action, goal_state);
    }

    void MotionPlanningFrame::appendStateToAction(apc_msgs::PrimitiveAction& action,
                                                  const robot_state::RobotState& state)
    {
        if (action.group_id.empty())
        {
            ROS_ERROR("No group provided in action");
        }
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(action.group_id);
        if (!jmg)
        {
            ROS_ERROR("Failed to get joint model group %s", action.group_id.c_str());
            return;
        }
        // Get the goal joint tolerance.
        double goal_joint_tolerance = move_group_->getGoalJointTolerance();
        // Construct goal constraints only for the joint model group.
        moveit_msgs::Constraints constraints = kinematic_constraints::constructGoalConstraints(state,
                                                                                               jmg,
                                                                                               goal_joint_tolerance);
        // Warn the user if the number of joints in the joint group
        // doesn't match previous points in the trajectory.
        int num_state_joints  = jmg->getActiveJointModelNames().size();
        int num_action_joints = action.joint_trajectory.joint_names.size();
        if (num_action_joints == 0)
        {
            action.joint_trajectory.joint_names = jmg->getActiveJointModelNames();
        }
        else if (num_state_joints != num_action_joints)
        {
            ROS_ERROR("Mismatch in number of joints when appending state to action");
            return;
        }
        else
        {
            for (int i = 0; i < jmg->getActiveJointModelNames().size(); i++)
                // if (std::find(action.joint_trajectory.joint_names.begin(),
                //               action.joint_trajectory.joint_names.end(),
                //               jmg->getJointModelNames()[i]) == action.joint_trajectory.joint_names.end())
                if (jmg->getActiveJointModelNames()[i] != action.joint_trajectory.joint_names[i])
                {
                    ROS_ERROR("Mismatch in joint names (or joint order) when appending state to action");
                    return;
                }
        }

        trajectory_msgs::JointTrajectoryPoint point;
        state.copyJointGroupPositions(jmg, point.positions);
        action.joint_trajectory.points.push_back(point);
    }

    void MotionPlanningFrame::saveFormatToAction(apc_msgs::PrimitiveAction& action)
    {
        saveFormatToAction(action, ui_->format_line_edit->text().toStdString());
    }

    void MotionPlanningFrame::saveFormatToAction(apc_msgs::PrimitiveAction& action, const std::string& format)
    {
        if (action.group_id.empty())
            action.group_id = planning_display_->getCurrentPlanningGroup();
        // Construct %a token.
        std::string a = action.group_id;
        // Construct %g token.
        std::string g;
        std::string token;
        std::istringstream iss(action.group_id);
        while (std::getline(iss, token, '_'))
            g.push_back(token[0]);
        // Construct %i token.
        std::stringstream ss;
        ss << ui_->active_actions_list->count();
        std::string i = ss.str();
        // Construct %o token.
        std::string o;
        std::string o_token;
        std::istringstream o_iss(action.object_id);
        while (std::getline(o_iss, o_token, '_'))
            o.push_back(o_token[0]);
        // Substitute args into format.
        std::string name = format;
        boost::replace_all(name, "%a", a);
        boost::replace_all(name, "%g", g);
        boost::replace_all(name, "%i", i);
        boost::replace_all(name, "%o", o);
        action.action_name = name;
    }

    std::string MotionPlanningFrame::computeEefLink(const std::string& group)
    {
        const boost::shared_ptr<const srdf::Model> &srdf = planning_display_->getRobotModel()->getSRDF();
        const moveit::core::JointModelGroup* jmg = planning_display_->getQueryStartState()->getJointModelGroup(group);
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
        return eef_link;
    }

    std::string MotionPlanningFrame::computeNearestBin(std::string link,
                                                       robot_state::RobotState& state)
    {
        ROS_DEBUG_FUNCTION;

        const boost::shared_ptr<const srdf::Model> &srdf = planning_display_->getRobotModel()->getSRDF();
        if (!_kiva_pod)
        {
            ROS_ERROR("KIVA Pod not loaded!");
            return "";
        }

        // Get world transform of link.
        Eigen::Affine3d T_link = state.getGlobalLinkTransform(link);

        // Get world transform to KIVA pod.
        Eigen::Affine3d T_pod_world;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            T_pod_world = ps->getWorld()->getObject("kiva_pod")->shape_poses_[0];
        }

        // Find closest bin to parent link.
        double min_dist = 1e9;
        std::string bin;
        for (char c = 'A'; c <= 'L'; c++)
        {
            std::string current_bin = std::string("bin_") + c;
            Eigen::Affine3d T_bin_pod = _kiva_pod->getGlobalTransform(current_bin);
            Eigen::Affine3d T_bin_world = T_pod_world * T_bin_pod;
            double dist = (T_bin_world.translation() - T_link.translation()).norm();
            if (min_dist > dist)
            {
                min_dist = dist;
                bin = current_bin;
            }
        }

        ROS_DEBUG("bin: %s", bin.c_str());
        ROS_DEBUG("min dist: %f", min_dist);
        ROS_DEBUG_FUNCTION;

        return bin;
    }

    std::string MotionPlanningFrame::computeNearestObject(const std::string& object,
                                                          const std::string& link,
                                                          robot_state::RobotState& state)
    {
        ROS_DEBUG_FUNCTION;

        // Get the world where all objects are stored.
        planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
        const collision_detection::WorldConstPtr world = ps->getWorld();

        // Get the group end-effector link transform.
        Eigen::Affine3d T_link_world = state.getGlobalLinkTransform(link);

        // Find the object closest to the group's end-effector link.
        const std::vector<std::string>& object_ids = ps->getWorld()->getObjectIds();
        double min_dist = 1e9;
        std::string nearest_object;
        for (int i = 0; i < object_ids.size(); i++)
            if (object_ids[i].find(object) == 0)
            {
                Eigen::Affine3d T_object_world = world->getObject(object_ids[i])->shape_poses_[0];
                double dist = (T_link_world.translation() - T_object_world.translation()).norm();
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest_object = object_ids[i];
                }
            }

        ROS_DEBUG("nearest object: %s", nearest_object.c_str());
        ROS_DEBUG("min dist: %.4f", min_dist);
        ROS_DEBUG_FUNCTION;

        return nearest_object;
    }

    Eigen::Affine3d MotionPlanningFrame::computeFrame(const std::string& frame)
    {
        ROS_DEBUG_FUNCTION;

        // Get the world state.
        planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
        const collision_detection::WorldConstPtr world = ps->getWorld();

        // Lookup the frame.
        Eigen::Affine3d T_frame_world = Eigen::Affine3d::Identity();

        collision_detection::World::ObjectConstPtr object;
        if (frame.find("bin") == 0)
            object = world->getObject("kiva_pod");
        else
            object = world->getObject(frame);

        T_frame_world = object->shape_poses_[0];

        if (frame.find("bin") == 0)
            T_frame_world = T_frame_world * _kiva_pod->getGlobalTransform(frame);

        ROS_DEBUG_STREAM("T_frame_world:\n" << T_frame_world.matrix());
        ROS_DEBUG_FUNCTION;

        return T_frame_world;
    }

    Eigen::Affine3d MotionPlanningFrame::computeNearestFrame(const std::string& frame,
                                                             const std::string& group,
                                                             robot_state::RobotState& state)
    {
        ROS_DEBUG_FUNCTION;

        if (frame.empty())
        {
            ROS_WARN("Attempting to look up empty frame!");
            return Eigen::Affine3d::Identity();
        }

        // Compute the end-effector link.
        std::string eef_link = computeEefLink(group);

        // Compute the nearest frame to our state, provided ambiguity exists.
        std::string nearest_frame;
        if (frame.find("bin") == 0)
            nearest_frame = computeNearestBin(eef_link, state);
        else
            nearest_frame = computeNearestObject(frame, eef_link, state);

        // Lookup the frame.
        Eigen::Affine3d T_frame_world = computeFrame(nearest_frame);

        ROS_DEBUG_FUNCTION;

        return T_frame_world;
    }

    void MotionPlanningFrame::saveFrameToAction(apc_msgs::PrimitiveAction& action)
    {
        saveFrameToAction(action, ui_->frame_combobox->currentText().toStdString());
    }

    void MotionPlanningFrame::saveFrameToAction(apc_msgs::PrimitiveAction& action, const std::string& frame)
    {
        // Clear the frame information.
        action.frame_id = "";
        action.eef_link_id = "";
        action.eef_trajectory.poses.clear();

        if (frame.empty())
            return;
        if (action.group_id.empty())
            action.group_id = planning_display_->getCurrentPlanningGroup();

        // Handle start and goal.
        robot_state::RobotState start_state = *planning_display_->getQueryStartState();
        robot_state::RobotState goal_state  = *planning_display_->getQueryGoalState();
        if (ui_->padlock_button->isChecked())
            start_state = goal_state;

        // Get frame location of "bin", "bin_%alpha", or "<object>".
        Eigen::Affine3d T_frame = computeNearestFrame(frame, action.group_id, start_state);

        // Find active end-effector.
        std::string eef_link = computeEefLink(action.group_id);

        // Get the frame transform relative to the end-effector link.
        Eigen::Affine3d T_start_world = start_state.getGlobalLinkTransform(eef_link);
        Eigen::Affine3d T_frame_start = T_start_world.inverse() * T_frame;
        Eigen::Affine3d T_goal_world = goal_state.getGlobalLinkTransform(eef_link);
        Eigen::Affine3d T_frame_goal = T_goal_world.inverse() * T_frame;

        // std::cout << "T_start_world:\n" << T_start_world.matrix() << std::endl;
        // std::cout << "T_goal_world:\n" << T_goal_world.matrix() << std::endl;

        // std::cout << "T_frame_start:\n" << T_frame_start.matrix() << std::endl;
        // std::cout << "T_frame_goal:\n" << T_frame_goal.matrix() << std::endl;

        // Write to action.
        action.frame_id = frame;
        action.eef_link_id = eef_link;
        action.eef_trajectory.poses.resize(2);
        tf::poseEigenToMsg(T_frame_start, action.eef_trajectory.poses[0]);
        tf::poseEigenToMsg(T_frame_goal, action.eef_trajectory.poses[1]);
    }

    void MotionPlanningFrame::saveObjectToAction(apc_msgs::PrimitiveAction& action)
    {
        saveObjectToAction(action, ui_->object_combobox->currentText().toStdString());
    }

    void MotionPlanningFrame::saveObjectToAction(apc_msgs::PrimitiveAction& action,
                                                 const std::string& object)
    {
        // Clear attached object information.
        action.object_id = "";
        action.attached_link_id = "";
        action.object_trajectory.poses.clear();

        if (object.empty())
            return;
        if (action.group_id.empty())
            action.group_id = planning_display_->getCurrentPlanningGroup();

        // Handle start and goal.
        robot_state::RobotState start_state = *planning_display_->getQueryStartState();
        robot_state::RobotState goal_state  = *planning_display_->getQueryGoalState();
        if (ui_->padlock_button->isChecked())
            start_state = goal_state;

        // Get frame location of "bin", "bin_%alpha", or "<object>".
        Eigen::Affine3d T_object = computeNearestFrame(object, action.group_id, start_state);

        // Find active end-effector.
        std::string eef_link = computeEefLink(action.group_id);

        // Get object transform relative to the end-effector link.
        Eigen::Affine3d T_start_inv = start_state.getGlobalLinkTransform(eef_link).inverse();
        Eigen::Affine3d T_object_start = T_start_inv * T_object;
        Eigen::Affine3d T_goal_inv = goal_state.getGlobalLinkTransform(eef_link).inverse();
        Eigen::Affine3d T_object_goal = T_start_inv * T_object;

        // Write to object.
        action.object_id = object;
        action.attached_link_id = eef_link;
        action.object_trajectory.poses.resize(2);
        tf::poseEigenToMsg(T_object_start, action.object_trajectory.poses[0]);
        tf::poseEigenToMsg(T_object_goal, action.object_trajectory.poses[1]);
    }

    void MotionPlanningFrame::saveOptionsToAction(apc_msgs::PrimitiveAction& action)
    {
        std::map<std::string, bool> options;
        options["interpolate_cartesian"] = ui_->interpolate_cartesian_checkbox->isChecked();
        options["monitor_contact"] = ui_->monitor_contact_checkbox->isChecked();
        options["monitor_haptic_profile"] = ui_->monitor_profile_checkbox->isChecked();
        saveOptionsToAction(action, options);
    }

    void MotionPlanningFrame::saveOptionsToAction(apc_msgs::PrimitiveAction& action,
                                                  const std::map<std::string, bool>& options)
    {
        action.interpolate_cartesian = false;
        action.monitor_contact = false;
        action.monitor_haptic_profile = false;
        if (options.size() == 0)
            return;
        action.interpolate_cartesian = options.find("interpolate_cartesian")->second;
        action.monitor_contact = options.find("monitor_contact")->second;
        action.monitor_haptic_profile = options.find("monitor_haptic_profile")->second;
    }

    void MotionPlanningFrame::saveActionToData(const std::vector<apc_msgs::PrimitiveAction>& actions,
                                               std::vector<QVariant>& data)
    {
        data.resize(actions.size());
        for (int i = 0; i < actions.size(); i++)
            saveActionToData(actions[i], data[i]);
    }

    void MotionPlanningFrame::saveActionToData(const apc_msgs::PrimitiveAction& action,
                                               QVariant& data)
    {
        // Store the action into the data.
        setMessageToUserData<apc_msgs::PrimitiveAction>(data, action);
    }

    void MotionPlanningFrame::loadActionFromData(apc_msgs::PrimitiveAction& action,
                                                 const QVariant& data)
    {
        // Retrieve the action from the data.
        action = getMessageFromUserData<apc_msgs::PrimitiveAction>(data);
    }

    void MotionPlanningFrame::loadActionFromData(std::vector<apc_msgs::PrimitiveAction>& actions,
                                                 const std::vector<QVariant>& data)
    {
        actions.resize(data.size());
        for (int i = 0; i < data.size(); i++)
            loadActionFromData(actions[i], data[i]);
    }

    void MotionPlanningFrame::snapStateToPoint(robot_state::RobotState& state,
                                               const std::vector<std::string>& joint_names,
                                               const trajectory_msgs::JointTrajectoryPoint& point)
    {
        for (int i = 0; i < joint_names.size(); i++)
            state.setJointPositions(joint_names[i], &point.positions[i]);
        state.update();
    }

    void MotionPlanningFrame::snapStateToFrame(robot_state::RobotState& state,
                                               const std::string& frame,
                                               const std::string& link,
                                               const geometry_msgs::Pose& pose_frame_link,
                                               const std::string& group)
    {
        if (frame.empty() || link.empty())
            return;
        Eigen::Affine3d T_frame_world = computeNearestFrame(frame, group, state);
        Eigen::Affine3d T_frame_link;
        tf::poseMsgToEigen(pose_frame_link, T_frame_link);
        snapStateToFrame(state, T_frame_world, link, T_frame_link, group);
    }

    void MotionPlanningFrame::snapStateToFrame(robot_state::RobotState& state,
                                               const Eigen::Affine3d& T_frame_world,
                                               const std::string& link,
                                               const Eigen::Affine3d& T_frame_link,
                                               const std::string& group)
    {
        ROS_DEBUG_FUNCTION;

        // std::cout << "T_frame_world:\n" << T_frame_world.matrix() << std::endl;
        // std::cout << "T_frame_link:\n" << T_frame_link.matrix() << std::endl;

        // Back out the desired link transform in global coordinates.
        Eigen::Affine3d T_link_world = T_frame_world * T_frame_link.inverse();
        // Snap to the frame using IK.
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group);

        // std::cout << "group: " << group << std::endl;

        state.update();
        trajectory_msgs::JointTrajectoryPoint point;
        state.copyJointGroupPositions(jmg, point.positions);
        ROS_DEBUG_STREAM("pre-ik positions:\n" << point);

        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(T_link_world, pose);
        state.setFromIK(jmg, pose, link);
        state.update();

        state.copyJointGroupPositions(jmg, point.positions);
        ROS_DEBUG_STREAM("post-ik positions:\n" << point);

        ROS_DEBUG_FUNCTION;
    }

    void MotionPlanningFrame::attachObjectToState(robot_state::RobotState& state,
                                                  const std::string& object_id,
                                                  const std::string& link_id,
                                                  const geometry_msgs::Pose& pose_object_link)
    {
        state.clearAttachedBodies();
        if (object_id.empty() || link_id.empty())
            return;
        Eigen::Affine3d T_object_link;
        tf::poseMsgToEigen(pose_object_link, T_object_link);
        attachObjectToState(state, object_id, link_id, T_object_link);
    }

    void MotionPlanningFrame::attachObjectToState(robot_state::RobotState& state,
                                                  const std::string& object_id,
                                                  const std::string& link_id,
                                                  const Eigen::Affine3d& T_object_link)
    {
        state.clearAttachedBodies();
        if (object_id.empty())
            return;

        // Compute the object key.
        std::string object_key = computeNearestObject(object_id, link_id, state);

        // Extract the object from the world.
        collision_detection::CollisionWorld::ObjectConstPtr object;
        std::vector<shapes::ShapeConstPtr> shapes;
        EigenSTL::vector_Affine3d poses;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            object = ps->getWorld()->getObject(object_key);
        }
        if (!object)
        {
            if (!object_id.empty())
                ROS_WARN("Failed to attach %s to %s. Object does not exist.",
                         object_id.c_str(), link_id.c_str());
            return;
        }

        // Compute poses relative to link.
        shapes = object->shapes_;
        poses = object->shape_poses_;
        for (int i = 0; i < poses.size(); i++)
            poses[i] = T_object_link * poses[0].inverse() * poses[i];
        // Attach collision object to robot.
        moveit_msgs::AttachedCollisionObject aco; // For dummy arguments.
        state.attachBody(object_id, shapes, poses, aco.touch_links, link_id, aco.detach_posture);
        // Update state.
        state.update();
    }

    void MotionPlanningFrame::loadStartAndGoalFromAction(robot_state::RobotState& start,
                                                         robot_state::RobotState& goal,
                                                         const apc_msgs::PrimitiveAction& action)
    {
        int num_points = action.joint_trajectory.points.size();
        const std::vector<std::string>& joint_names = action.joint_trajectory.joint_names;
        if (joint_names.size() == 0 || num_points == 0)
        {
            ROS_ERROR("Invalid action: No joint trajectory to load!");
            return;
        }

        // Next snap start and goal states to joint trajectory points.
        const trajectory_msgs::JointTrajectoryPoint start_point = action.joint_trajectory.points.front();
        const trajectory_msgs::JointTrajectoryPoint goal_point  = action.joint_trajectory.points.back();
        // If it's "bin", do IK from current state instead of recorded
        // as it's closer to the nearest bin by definition.
        if (action.frame_id != "bin")
        {
            snapStateToPoint(start, joint_names, start_point);
            snapStateToPoint(goal, joint_names, goal_point);
        }

        // Next snap start and goal states to the frame. Note that we
        // must provide the same transform from from world to frame to
        // both snaps.
        if (!action.frame_id.empty() && !action.eef_link_id.empty())
        {
            int num_eef = action.eef_trajectory.poses.size();
            // Use compute the frame nearest the *start* state. Or
            // else, the goal state will shift itself around.
            Eigen::Affine3d T_frame_world = computeNearestFrame(action.frame_id, action.group_id, start);
            Eigen::Affine3d T_frame_start;
            Eigen::Affine3d T_frame_goal;
            geometry_msgs::Pose pose_start = action.eef_trajectory.poses[0];
            geometry_msgs::Pose pose_goal = action.eef_trajectory.poses[num_eef-1];
            tf::poseMsgToEigen(pose_start, T_frame_start);
            tf::poseMsgToEigen(pose_goal, T_frame_goal);

            snapStateToFrame(start, T_frame_world, action.eef_link_id, T_frame_start, action.group_id);
            snapStateToFrame(goal, T_frame_world, action.eef_link_id, T_frame_goal, action.group_id);
        }

        // Attach manipulated objects.
        int num_object_poses = action.object_trajectory.poses.size();
        attachObjectToState(start, action.object_id, action.attached_link_id, action.object_trajectory.poses[0]);
        attachObjectToState(goal, action.object_id, action.attached_link_id, action.object_trajectory.poses[num_object_poses-1]);
    }

    void MotionPlanningFrame::loadStartAndGoalFromAction(const apc_msgs::PrimitiveAction& action)
    {
        robot_state::RobotState start = *getQueryStartState();
        robot_state::RobotState goal = *getQueryGoalState();
        loadStartAndGoalFromAction(start, goal, action);
        planning_display_->setQueryStartState(start);
        planning_display_->setQueryGoalState(goal);
    }

    void MotionPlanningFrame::loadWaypointsToDisplay(QList<QListWidgetItem*> items)
    {
        std::vector<apc_msgs::PrimitiveAction> actions(items.count());
        for (int i = 0; i < items.count(); i++)
            loadActionFromData(actions[i], items[i]->data(Qt::UserRole));
        loadWaypointsToDisplay(actions);
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
        std::vector<apc_msgs::PrimitiveAction> actions;
        loadActionFromData(actions, data);
        loadWaypointsToDisplay(actions);
    }

    void MotionPlanningFrame::loadWaypointsToDisplay(std::vector<apc_msgs::PrimitiveAction>& actions)
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
        robot_state::RobotState start_waypoint = *getQueryStartState();
        robot_state::RobotState goal_waypoint = *getQueryGoalState();
        for (int i = 0; i < actions.size(); i++)
        {
            // Get the ith action message in the data vector.
            const apc_msgs::PrimitiveAction& action = actions[i];

            // Get the group name.
            group = action.group_id;

            // Load action into state.
            loadStartAndGoalFromAction(start_waypoint, goal_waypoint, action);

            // Update the dirty transforms, etc.
            start_waypoint.update();
            goal_waypoint.update();

            // Add waypoint.
            planning_display_->addDisplayWaypoint(start_waypoint, group, link_names, focus);
            planning_display_->addDisplayWaypoint(goal_waypoint, group, link_names, focus);
        }
    }

}
