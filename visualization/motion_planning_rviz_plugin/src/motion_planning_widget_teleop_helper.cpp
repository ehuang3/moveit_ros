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


namespace moveit_rviz_plugin
{

    bool MotionPlanningFrame::showQueryStartInteractiveMarkers()
    {
        return !ui_->padlock_button->isChecked() && ui_->start_radio_button->isChecked();
    }

    bool MotionPlanningFrame::showQueryGoalInteractiveMarkers()
    {
        return ui_->padlock_button->isChecked() || ui_->goal_radio_button->isChecked();
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
        class QCheckBox* checkbox[num_checkboxes] = { ui_->relative_to_frame_checkbox,
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
                unsigned char options[num_checkboxes] = { action.relative_to_frame,
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
        if (!ui_->relative_to_frame_checkbox->isEnabled())
            return;

        for (int i = 0; i < data.size(); i++)
        {
            // Get message from data.
            apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[i]);

            // Fill message options.
            for (int j = 0; j < plan.actions.size(); j++)
            {
                apc_msgs::PrimitiveAction& action = plan.actions[i];
                QCheckBox* checkbox[7] = { ui_->relative_to_frame_checkbox,
                                           ui_->relative_to_pose_checkbox,
                                           ui_->eef_trajectory_checkbox,
                                           ui_->dense_trajectory_checkbox,
                                           ui_->monitor_contact_checkbox,
                                           ui_->monitor_profile_checkbox,
                                           ui_->cartesian_interpolate_checkbox };
                unsigned char* options[7] = { &action.relative_to_frame,
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

    void MotionPlanningFrame::computeAttachObjectToState(robot_state::RobotState& state,
                                                         const std::string& object_name,
                                                         const std::string& link_name)
    {
        // Extract the object from the world.
        collision_detection::CollisionWorld::ObjectConstPtr object;
        EigenSTL::vector_Affine3d poses;
        {
            planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
            object = ps->getWorld()->getObject(object_name);
        }
        if (!object)
        {
            ROS_ERROR("Failed to attach %s to %s. Object does not exist.",
                      object_name.c_str(), link_name.c_str());
            return;
        }
        poses = object->shape_poses_;
        // Transform object poses to link frame.
        Eigen::Affine3d T_inv = state.getGlobalLinkTransform(link_name).inverse();
        for (int i = 0; i < poses.size(); i++)
            poses[i] = T_inv * poses[i];
        // Attach object.
        computeAttachObjectToState(state, object_name, link_name, poses);
    }

    void MotionPlanningFrame::computeAttachObjectToState(robot_state::RobotState& state,
                                                         const std::string& object_name,
                                                         const std::string& link_name,
                                                         const std::vector<geometry_msgs::Pose>& poses)
    {
        EigenSTL::vector_Affine3d eigen_poses(poses.size());
        for (int i = 0; i < poses.size(); i++)
            tf::poseMsgToEigen(poses[i], eigen_poses[i]);
        computeAttachObjectToState(state, object_name, link_name, eigen_poses);
    }

    void MotionPlanningFrame::computeAttachObjectToState(robot_state::RobotState& state,
                                                         const std::string& object_name,
                                                         const std::string& link_name,
                                                         const EigenSTL::vector_Affine3d& poses)
    {
        // If the object name is empty, remove all attached bodies!
        if (object_name.empty())
            state.clearAttachedBodies();
        // Extract the object from the world.
        collision_detection::CollisionWorld::ObjectConstPtr object;
        std::vector<shapes::ShapeConstPtr> shapes;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            object = ps->getWorld()->getObject(object_name);
        }
        if (!object)
        {
            if (!object_name.empty())
                ROS_WARN("Failed to attach %s to %s. Object does not exist.",
                         object_name.c_str(), link_name.c_str());
            return;
        }
        shapes = object->shapes_;
        // If the state already has an attached body, remove it.
        if (state.hasAttachedBody(object_name))
            state.clearAttachedBody(object_name);
        // Attach collision object to robot.
        moveit_msgs::AttachedCollisionObject aco; // For dummy arguments.
        state.attachBody(object_name, shapes, poses, aco.touch_links, link_name, aco.detach_posture);
        // Update state.
        state.update();
    }

    void MotionPlanningFrame::computeDetachObjectFromState(robot_state::RobotState& state,
                                                           const std::string& object_name)
    {
        if (!state.hasAttachedBody(object_name))
        {
            ROS_ERROR("Failed to remove non-existent attached body %s", object_name.c_str());
            return;
        }
        state.clearAttachedBody(object_name);
    }

    void MotionPlanningFrame::computeAttachObjectToPlan(apc_msgs::PrimitivePlan& plan,
                                                        const robot_state::RobotState& state)
    {
        for (int i = 0; i < plan.actions.size(); i++)
            computeAttachObjectToAction(state, plan.actions[i]);
    }

    void MotionPlanningFrame::computeAttachObjectToAction(const robot_state::RobotState& state,
                                                          apc_msgs::PrimitiveAction& action)
    {
        std::vector<const robot_state::AttachedBody*> attached_bodies;
        state.getAttachedBodies(attached_bodies);
        // Should only be one attached body.
        if (attached_bodies.size() != 1)
        {
            ROS_WARN("Failed to attach %ld bodies", attached_bodies.size());
            return;
        }
        // Set the attached body into the action.
        for (int i = 0; i < attached_bodies.size(); i++)
        {
            std::string object_name = attached_bodies[i]->getName();
            std::string link_name   = attached_bodies[i]->getAttachedLinkName();
            EigenSTL::vector_Affine3d poses = attached_bodies[i]->getFixedTransforms();
            action.object_name = object_name;
            action.link_name = link_name;
            action.object_poses.resize(poses.size());
            for (int k = 0; k < poses.size(); k++)
                tf::poseEigenToMsg(poses[k], action.object_poses[k]);
        }
    }

    void MotionPlanningFrame::computeDetachObjectFromPlan(apc_msgs::PrimitivePlan& plan)
    {
        for (int i = 0; i < plan.actions.size(); i++)
            computeDetachObjectFromAction(plan.actions[i]);
    }

    void MotionPlanningFrame::computeDetachObjectFromAction(apc_msgs::PrimitiveAction& action)
    {
        action.object_name = "";
        action.link_name = "";
        action.object_poses.clear();
    }

}
