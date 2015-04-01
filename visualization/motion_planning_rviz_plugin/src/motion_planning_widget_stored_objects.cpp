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
#include <apc_msgs/StoredScene.h>
#include <eigen_conversions/eigen_msg.h>


namespace moveit_rviz_plugin
{

    QString toDebug2(const QByteArray & line) {

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

    std::string MotionPlanningFrame::computeObjectScenePath()
    {
        // Get string from objects path line edit.
        std::string path = ui_->objects_path_line_edit->text().toStdString();
        // Extract the path components.
        std::vector<std::string> tokens;
        boost::split(tokens, path, boost::is_any_of("/"));
        // Get path to package (if it exists)
        std::string pkg_path = ros::package::getPath(tokens[0]);
        // Concatenate the package path with the original path.
        std::stringstream path_stream;
        path_stream << pkg_path;
        for (int i = 1; i < tokens.size(); i++)
            path_stream << "/" << tokens[i];
        return path_stream.str();
    }

    void MotionPlanningFrame::computeLoadObjectsButtonClicked()
    {
        // Get path to stored scene file.
        std::string path = computeObjectScenePath();

        // Clear all objects from the world.
        {
            planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
            if (!ps)
                return;
            ps->getWorldNonConst()->clearObjects();
            ps->getCurrentStateNonConst().clearAttachedBodies();
        }

        // Read in file and deserialize into stored scene message.
        QFile file(path.c_str());
        if (file.fileName().isEmpty())
            return;
        file.open(QIODevice::ReadOnly);
        QByteArray byte_array = file.readAll();
        file.close();

        // ROS_INFO_STREAM(toDebug2(byte_array).toStdString());

        apc_msgs::StoredScene scene = deserializeMessage<apc_msgs::StoredScene>(byte_array);

        // Load all objects in the scene.
        for (int i = 0; i < scene.objects.size(); i++)
        {
            Eigen::Affine3d pose;
            tf::poseMsgToEigen(scene.objects[i].poses[0], pose);
            importResource(scene.objects[i].path,
                           scene.objects[i].name,
                           pose);
        }

        // Update object lists.
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computePopulateObjects, this));
    }

    void MotionPlanningFrame::computeSaveObjectsButtonClicked()
    {
        // Get path to stored scene file.
        std::string path = computeObjectScenePath();

        // Create a scene message.
        apc_msgs::StoredScene scene;

        // Populate the stored scene message from world objects.
        planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
        if (!ps)
            return;
        const std::vector<std::string>& object_names = ps->getWorld()->getObjectIds();
        for (int i = 0; i < object_names.size(); i++)
        {
            apc_msgs::StoredObject stored_object;
            stored_object.name = object_names[i];
            stored_object.path = object_paths_[object_names[i]];
            collision_detection::World::ObjectConstPtr object = ps->getWorld()->getObject(object_names[i]);
            stored_object.poses.resize(object->shape_poses_.size());
            for (int j = 0; j < object->shape_poses_.size(); j++)
                tf::poseEigenToMsg(object->shape_poses_[j], stored_object.poses[j]);
            scene.objects.push_back(stored_object);
        }

        // Populate the stored scene message from attached objects. We do not
        // store attachment meta-data.
        const robot_state::RobotState &cs = ps->getCurrentState();
        std::vector<const robot_state::AttachedBody*> attached_bodies;
        cs.getAttachedBodies(attached_bodies);
        for (int i = 0; i < attached_bodies.size(); i++)
        {
            apc_msgs::StoredObject stored_object;
            stored_object.name = attached_bodies[i]->getName();
            stored_object.path = object_paths_[attached_bodies[i]->getName()];
            const EigenSTL::vector_Affine3d& poses = attached_bodies[i]->getGlobalCollisionBodyTransforms();
            stored_object.poses.resize(poses.size());
            for (int j = 0; j < poses.size(); j++)
                tf::poseEigenToMsg(poses[j], stored_object.poses[j]);
            scene.objects.push_back(stored_object);
        }

        // Serialize scene message and save to disk.
        QByteArray byte_array = serializeMessage<apc_msgs::StoredScene>(scene);
        QFile file(path.c_str());
        if (file.fileName().isEmpty())
            return;
        file.open(QIODevice::WriteOnly);
        file.write(byte_array);
        file.close();

        // Debug print
        // ROS_INFO_STREAM(toDebug2(byte_array).toStdString());
    }

    void MotionPlanningFrame::computePopulateObjects()
    {
        // Disable updates and signals for the object list temporarily.
        ui_->object_list_widget->setUpdatesEnabled(false);
        bool old_state = ui_->object_list_widget->blockSignals(true);

        // Get the object list.
        QListWidget* object_list = ui_->object_list_widget;

        // Store the selected items.
        std::set<std::string> objects_to_select;
        {
            QList<QListWidgetItem*> select = object_list->selectedItems();
            for (int i = 0; i < select.size(); i++)
                objects_to_select.insert(select[i]->text().toStdString());
        }

        // Clear all objects.
        object_list->clear();
        attached_objects_.clear();

        // Get planning scene.
        planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
        if (!ps)
            return;

        // Rebuild object list.
        const std::vector<std::string> &object_names = ps->getWorld()->getObjectIds();
        for (std::size_t i = 0 ; i < object_names.size() ; ++i)
        {
            if (object_names[i] == planning_scene::PlanningScene::OCTOMAP_NS)
                continue;
            QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(object_names[i]),
                                                        object_list, (int)i);
            item->setFlags(item->flags() | Qt::ItemIsEditable);
            item->setToolTip(item->text());
            item->setCheckState(Qt::Unchecked);
            if (objects_to_select.find(object_names[i]) != objects_to_select.end())
                item->setSelected(true);
            object_list->addItem(item);
            attached_objects_.push_back(std::make_pair(object_names[i], false));
        }

        // Attach objects.
        const robot_state::RobotState &cs = ps->getCurrentState();
        std::vector<const robot_state::AttachedBody*> attached_bodies;
        cs.getAttachedBodies(attached_bodies);
        for (std::size_t i = 0 ; i < attached_bodies.size() ; ++i)
        {
            QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(attached_bodies[i]->getName()),
                                                        object_list, (int)(i + object_names.size()));
            item->setFlags(item->flags() | Qt::ItemIsEditable);
            item->setToolTip(item->text());
            item->setCheckState(Qt::Checked);
            if (objects_to_select.find(attached_bodies[i]->getName()) != objects_to_select.end())
                item->setSelected(true);
            object_list->addItem(item);
            attached_objects_.push_back(std::make_pair(attached_bodies[i]->getName(), true));
        }

        // Re-enable updates and signals.
        ui_->object_list_widget->blockSignals(old_state);
        ui_->object_list_widget->setUpdatesEnabled(true);

        // Re-select object. FIXME Called from elsewhere?
        // selectedCollisionObjectChanged();
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
