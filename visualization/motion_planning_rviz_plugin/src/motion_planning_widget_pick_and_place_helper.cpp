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
#include <ros/package.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <interactive_markers/tools.h>
#include <robot_calibration/urdf_loader.h>
#include <boost/xpressive/xpressive.hpp>
#include <apc_msgs/ComputePreGrasps.h>


namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::computeLoadBinContentsToScene()
    {
        // Get a list of the objects and their bins.
        typedef std::pair<std::string, std::string> BinItem;
        typedef std::vector<BinItem> BinContents;
        typedef std::vector<std::string> Keys;
        BinContents bin_contents;
        QTableWidget* widget = ui_->bin_contents_table_widget;
        for (int i = 0; i < widget->rowCount(); i++)
        {
            std::string bin = widget->item(i, 0)->text().toStdString();
            std::string item = widget->item(i, 1)->text().toStdString();
            bin_contents.push_back(BinItem(bin, item));
        }
        // Load bin contents to scene and save keys into widget items.
        Keys keys = computeLoadBinContentsToScene(bin_contents);
        for (int i = 0; i < widget->rowCount(); i++)
        {
            widget->item(i, 0)->setData(Qt::UserRole, QString::fromStdString("kiva_pod")); // HACK
            widget->item(i, 1)->setData(Qt::UserRole, QString::fromStdString(keys[i]));
        }
        // Set current row to none.
        widget->setCurrentCell(0,0, QItemSelectionModel::Clear);
        // Redraw scene geometry.
        planning_display_->queueRenderSceneGeometry();
        // Call items loaded hook.
        itemsLoadedHook();
    }

    std::vector<std::string>
    MotionPlanningFrame::computeLoadBinContentsToScene(const std::vector<std::pair<std::string, std::string> >& bin_contents)
    {
        typedef std::pair<std::string, std::string> BinItem;
        typedef std::vector<BinItem> BinContents;
        typedef std::map<std::string, int> ItemCount;
        typedef std::vector<std::string> Keys;
        // Load KIVA pod to scene.
        loadKivaPodToScene();
        // Load order bin to scene.
        loadOrderBinToScene();
        // Load work table to scene.
        loadWorkTableToScene();
        // Add items to scene.
        ItemCount item_count;
        Keys keys;
        for (BinContents::const_iterator iter = bin_contents.begin(); iter != bin_contents.end(); ++iter)
        {
            // Generate item model path and item scene key.
            std::string bin = iter->first;
            std::string item = iter->second;
            std::string item_model_path = computeItemModelPath(item);
            std::string item_key = computeItemSceneKey(item, item_count[item]++);
            // Load/add item to scene.
            addItemToScene(item_model_path, item_key, bin);
            // Store the item key.
            keys.push_back(item_key);
        }
        // Remove extra items from scene.
        for (ItemCount::iterator iter = _bin_item_counts.begin(); iter != _bin_item_counts.end(); ++iter)
            for (int i = item_count[iter->first]; i < iter->second; i++)
                removeItemFromScene(computeItemSceneKey(iter->first, i));
        _bin_item_counts = item_count;
        // Return keys.
        return keys;
    }

    void MotionPlanningFrame::loadOrderBinToScene()
    {
        std::string object_key = "order_bin";
        std::string object_model_path = computeItemModelPath("order_bin");
        Eigen::Matrix4d T_object_world;
        T_object_world.matrix() <<
            1, 0, 0, -0.4,
            0, 1, 0, -0.8,
            0, 0, 1,  0.7,
            0, 0, 0,  1;
        loadObjectToScene(object_key, object_model_path, Eigen::Affine3d(T_object_world));
    }

    void MotionPlanningFrame::loadWorkTableToScene()
    {
        std::string object_key = "work_table";
        std::string object_model_path = computeItemModelPath("work_table");
        Eigen::Affine3d T_object_world;
        T_object_world.matrix() <<
            1, 0, 0, -0.4,
            0, 1, 0,  0.0,
            0, 0, 1,  0.0,
            0, 0, 0,  1;
        loadObjectToScene(object_key, object_model_path, Eigen::Affine3d(T_object_world));
    }

    void MotionPlanningFrame::loadObjectToScene(const std::string& object_key,
                                                const std::string& object_model_path,
                                                const Eigen::Affine3d& T_object_world)
    {
        // Determine if we need to load and add the object.
        bool add_object = false;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            add_object = ps->getWorld()->hasObject(object_key) == false;
        }

        // Retrieve the object's mesh shape.
        shapes::ShapeConstPtr object_shape;
        if (add_object)
        {
            std::string object_model_uri = "file://" + object_model_path;
            shapes::Mesh* object_mesh = shapes::createMeshFromResource(object_model_uri);
            if (!object_mesh)
            {
                ROS_ERROR("Failed to load URI: %s", object_model_uri.c_str());
                return;
            }
            object_shape.reset(object_mesh);
        }
        else
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            object_shape = ps->getWorld()->getObject(object_key)->shapes_[0];
        }

        // Add or move object to scene.
        {
            planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
            collision_detection::WorldPtr world = ps->getWorldNonConst();
            if (add_object)
                world->addToObject(object_key, object_shape, T_object_world);
            else
                world->moveShapeInObject(object_key, object_shape, T_object_world);
        }
    }

    void MotionPlanningFrame::loadKivaPodToScene()
    {
        if (!planning_display_->getPlanningSceneMonitor())
        {
            ROS_ERROR("Failed to load KIVA pod: No planning scene monitor");
            return;
        }
        // Load KIVA pod into planning scene.
        if (!_kiva_pod)
        {
            _kiva_pod.reset(new robot_calibration::Robotd);
            robot_calibration::LoadUrdf(_kiva_pod.get(), "package://apc_description/urdf/kiva_pod/kiva_pod.urdf", true);
            _kiva_pod->update();
        }
        std::string item_key = "kiva_pod";
        shapes::ShapeConstPtr item_shape = _kiva_pod->getRootLink()->getState().shapes[0];
        Eigen::Isometry3d item_pose = Eigen::Isometry3d::Identity();
        item_pose.translation() = Eigen::Vector3d(-1.2, 0, 0);

        planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
        collision_detection::WorldPtr world = ps->getWorldNonConst();
        bool add_item = !world->hasObject(item_key);
        if (add_item)
            world->addToObject(item_key, item_shape, item_pose);
    }

    std::string MotionPlanningFrame::computeItemModelPath(const std::string& item)
    {
        std::string path = ros::package::getPath("apc_description");
        path = path + "/objects/" + item + "/reduced_meshes/" + item + ".stl";
        return path;
    }

    std::string MotionPlanningFrame::computeItemSceneKey(const std::string& item,
                                                         const int number)
    {
        char buf[50];
        sprintf(buf, "%d", number);
        return item + "_" + buf;
    }

    std::string MotionPlanningFrame::computeItemIdFromItemKey(const std::string& item_key)
    {
        using namespace boost::xpressive;
        sregex rex = sregex::compile("^([A-Za-z]+(\\_[A-Za-z0-9]+)*?)([\\_][0-9]{1,2})?");
        smatch what;
        APC_ASSERT(regex_match(item_key, what, rex),
                   "Failed to extract ID of %s", item_key.c_str());
        std::string id = what[1];
        return id;
    }

    void MotionPlanningFrame::addItemToScene(const std::string& item_model_path,
                                             const std::string& item_key,
                                             const std::string& item_bin)
    {
        if (!planning_display_->getPlanningSceneMonitor())
        {
            ROS_ERROR("Failed to load items: No planning scene monitor");
            return;
        }

        // Determine if we need to load and add the item.
        bool add_item = false;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            add_item = ps->getWorld()->hasObject(item_key) == false;
        }

        // Retrieve the item's mesh shape.
        shapes::ShapeConstPtr item_shape;
        if (add_item)
        {
            std::string item_model_uri = "file://" + item_model_path;
            shapes::Mesh* item_mesh = shapes::createMeshFromResource(item_model_uri);
            if (!item_mesh)
            {
                ROS_ERROR("Failed to load URI: %s", item_model_uri.c_str());
                return;
            }
            item_shape.reset(item_mesh);

            Eigen::Vector3d extent = shapes::computeShapeExtents(item_shape.get());
            std::string item_id = computeItemIdFromItemKey(item_key);
            cached_shape_extents_[item_id] = extent;
        }
        else
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            item_shape = ps->getWorld()->getObject(item_key)->shapes_[0];
        }

        // Compute pose of the object.
        Eigen::Affine3d item_pose = Eigen::Affine3d::Identity();
        if (_kiva_pod)
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            Eigen::Affine3d T_pod = ps->getWorld()->getObject("kiva_pod")->shape_poses_[0];
            Eigen::Affine3d T_bin = _kiva_pod->getGlobalTransform(item_bin);
            robot_calibration::Linkd* bin = _kiva_pod->getLink(item_bin);
            double bin_height = static_cast<const shapes::Box*>(bin->getState().shapes[0].get())->size[2];
            double bin_depth = static_cast<const shapes::Box*>(bin->getState().shapes[0].get())->size[0];

            std::string item_id = computeItemIdFromItemKey(item_key);
            double item_depth = cached_shape_extents_[item_id].x();
            double item_height = cached_shape_extents_[item_id].z();

            double d_x = -item_depth / 2.0;

            T_bin.translate(Eigen::Vector3d(d_x, 0, 0)); //-bin_height / 2.0 + bin_height/5.0));
            item_pose = T_pod * T_bin;
        }

        // Add or move item to scene.
        {
            planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
            collision_detection::WorldPtr world = ps->getWorldNonConst();
            if (add_item)
                world->addToObject(item_key, item_shape, item_pose);
            else
                world->moveShapeInObject(item_key, item_shape, item_pose);
        }
    }

    void MotionPlanningFrame::removeItemFromScene(const std::string& item_key)
    {
        planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
        collision_detection::WorldPtr world = ps->getWorldNonConst();
        if (!world->hasObject(item_key))
        {
            ROS_ERROR("Failed to remove non-existent object from world: %s", item_key.c_str());
            return;
        }
        world->removeObject(item_key);
    }

    void MotionPlanningFrame::createInteractiveMarkerForItem(const std::string& item_key)
    {
        const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
        if (!ps)
        {
            ROS_ERROR("Failed to acquire planning scene");
            return;
        }

        if (_item_marker)
            _item_marker.reset();

        // Create interactive marker for the item.
        const collision_detection::CollisionWorld::ObjectConstPtr& item = ps->getWorld()->getObject(item_key);
        if (!item)
        {
            ROS_ERROR("Failed to retreive item: %s", item_key.c_str());
            return;
        }
        Eigen::Affine3d pose = item->shape_poses_[0];
        geometry_msgs::PoseStamped item_pose;
        tf::poseEigenToMsg(pose, item_pose.pose);
        Eigen::Vector3d item_extents = shapes::computeShapeExtents(item->shapes_[0].get());
        double item_scale = 1.80 * item_extents.minCoeff();
        item_scale = std::min(std::max(item_scale, 0.3), 1.0);
        visualization_msgs::InteractiveMarker marker_msg =
            robot_interaction::make6DOFMarker("marker_" + item_key, item_pose, item_scale, false);
        marker_msg.header.frame_id = context_->getFrameManager()->getFixedFrame();
        marker_msg.description = item_key;
        interactive_markers::autoComplete(marker_msg);

        rviz::InteractiveMarker* marker = new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_);
        marker->processMessage(marker_msg);
        marker->setShowAxes(false);
        _item_marker.reset(marker);
        connect(marker, SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)),
                this, SLOT(processInteractiveMarkerFeedbackForItem(visualization_msgs::InteractiveMarkerFeedback&)));
    }

    void MotionPlanningFrame::updateInteractiveMarkerForItem(float wall_dt)
    {
        if (_item_marker)
            _item_marker->update(wall_dt);
    }

    void MotionPlanningFrame::processInteractiveMarkerFeedbackForItem(visualization_msgs::InteractiveMarkerFeedback& feedback)
    {
        planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
        if (!ps)
        {
            ROS_ERROR("Failed to get planning scene for interactive marker");
            return;
        }

        // Set marker pose to scene item.
        Eigen::Affine3d item_pose;
        tf::poseMsgToEigen(feedback.pose, item_pose);
        std::string item_key = feedback.marker_name.erase(0, 7); // Remove "marker_" from marker name.
        collision_detection::CollisionWorld::ObjectConstPtr item = ps->getWorld()->getObject(item_key);
        ps->getWorldNonConst()->moveShapeInObject(item_key, item->shapes_[0], item_pose);
        // Queue render.
        planning_display_->queueRenderSceneGeometry();
    }

    void MotionPlanningFrame::computeTestPreGraspsButtonClicked()
    {
        apc_msgs::ComputePreGrasps srv;
        // Read off the highlighted item in the bin contents.
        int row = ui_->bin_contents_table_widget->currentRow();
        if (row < 0) return;
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        std::string item_id = bin_contents->item(row, 1)->text().toStdString();
        std::string item_key = bin_contents->item(row, 1)->data(Qt::UserRole).toString().toStdString();

        // Get all grasps for an item.
        std::vector<apc_msgs::PrimitivePlan> item_grasps;
        retrieveItemGrasps(item_grasps, item_id);

        // Snap all grasps to the correct locations.
        robot_state::RobotState robot_state = *getQueryStartState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        for (int i = 0; i < item_grasps.size(); i++) {
            try {
                APC_ASSERT(item_grasps[i].actions.size() == 1, "Bad grasp, too many actions");
                computeNearestFrameAndObjectKeys(robot_state, world_state, item_grasps[i]);
                computeActionJointTrajectoryPoints(robot_state, world_state, item_grasps[i]);
                srv.request.grasps.push_back(item_grasps[i].actions[0]);
            } catch (apc_exception::Exception& error) {
                ROS_ERROR("Skipping over grasp %s\n%s", item_grasps[i].plan_name.c_str(), error.what());
            }
        }

        // Add bin items.
        setWorldKeyPoseToWorldStateMessage(world_state, srv.request.world_state);
        setBinStatesToBinStatesMessage(srv.request.bin_states, world_state);

        try {
            APC_ASSERT(compute_pregrasps_client_.call(srv),
                       "Failed call to compute pregrasp client");
            apc_msgs::PrimitivePlan pregrasps;
            pregrasps.actions = srv.response.pregrasps;
            planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, pregrasps));
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Caught error\n%s", error.what());
        }
    }

}
