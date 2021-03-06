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
#include <apc_msgs/ComputeIk.h>
#include <apc_msgs/CheckCollisions.h>
#include <moveit/robot_state/conversions.h>
#include <apc/planning.h>


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
            if (i == 0)
                widget->item(i, 0)->setData(Qt::UserRole, QString::fromStdString("kiva_pod")); // HACK
            if (i == 1)
                widget->item(i, 0)->setData(Qt::UserRole, QString::fromStdString("order_bin")); // HACK
            if (i == 2)
                widget->item(i, 0)->setData(Qt::UserRole, QString::fromStdString("work_table")); // HACK
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
            //
            bool push_back = !(ui_->json_table_widget->findItems(QString::fromStdString(item), Qt::MatchExactly).size() > 0);
            // Load/add item to sc
            addItemToScene(item_model_path, item_key, bin, push_back);
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
            -0.00814543,     0.999967, -1.00864e-06,    -0.455882,
            -0.999967,  -0.00814543,  2.28449e-06,   -0.0150576,
            2.2762e-06,  1.02722e-06,            1,     0.505588,
            0,            0,            0,            1;
            // 1, 0, 0, -0.4,
            // 0, 1, 0, -0.8,
            // 0, 0, 1,  0.7,
            // 0, 0, 0,  1;
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
            0, 0, 1,  -0.23675,
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
        item_pose.matrix() <<
            0.999796,   -0.0201899,  4.50288e-10,     -1.25476,
            0.0201899,     0.999796,  2.12423e-09,   -0.0194015,
            -4.93084e-10, -2.11471e-09,            1,   -0.0294048,
            0,            0,            0,            1;

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
                                             const std::string& item_bin,
                                             bool push_back)
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

        // compute pose of the object.
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

            if (push_back)
                T_bin.translate(Eigen::Vector3d(-2 + d_x, 0, 0)); //-bin_height / 2.0 + bin_height/5.0));
            else
                T_bin.translate(Eigen::Vector3d(d_x, 0, -item_height / 4.0));
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

        ROS_INFO_STREAM("Marker for " << item_key << " at\n" << pose.matrix());

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

        // // Get all grasps for an item.
        // std::vector<apc_msgs::PrimitivePlan> item_grasps;
        // retrieveItemGrasps(item_grasps, item_id);

        //
        apc_msgs::PrimitivePlan grasps = getPrimitivePlanFromActiveActions();
        std::vector<apc_msgs::PrimitivePlan> item_grasps;
        for (int i = 0; i < grasps.actions.size(); i++) {
            apc_msgs::PrimitivePlan grasp;
            grasp.actions.push_back(grasps.actions[i]);
            item_grasps.push_back(grasp);
        }

        // Snap all grasps to the correct locations.
        robot_state::RobotState robot_state = *getQueryStartState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        for (int i = 0; i < item_grasps.size(); i++) {
            try {
                APC_ASSERT(item_grasps[i].actions.size() == 1, "Bad grasp, too many actions");
                computeNearestFrameAndObjectKeysPartial(robot_state, world_state, item_grasps[i]);
                computeActionJointTrajectoryPoints(robot_state, world_state, item_grasps[i]);
            } catch (apc_exception::Exception& error) {
                ROS_ERROR("Skipping over grasp %s\n%s", item_grasps[i].plan_name.c_str(), error.what());
            }
        }

        try {
            computePreGrasps(item_grasps, robot_state, world_state);
        } catch (apc_exception::Exception& e) {
            ROS_ERROR("%s", e.what());
        }

        apc_msgs::PrimitivePlan ogs;
        for (int i = 0; i < item_grasps.size(); i++) {
            ogs.actions.insert(ogs.actions.end(), item_grasps[i].actions.begin(),
                               item_grasps[i].actions.end());
        }
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ogs));
        // display trajectory
        {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(robot_state, start_state_msg);
            loadPlanToPreview(start_state_msg, ogs);
            // loadPlanToActiveActions(plan);
        }
    }

    void MotionPlanningFrame::computeTestIkButtonClicked()
    {
        apc_msgs::ComputeIk srv;
        // Read off the highlighted item in the bin contents.
        int row = ui_->bin_contents_table_widget->currentRow();
        if (row < 0) return;
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        std::string item_id = bin_contents->item(row, 1)->text().toStdString();
        std::string item_key = bin_contents->item(row, 1)->data(Qt::UserRole).toString().toStdString();

        // Get all grasps for an item.
        std::vector<apc_msgs::PrimitivePlan> item_grasps;
        // retrieveItemGrasps(item_grasps, item_id);
        retrieveBinPoses(item_grasps, "bin_A");

        // Provide all grasps with keys.
        robot_state::RobotState robot_state = *getQueryStartState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        for (int i = 0; i < item_grasps.size(); i++) {
            try {
                // APC_ASSERT(item_grasps[i].actions.size() == 1, "Bad grasp, too many actions");
                computeNearestFrameAndObjectKeys(robot_state, world_state, item_grasps[i]);
                // computeActionJointTrajectoryPoints(robot_state, world_state, item_grasps[i]); // Don't snap to IK location!
                srv.request.actions.push_back(item_grasps[i].actions[1]);
            } catch (apc_exception::Exception& error) {
                ROS_ERROR("Skipping over grasp %s\n%s", item_grasps[i].plan_name.c_str(), error.what());
            }
        }

        // Add bin items.
        setWorldKeyPoseToWorldStateMessage(world_state, srv.request.world_state);
        setBinStatesToBinStatesMessage(srv.request.bin_states, world_state);
        robotStateToRobotStateMsg(robot_state, srv.request.robot_state);

        ROS_INFO("Computing test IK");

        try {
            APC_ASSERT(compute_ik_client_.call(srv),
                       "Failed call to compute ik client");
            apc_msgs::PrimitivePlan ik_actions;
            ik_actions.actions = srv.response.actions;
            planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ik_actions));
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Caught error\n%s", error.what());
        }
    }

    void MotionPlanningFrame::computeTestCollisionsButtonClicked()
    {
        apc_msgs::CheckCollisions srv;
        // Read off the highlighted item in the bin contents.
        int row = ui_->bin_contents_table_widget->currentRow();
        if (row < 0) return;
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        std::string bin_id = bin_contents->item(row, 0)->text().toStdString();
        std::string item_id = bin_contents->item(row, 1)->text().toStdString();
        std::string item_key = bin_contents->item(row, 1)->data(Qt::UserRole).toString().toStdString();

        // Get all grasps for an item.
        std::vector<apc_msgs::PrimitivePlan> db_grasps;
        retrieveItemGrasps(db_grasps, item_id);

        // Compute the offset grasps.
        std::vector<apc_msgs::PrimitivePlan> item_grasps;
        robot_state::RobotState robot_state = *getQueryGoalState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        int vanilla_grasp_count = db_grasps.size();
        for (int i = 0; i < vanilla_grasp_count; i++) {
            apc_msgs::PrimitivePlan grasp = db_grasps[i];
            try {
                computeNearestFrameAndObjectKeys(robot_state, world_state, grasp);
                computeActionJointTrajectoryPoints(robot_state, world_state, grasp);
                ROS_INFO("grasp key %s", grasp.actions[0].object_key.c_str());
                item_grasps.push_back(grasp);
                computeOffsetGrasps(item_grasps, grasp, robot_state, world_state);
            } catch (apc_exception::Exception& error) {
                ROS_ERROR("Skipping over grasp %s\n%s", db_grasps[i].plan_name.c_str(), error.what());
            }
        }

        try {
            computeCheckCollisions(item_grasps,
                                   robot_state,
                                   world_state);
        } catch (apc_exception::Exception& eror) {
            ROS_ERROR("ERror %s", eror.what());
        }

        try {
            std::sort(item_grasps.begin(), item_grasps.end(), apc_planning::less_than_dot_x(robot_state, world_state, bin_id));
        } catch (apc_exception::Exception& e) {
            ROS_ERROR("Failed to sort %s", e.what());
        }

        apc_msgs::PrimitivePlan ogs;
        for (int i = 0; i < item_grasps.size(); i++) {
            ogs.actions.insert(ogs.actions.end(), item_grasps[i].actions.begin(),
                               item_grasps[i].actions.end());
        }
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ogs));
        // display trajectory
                {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(robot_state, start_state_msg);
            loadPlanToPreview(start_state_msg, ogs);
            // loadPlanToActiveActions(plan);
        }
    }

    void MotionPlanningFrame::computeTestGraspsButtonClicked()
    {
        // Aggregate the active actions into a plan.
        apc_msgs::PrimitivePlan grasp = getPrimitivePlanFromActiveActions();

        // Get the starting state.
        robot_state::RobotState start_state = planning_display_->getPlanningSceneRO()->getCurrentState();

        // Compute the offset grasps.
        std::vector<apc_msgs::PrimitivePlan> offset_grasps;
        offset_grasps.push_back(grasp);
        try {
            KeyPoseMap world_state = computeWorldKeyPoseMap();
            computeNearestFrameAndObjectKeys(start_state, world_state, grasp);
            computeActionJointTrajectoryPoints(start_state, world_state, grasp);
            computeOffsetGrasps(offset_grasps, grasp, start_state, world_state);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Error: %s", error.what());
        }
        ROS_INFO_STREAM("offset_grasps: " << offset_grasps.size());
        apc_msgs::PrimitivePlan ogs;
        for (int i = 0; i < offset_grasps.size(); i++) {
            ogs.actions.insert(ogs.actions.end(), offset_grasps[i].actions.begin(),
                               offset_grasps[i].actions.end());
        }
        ROS_INFO_STREAM("ogs: " << ogs.actions.size());
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ogs));
    }

    void MotionPlanningFrame::computeTestEnterButtonClicked()
    {
        typedef std::vector<apc_msgs::PrimitivePlan> PlanList;
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;
        // Get the to bin pose.
        int row = ui_->bin_contents_table_widget->currentRow();
        if (row < 0) return;
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        std::string bin_id = bin_contents->item(row, 0)->text().toStdString();
        // Retrieve bin pose.
        PlanList bin_poses;
        retrieveBinPoses(bin_poses, bin_id);
        // Get the pregrasps.
        Plan aa = getPrimitivePlanFromActiveActions();
        // Convert to list.
        PlanList pregrasps;
        for (int i = 0; i < aa.actions.size(); i++) {
            Plan pg;
            pg.actions.push_back(aa.actions[i]);
            pregrasps.push_back(pg);
        }
        // Get things to call functionw ith.
        robot_state::RobotState robot_state = *getQueryGoalState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        try {
            computeEnter(pregrasps, bin_poses[0], robot_state, world_state);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Error:%s", error.what());
        }
        apc_msgs::PrimitivePlan ogs;
        for (int i = 0; i < pregrasps.size(); i++) {
            ogs.actions.insert(ogs.actions.end(), pregrasps[i].actions.begin(),
                               pregrasps[i].actions.end());
        }
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ogs));
        // display trajectory
                {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(robot_state, start_state_msg);
            loadPlanToPreview(start_state_msg, ogs);
            // loadPlanToActiveActions(plan);
        }
    }

    void MotionPlanningFrame::computeTestExitButtonClicked()
    {
                typedef std::vector<apc_msgs::PrimitivePlan> PlanList;
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;
        // Get the to bin pose.
        int row = ui_->bin_contents_table_widget->currentRow();
        if (row < 0) return;
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        std::string bin_id = bin_contents->item(row, 0)->text().toStdString();
        // Retrieve bin pose.
        PlanList bin_poses;
        retrieveBinPoses(bin_poses, bin_id);
        // Get the pregrasps.
        Plan aa = getPrimitivePlanFromActiveActions();
        // Convert to list.
        PlanList pregrasps;
        for (int i = 0; i < aa.actions.size(); i++) {
            Plan pg;
            pg.actions.push_back(aa.actions[i]);
            pregrasps.push_back(pg);
        }
        // Get things to call functionw ith.
        robot_state::RobotState robot_state = *getQueryGoalState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        try {
            computeExit(pregrasps, bin_poses[0], robot_state, world_state);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Error:%s", error.what());
        }
        apc_msgs::PrimitivePlan ogs;
        for (int i = 0; i < pregrasps.size(); i++) {
            ogs.actions.insert(ogs.actions.end(), pregrasps[i].actions.begin(),
                               pregrasps[i].actions.end());
        }
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ogs));
        // display trajectory
                {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(robot_state, start_state_msg);
            loadPlanToPreview(start_state_msg, ogs);
            // loadPlanToActiveActions(plan);
        }
    }

    void MotionPlanningFrame::computeTestPlanButtonClicked()
    {
                typedef std::vector<apc_msgs::PrimitivePlan> PlanList;
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;
        // Get the to bin pose.
        int row = ui_->bin_contents_table_widget->currentRow();
        if (row < 0) return;
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        std::string bin_id = bin_contents->item(row, 0)->text().toStdString();
        std::string item_id = bin_contents->item(row, 1)->text().toStdString();
        std::string item_key = bin_contents->item(row, 1)->data(Qt::UserRole).toString().toStdString();
        // Get things to call functionw ith.
        PlanList picks;
        robot_state::RobotState robot_state = planning_display_->getPlanningSceneRO()->getCurrentState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        try {
            PlanList bin_poses;
            retrieveBinPoses(bin_poses, bin_id);
            computePick(picks, item_id, bin_id, bin_poses[0], robot_state, world_state);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Error:%s", error.what());
        }
        apc_msgs::PrimitivePlan ogs;
        for (int i = 0; i < picks.size(); i++) {
            ogs.actions.insert(ogs.actions.end(), picks[i].actions.begin(),
                               picks[i].actions.end());
        }
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ogs));
        // display trajectory
                {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(robot_state, start_state_msg);
            loadPlanToPreview(start_state_msg, ogs);
            // loadPlanToActiveActions(plan);
        }
    }
    void MotionPlanningFrame::computeTestTrajoptButtonClicked()
    {
    }

    void
    MotionPlanningFrame::computeFixGraspsButtonClicked()
    {
        typedef std::vector<apc_msgs::PrimitivePlan> PlanList;
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;
        robot_state::RobotState robot_state = *getQueryGoalState();
        PlanList grasps;
        grasps = findMatchingPlansAny("", "grasp.*", ".*", ".*", ".*", ".*", true);
        PlanList invalid_grasps;
        for (int i = 0; i < grasps.size(); i++) {
            try {
                apc_planning::assertGraspPreconditions(grasps[i], robot_state);
            } catch (apc_exception::Exception& e) {
                ROS_DEBUG("INVALID -> grasp[%d]: \n%s\n%s", i, apc_planning::toStringNoArr(grasps[i]).c_str(),
                         e.what());
                invalid_grasps.push_back(grasps[i]);
                // if (invalid_grasps.back().actions.size() == 1)
                //     break;
            }
        }
        ROS_INFO("Found %ld invalid grasps", invalid_grasps.size());
        PlanList fixed_grasps;

        KeyPoseMap world_state = computeWorldKeyPoseMap();
        for (int i = 0; i < invalid_grasps.size(); i++) {
            try {
                apc_planning::fixGrasp(invalid_grasps[i], robot_state, world_state);
                apc_planning::assertGraspPreconditions(invalid_grasps[i], robot_state);
                fixed_grasps.push_back(invalid_grasps[i]);
            } catch (apc_exception::Exception& e) {
                ROS_WARN("INVALID -> grasp[%d]: %s\n%s", i, grasps[i].plan_name.c_str(), e.what());
            }
        }
        ROS_INFO("Fixed %ld invalid grasps", fixed_grasps.size());
        // Load to display.
        Plan I;
        apc_planning::convertPlanListToPlanActions(invalid_grasps, I);
        Action divider;
        divider.action_name = "---";
        I.actions.push_back(divider);
        apc_planning::convertPlanListToPlanActions(fixed_grasps, I);
        loadPlanToActiveActions(I);
        // Save fixed actions back to list.
        overwriteStoredPlans(fixed_grasps);
    }

}
