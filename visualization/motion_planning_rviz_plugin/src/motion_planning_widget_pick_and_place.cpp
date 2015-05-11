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
#include <moveit/robot_state/conversions.h>
#include <boost/regex.hpp>
#include <moveit/warehouse/primitive_plan_storage.h>

#include <apc/eigen.h>


namespace moveit_rviz_plugin
{

    bool MotionPlanningFrame::testForItemKey(const std::string& key)
    {
        return (key.find("bin") != 0 &&
                key.find("kiva_pod") != 0);
    }

    bool MotionPlanningFrame::testForBinKey(const std::string& key)
    {
        return key.find("bin_") == 0;
    }

    std::string MotionPlanningFrame::findNearestBinToItemKey(const std::string& item_key,
                                                             const KeyPoseMap& world)
    {
        std::string nearest_bin;
        double min_dist = 1e9;
        for (KeyPoseMap::const_iterator iter = world.begin(); iter != world.end(); ++iter) {
            if (!testForBinKey(iter->first))
                continue;
            double dist = (iter->second.translation() - world.find(item_key)->second.translation()).norm();
            if (dist < min_dist) {
                nearest_bin = iter->first;
                min_dist = dist;
            }
        }
        APC_ASSERT(!nearest_bin.empty(),
                   "Failed to find nearest bin to %s", item_key.c_str());
        return nearest_bin;
    }

    std::vector<std::string> MotionPlanningFrame::findItemKeysInBin(const std::string& bin_id,
                                                                    const KeyPoseMap& world)
    {
        APC_ASSERT(world.count(bin_id) > 0,
                   "Failed to retrieve %s from world", bin_id.c_str());
        std::vector<std::string> item_keys;
        for (KeyPoseMap::const_iterator iter = world.begin(); iter != world.end(); ++iter) {
            // Skip over non-item keys.
            if (!testForItemKey(iter->first))
                continue;
            if (bin_id == findNearestBinToItemKey(iter->first, world))
                item_keys.push_back(iter->first);
        }
        return item_keys;
    }

    std::vector<std::string> MotionPlanningFrame::findItemKeysInBinMatchingItemID(const std::string& bin_id,
                                                                                  const std::string& item_id,
                                                                                  const KeyPoseMap& world)
    {
        std::vector<std::string> bin_items = findItemKeysInBin(bin_id, world);
        std::vector<std::string> matching_items;
        for (int i = 0; i < bin_items.size(); i++)
            if (bin_items[i].find(item_id) == 0)
                matching_items.push_back(bin_items[i]);
        APC_ASSERT(matching_items.size() > 0,
                   "Failed to find target item %s in %s", item_id.c_str(), bin_id.c_str());
        return matching_items;
    }

    std::vector<srdf::Model::EndEffector> MotionPlanningFrame::getEndEffectors()
    {
        const boost::shared_ptr<const srdf::Model> &srdf = planning_display_->getRobotModel()->getSRDF();
        APC_ASSERT(srdf, "Failed to get robot model");
        const std::vector<srdf::Model::EndEffector> &eef = srdf->getEndEffectors();
        return eef;
    }

    bool MotionPlanningFrame::matchRegex(const std::string& in,
                                         const std::string& expr)
    {
        boost::cmatch what;
        boost::regex expression(expr);
        return boost::regex_match(in.c_str(), what, expression);
    }

    bool MotionPlanningFrame::matchEef(const std::string& in,
                                       const std::string& expr)
    {
        return matchRegex(in, expr);
    }

    std::vector<apc_msgs::PrimitivePlan> MotionPlanningFrame::findMatchingPlansAny(const std::string& database,
                                                                                   const std::string& plan_expr,
                                                                                   const std::string& group_expr,
                                                                                   const std::string& frame_expr,
                                                                                   const std::string& object_expr,
                                                                                   const std::string& eef_expr,
                                                                                   bool grasp)
    {
        APC_ASSERT(primitive_plan_storage_, "Failed to load database");
        // Get all plan names.
        std::vector<std::string> plan_names;
        primitive_plan_storage_->getKnownPrimitivePlans(plan_names);
        // For each plan, test for match.
        std::vector<apc_msgs::PrimitivePlan> matched_plans;
        for (int i = 0; i < plan_names.size(); i++) {
            moveit_warehouse::PrimitivePlanWithMetadata meta_plan;
            APC_ASSERT(primitive_plan_storage_->getPrimitivePlan(meta_plan, plan_names[i]),
                       "Failed to get plan %s", plan_names[i].c_str());
            const apc_msgs::PrimitivePlan& plan = *meta_plan;
            bool match_plan   = matchRegex(plan.plan_name, plan_expr);
            bool match_group  = false;
            bool match_frame  = false;
            bool match_object = false;
            bool match_eef    = false;
            bool match_grasp  = false;
            // If any action in the plan matches, then the plan matches.
            for (int j = 0; j < plan.actions.size(); j++) {
                const apc_msgs::PrimitiveAction& action = plan.actions[j];
                match_group  |= matchRegex(action.group_id, group_expr);
                match_frame  |= matchRegex(action.frame_id, frame_expr);
                match_object |= matchRegex(action.object_id, object_expr);
                match_eef    |= matchEef(action.group_id, eef_expr);
                match_grasp  |= action.grasp == grasp;
            }
            if (match_plan && match_group && match_frame && match_object && match_eef && match_grasp) {
                matched_plans.push_back(plan);
            }
        }
        APC_ASSERT(matched_plans.size() > 0,
                   "Failed to find any matching plans. Inputs are\n"
                   "        plan: %s\n"
                   "       group: %s\n"
                   "       frame: %s\n"
                   "      object: %s\n"
                   "end-effector: %s\n"
                   "       grasp: %s",
                   plan_expr.c_str(), group_expr.c_str(), frame_expr.c_str(), object_expr.c_str(), eef_expr.c_str(),
                   (grasp ? "true" : "false"));
        return matched_plans;
    }

    void MotionPlanningFrame::retrieveStartingPoses(std::vector<apc_msgs::PrimitivePlan>& starting_poses)
    {
        starting_poses = findMatchingPlansAny("", "start.*");
    }

    void MotionPlanningFrame::computeReachableStartingPoses(std::vector<apc_msgs::PrimitivePlan>& valid_starts,
                                                            const robot_state::RobotState& robot_state,
                                                            const KeyPoseMap& world_state)
    {
        // Find all plans with "start" in the plan name.
        std::vector<apc_msgs::PrimitivePlan> starting_poses;
        retrieveStartingPoses(starting_poses);

        // Verify plans from current robot state to plan state.
        valid_starts.clear();
#pragma omp parallel num_threads(8)
        {
#pragma omp for
            for (int i = 0; i < starting_poses.size(); i++) {
                try {
                    ROS_DEBUG("Testing starting pose: %s", starting_poses[i].plan_name.c_str());
                    // On failure an exception is thrown.
                    ROS_DEBUG("Testing starting pose: %s", starting_poses[i].plan_name.c_str());
                    computePlan(starting_poses[i], robot_state, world_state, i % 8);
                    // This line will only be reached if the plan was valid.
#pragma omp critical
                    {
                        valid_starts.push_back(starting_poses[i]);
                    }
                } catch (std::exception& error) {
                    ROS_DEBUG("Starting pose failed with %s", error.what());
                }
            }
        }

        APC_ASSERT(valid_starts.size() > 0,
                   "No reachable starting poses!");
    }

    void MotionPlanningFrame::retrieveBinPoses(std::vector<apc_msgs::PrimitivePlan>& bin_poses,
                                               const std::string& bin_id)
    {
        bin_poses = findMatchingPlansAny("", ".*", ".*", bin_id);
    }

    void MotionPlanningFrame::computeReachableBinPoses(std::vector<apc_msgs::PrimitivePlan>& valid_bins,
                                                       const std::string& bin_id,
                                                       const robot_state::RobotState& start_state,
                                                       const KeyPoseMap& world_state)
    {
        // Find all reachable starting poses.
        std::vector<apc_msgs::PrimitivePlan> starting_poses;
        computeReachableStartingPoses(starting_poses, start_state, world_state);

        // Verify the bins reachable from the starting poses.
        std::vector<apc_msgs::PrimitivePlan> bin_poses;
        retrieveBinPoses(bin_poses, bin_id);
        valid_bins.clear();
        for (int i = 0; i < bin_poses.size(); i++) {
            bool valid = false;

            // FIXME Put OMP over outer loop
#pragma omp parallel num_threads(1) shared(valid)
            {
#pragma omp for
                for (int j = 0; j < starting_poses.size(); j++) {
                    if (valid) {
                        continue;
                    }

                    // Get the next bin pose.
                    apc_msgs::PrimitivePlan bin_pose = bin_poses[i];

                    robot_state::RobotState robot_state = start_state;
                    // Set robot state to the starting pose. We do this by setting
                    // the robot state to the end state of each individual action.
                    for (int k = 0; k < starting_poses[j].actions.size(); k++) {
                        const apc_msgs::PrimitiveAction& action = starting_poses[j].actions[k];
                        int num_pts = action.joint_trajectory.points.size();
                        setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
                        // setStateFromAction(robot_state, world_state, action, num_pts - 1);
                    }
                    // Compute the trajectory from starting pose to bin. If it
                    // works, add the plan from starting pose to bin pose to the
                    // valid bin poses.
                    try {
                        ROS_DEBUG("Testing bin pose: %s", bin_poses[i].plan_name.c_str());
                        // On failure an exception is thrown.
                        computePlan(bin_pose, robot_state, world_state, j % 8);

                        // This line will only be reached if the bin pose was valid.

                        // Build the full starting to bin pose trajectory, then
                        // break because only one valid trajectory to that bin pose
                        // is needed.
#pragma omp critical
                        {
                            if (!valid) {
                                valid = true;
                                valid_bins.push_back(apc_msgs::PrimitivePlan());
                                valid_bins.back() = starting_poses[j];
                                valid_bins.back().actions.insert(valid_bins.back().actions.end(),
                                                                 bin_pose.actions.begin(),
                                                                 bin_pose.actions.end());
                            }
                        }

                    } catch (std::exception& error) {
                        ROS_DEBUG("Starting pose failed with %s", error.what());
                    }
                } // omp for
            } // omp parallel

        }

        APC_ASSERT(valid_bins.size() > 0,
                   "No reachable poses for bin %s!", bin_id.c_str());
    }

    void MotionPlanningFrame::setStateToPlanJointTrajectoryEnd(robot_state::RobotState& robot_state,
                                                               const apc_msgs::PrimitivePlan& plan)
    {
        // Set robot state to the starting pose. We do this by setting
        // the robot state to the end state of each individual action.
        for (int k = 0; k < plan.actions.size(); k++) {
            const apc_msgs::PrimitiveAction& action = plan.actions[k];
            setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
        }
    }

    void MotionPlanningFrame::retrieveItemGrasps(std::vector<apc_msgs::PrimitivePlan>& item_grasps,
                                                 const std::string& item_id)
    {
        item_grasps = findMatchingPlansAny("", "grasp.*", ".*", ".*", item_id, ".*", true);
    }

    void MotionPlanningFrame::computePickAndPlaceForItem(apc_msgs::PrimitivePlan& item_plan,
                                                         const std::string& bin_id,
                                                         const std::string& item_id,
                                                         const robot_state::RobotState& start_state,
                                                         const KeyPoseMap& world_state)
    {
        std::exception except;

        // Get the items keys matching item ids in the bin.
        std::vector<std::string> target_keys = findItemKeysInBinMatchingItemID(bin_id, item_id, world_state);

        // Get the reachable bin poses to the target bin.
        std::vector<apc_msgs::PrimitivePlan> bin_poses;
        computeReachableBinPoses(bin_poses, bin_id, start_state, world_state);

        std::vector<apc_msgs::PrimitivePlan> item_grasps;
        retrieveItemGrasps(item_grasps, item_id);

        std::vector<apc_msgs::PrimitivePlan> valid_grasps;
#pragma omp parallel num_threads(8)
        {
#pragma omp for
            for (int i = 0; i < item_grasps.size(); i++) {
                if (valid_grasps.size() > 0) {
                    continue;
                }

                bool valid = false;

                for (int j = 0; j < bin_poses.size(); j++) {
                    if (valid)
                        continue;

                    apc_msgs::PrimitivePlan item_grasp = item_grasps[i];

                    robot_state::RobotState robot_state = start_state;
                    setStateToPlanJointTrajectoryEnd(robot_state, bin_poses[j]);
                    // Compute the trajectory from starting pose to bin. If it
                    // works, add the plan from starting pose to bin pose to the
                    // valid bin poses.
                    try {
                        ROS_DEBUG("Testing grasp: %s", item_grasp.plan_name.c_str());
                        // On failure an exception is thrown.
                        computePlan(item_grasp, robot_state, world_state, i % 8);

                        // This line will only be reached if the bin pose was valid.
                        valid = true;

                        // Append valid grasp to vector.
#pragma omp critical
                        {
                            valid_grasps.push_back(apc_msgs::PrimitivePlan());
                            valid_grasps.back() = bin_poses[j];
                            valid_grasps.back().actions.insert(valid_grasps.back().actions.end(),
                                                               item_grasp.actions.begin(),
                                                               item_grasp.actions.end());
                        }

                    } catch (std::exception& error) {
                        ROS_DEBUG("Starting pose failed with %s", error.what());
                    }
                }
            }
        }

        if (valid_grasps.size() > 0)
            item_plan = valid_grasps[0];

        APC_ASSERT(item_plan.actions.size() > 0,
                   "No reachable poses for bin %s!", bin_id.c_str());
    }

    KeyPoseMap MotionPlanningFrame::computeExpectedWorldState(const apc_msgs::PrimitivePlan& plan,
                                                              const robot_state::RobotState& robot_state,
                                                              const KeyPoseMap& world_state)
    {
        apc_msgs::PrimitivePlan cache = plan;
        return computeNearestFrameAndObjectKeysPartial(robot_state, world_state, cache);
    }

    robot_state::RobotState MotionPlanningFrame::computeExpectedRobotState(const apc_msgs::PrimitivePlan& plan,
                                                                           const robot_state::RobotState& robot_state,
                                                                           const KeyPoseMap& world_state)
    {
        robot_state::RobotState robot = robot_state;
        for (int i = 0; i < plan.actions.size(); i++) {
            const apc_msgs::PrimitiveAction& action = plan.actions[i];
            const std::vector<std::string> joint_names = action.joint_trajectory.joint_names;
            const trajectory_msgs::JointTrajectoryPoint& point = action.joint_trajectory.points.back();
            setStateFromPoint(robot, joint_names, point);
        }
        return robot;
    }

    void MotionPlanningFrame::computeRunAPC(apc_msgs::PrimitivePlan& work_plan,
                                            const WorkOrder& work_order,
                                            const robot_state::RobotState& start,
                                            const KeyPoseMap& world,
                                            bool use_vision,
                                            bool execute)
    {
        robot_state::RobotState robot_state = start;
        KeyPoseMap world_state = world;
        std::string bin_id;
        std::string item_id;
        apc_msgs::PrimitivePlan item_plan;
        for (int i = 0; i < work_order.size(); i++) {
            // Get the target bin and target item.
            bin_id = work_order[i].first;
            item_id = work_order[i].second;

            if (use_vision) {
                // TODO Call vision code.
                // world_state = computeRunVision(bin_id, item_id, world_state);
            }

            if (execute) {
                // TODO Get the current state of the robot.
                // robot_state = getCurrentState();
            }

            // Compute a pick and place plan for the item.
            computePickAndPlaceForItem(item_plan, bin_id, item_id, robot_state, world_state);

            // Compute the expected states after executing the plan.
            world_state = computeExpectedWorldState(item_plan, robot_state, world_state);
            robot_state = computeExpectedRobotState(item_plan, robot_state, world_state);

            // Store pick and place into record plan.
            work_plan.actions.insert(work_plan.actions.end(),
                                     item_plan.actions.begin(),
                                     item_plan.actions.end());

            if (execute) {
                // TODO Call execution code.
            }
        }
    }

    WorkOrder MotionPlanningFrame::computeWorkOrder(bool single_step) {
        // Build work order.
        WorkOrder work_order;
        QTableWidget* json_table = ui_->json_table_widget;
        int index = json_table->currentRow();
        if (index < 0)
            index = 0;
        for (int i = index; i < json_table->rowCount(); i++) {
            work_order.push_back(WorkOrderItem(json_table->item(i, 0)->text().toStdString(),
                                               json_table->item(i, 1)->text().toStdString()));
            if (single_step)
                return work_order;
        }
        return work_order;
    }

    void MotionPlanningFrame::computeRunAPCButtonClicked()
    {
        // Get flags.
        bool single_step = ui_->single_step_checkbox->isChecked();
        bool check_plan = ui_->check_plan_checkbox->isChecked();
        bool use_vision = ui_->use_vision_checkbox->isChecked();
        bool execute = ui_->execute_checkbox->isChecked();

        // Build work order.
        WorkOrder work_order = computeWorkOrder(single_step);

        // Run APC.
        apc_msgs::PrimitivePlan work_plan;
        // Get the current states.
        robot_state::RobotState robot_state = planning_display_->getPlanningSceneRO()->getCurrentState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        try {
            // Run APC.
            computeRunAPC(work_plan, work_order, robot_state, world_state, use_vision, execute);

        } catch (std::exception& error) {
            ROS_ERROR("Caught exception in %s", error.what());
        }

        // If check plan, overwrite plan to active actions list.
        if (check_plan) {
            ROS_DEBUG_STREAM("work_plan:\n" << work_plan);

            planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, work_plan));
            {
                computeSmoothedPath(work_plan);
                moveit_msgs::RobotState start_state_msg;
                robot_state::robotStateToRobotStateMsg(robot_state, start_state_msg);
                loadPlanToPreview(start_state_msg, work_plan);
            }
        }
    }

}
