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
#include <apc_msgs/ComputeDenseMotion.h>

#include <apc/planning.h>
#include <apc/eigen.h>


namespace moveit_rviz_plugin
{

    bool MotionPlanningFrame::testForItemKey(const std::string& key)
    {
        return (key.find("bin") != 0 &&
                key.find("kiva_pod") != 0 &&
                key.find("order_bin") != 0 &&
                key.find("work_table") != 0);
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

        ROS_DEBUG("starting_poses.size() = %ld", starting_poses.size());

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
                    computePlan(starting_poses[i], robot_state, world_state,
                                omp_get_thread_num() % _compute_dense_motion_clients.size());
                    // This line will only be reached if the plan was valid.
#pragma omp critical
                    {
                        valid_starts.push_back(starting_poses[i]);
                    }
                } catch (apc_exception::Exception& error) {
                    ROS_DEBUG("Starting pose failed with %s", error.what());
                }
            }
        }

        APC_ASSERT_PLAN_VECTOR(valid_starts.size() > 0, starting_poses,
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

        // For debugging, store all starting x bin poses in this vector.
        std::vector<apc_msgs::PrimitivePlan> starting_x_bin_poses;
        starting_x_bin_poses.resize(starting_poses.size() * bin_poses.size());

        // Search!
        valid_bins.clear();
        for (int i = 0; i < bin_poses.size(); i++) {
            bool valid = false;

#pragma omp parallel num_threads(8) shared(valid)
            {
#pragma omp for
                for (int j = 0; j < starting_poses.size(); j++) {
                    // If we've already found a valid bin pose, skip over the
                    // remaining starting poses.
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
                        // setStateFromAction(robot_state, world_state, action, -1);
                    }

                    // Compute the trajectory from starting pose to bin. If it
                    // works, add the plan from starting pose to bin pose to the
                    // valid bin poses.
                    try {
                        ROS_DEBUG("Testing bin pose: %s", bin_poses[i].plan_name.c_str());

                        // On failure an exception is thrown.
                        computePlan(bin_pose, robot_state, world_state, omp_get_thread_num() % _compute_dense_motion_clients.size());

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

                    } catch (apc_exception::Exception& error) {
                        ROS_DEBUG("Starting pose failed with %s", error.what());

                        // Only store to starting_x_bin_poses on an error. We
                        // only read from 'starting_x_bin_poses' on a failure to
                        // completely find a reachable bin pose anyhow.
                        apc_msgs::PrimitivePlan& sxb = starting_x_bin_poses[j * bin_poses.size() + i];
                        sxb = starting_poses[j];
                        sxb.actions.insert(sxb.actions.end(),
                                           bin_pose.actions.begin(),
                                           bin_pose.actions.end());

                    }
                } // omp for
            } // omp parallel

        }

        APC_ASSERT_PLAN_VECTOR(valid_bins.size() > 0, starting_x_bin_poses,
                               "No reachable poses for bin %s!", bin_id.c_str());

        apc_msgs::PrimitivePlan ogs;
        for (int i = 0; i < valid_bins.size(); i++) {
            ogs.actions.insert(ogs.actions.end(), valid_bins[i].actions.begin(),
                               valid_bins[i].actions.end());
        }
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, ogs));
        // display trajectory
        {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(start_state, start_state_msg);
            loadPlanToPreview(start_state_msg, ogs);
            // loadPlanToActiveActions(plan);
        }
    }

    void MotionPlanningFrame::retrievePregraspPoses(std::vector<apc_msgs::PrimitivePlan>& pregrasp_poses)
    {
        pregrasp_poses = findMatchingPlansAny("", "pregrasp.*", ".*arm_torso", "bin");
    }

    void MotionPlanningFrame::computeReachablePregraspPoses(std::vector<apc_msgs::PrimitivePlan>& valid_pregrasps,
                                                            const std::string& bin_id,
                                                            const robot_state::RobotState& start_state,
                                                            const KeyPoseMap& world_state)
    {
        // Verify the bins reachable from the starting poses.
        std::vector<apc_msgs::PrimitivePlan> bin_poses;
        computeReachableBinPoses(bin_poses, bin_id, start_state, world_state);

        std::vector<apc_msgs::PrimitivePlan> pregrasp_poses;
        retrievePregraspPoses(pregrasp_poses);

        // For debugging, store all starting x bin poses in this vector.
        std::vector<apc_msgs::PrimitivePlan> bin_x_pregrasp_poses;
        bin_x_pregrasp_poses.resize(bin_poses.size() * pregrasp_poses.size());

        // Search!
        valid_pregrasps.clear();
        bool valid = false;
#pragma omp parallel num_threads(8) shared(valid)
        {
#pragma omp for
            for (int i = 0; i < pregrasp_poses.size(); i++) {

                for (int j = 0; j < bin_poses.size(); j++) {
                    // If we've already found a valid bin pose, skip over the
                    // remaining starting poses.
                    // if (valid) {
                    //     continue;
                    // }

                    // Get the next bin pose.
                    apc_msgs::PrimitivePlan pregrasp_pose = pregrasp_poses[i];

                    robot_state::RobotState robot_state = start_state;
                    // Set robot state to the starting pose. We do this by setting
                    // the robot state to the end state of each individual action.
                    for (int k = 0; k < bin_poses[j].actions.size(); k++) {
                        const apc_msgs::PrimitiveAction& action = bin_poses[j].actions[k];
                        int num_pts = action.joint_trajectory.points.size();
                        setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
                        // setStateFromAction(robot_state, world_state, action, -1);
                    }

                    // Compute the trajectory from starting pose to bin. If it
                    // works, add the plan from starting pose to bin pose to the
                    // valid bin poses.
                    try {
                        ROS_DEBUG("Testing pregrasp pose: %s", pregrasp_poses[i].plan_name.c_str());

                        // On failure an exception is thrown.
                        computePlan(pregrasp_pose, robot_state, world_state,
                                    omp_get_thread_num() % _compute_dense_motion_clients.size());

                        // This line will only be reached if the bin pose was valid.

                        // Build the full starting to bin pose trajectory, then
                        // break because only one valid trajectory to that bin pose
                        // is needed.
#pragma omp critical
                        {
                            // if (!valid) {
                            valid_pregrasps.push_back(apc_msgs::PrimitivePlan());
                            valid_pregrasps.back() = bin_poses[j];
                            valid_pregrasps.back().actions.insert(valid_pregrasps.back().actions.end(),
                                                                  pregrasp_pose.actions.begin(),
                                                                  pregrasp_pose.actions.end());
                            // }
                        }

                    } catch (apc_exception::Exception& error) {
                        ROS_DEBUG("Bin pose failed with %s", error.what());

                        // Only store to bin_x_pregrasp_poses on an error. We
                        // only read from 'bin_x_pregrasp_poses' on a failure to
                        // completely find a reachable bin pose anyhow.
                        apc_msgs::PrimitivePlan& sxb = bin_x_pregrasp_poses[j * pregrasp_poses.size() + i];
                        sxb = bin_poses[j];
                        sxb.actions.insert(sxb.actions.end(),
                                           pregrasp_pose.actions.begin(),
                                           pregrasp_pose.actions.end());

                    }
                }
            } // omp for
        }// omp parallel

        APC_ASSERT_PLAN_VECTOR(valid_pregrasps.size() > 0, bin_x_pregrasp_poses,
                               "No reachable pregrasp poses for bin %s!", bin_id.c_str());
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
        // Get the items keys matching item ids in the bin.
        std::vector<std::string> target_keys = findItemKeysInBinMatchingItemID(bin_id, item_id, world_state);

        // Get the reachable bin poses to the target bin.
        std::vector<apc_msgs::PrimitivePlan> bin_poses;
        computeReachableBinPoses(bin_poses, bin_id, start_state, world_state);

        APC_ASSERT(bin_poses.size() > 0,
                   "NO BIN POSES?");

        APC_ASSERT(bin_poses[0].actions.size() > 0, "WHAT");

        ROS_INFO("FOO");

        apc_planning::assertPlanningPreconditions(bin_poses, start_state, world_state);

        // Get a pick.
        std::vector<apc_msgs::PrimitivePlan> picks;
        try {
            computePick(picks, item_id, bin_id, bin_poses[0], start_state, world_state);
        } catch (apc_exception::Exception& e) {
            ROS_ERROR("%s", e.what());
            throw e;
        }

        ROS_INFO("FOO");

        {
            robot_state::RobotState test_robot = start_state;
            setStateToPlanJointTrajectoryEnd(test_robot, bin_poses[0]);
            apc_planning::assertPlanningPreconditions(picks, test_robot, world_state);
        }

        // Append reachable bin poses to the picks.
        for (int i = 0; i < picks.size(); i++) {
            picks[i].actions.insert(picks[i].actions.begin(),
                                    bin_poses[0].actions.begin(),
                                    bin_poses[0].actions.end());
        }

        apc_planning::assertPlanningPreconditions(picks, start_state, world_state);

        // Get a scoring plan.
        std::vector<apc_msgs::PrimitivePlan> scoring_plans;
        computeReachableScoreWithItemPoses(scoring_plans, picks, start_state, world_state);

        apc_planning::assertPlanningPreconditions(scoring_plans, start_state, world_state);

        if (scoring_plans.size() > 0)
            item_plan = scoring_plans[0];

    }


    void MotionPlanningFrame::retrievePostgraspPoses(std::vector<apc_msgs::PrimitivePlan>& score_with_item_poses)
    {
        score_with_item_poses = findMatchingPlansAny("", "postgrasp.*", ".*_arm", "bin");
    }

    void MotionPlanningFrame::computeReachablePostgraspPoses(std::vector<apc_msgs::PrimitivePlan>& valid_postgrasps,
                                                             const std::vector<apc_msgs::PrimitivePlan>& valid_grasps,
                                                             const robot_state::RobotState& start_state,
                                                             const KeyPoseMap& world_state)
    {
        // Clear valid scores.
        valid_postgrasps.clear();

        // Get the scoring poses.
        std::vector<apc_msgs::PrimitivePlan> postgrasp_poses;
        retrievePostgraspPoses(postgrasp_poses);

        // Debugging.
        std::vector<apc_msgs::PrimitivePlan> grasp_x_postgrasp;
        grasp_x_postgrasp.resize(valid_grasps.size() * postgrasp_poses.size());

        bool valid = false;
#pragma omp parallel num_threads(8) shared(valid)
        {
#pragma omp for
            for (int i = 0; i < valid_grasps.size(); i++) {
                if (valid_postgrasps.size() > 0) {
                    continue;
                }

                for (int j = 0; j < postgrasp_poses.size(); j++) {
                    if (valid)
                        continue;

                    apc_msgs::PrimitivePlan postgrasp_pose = postgrasp_poses[j];

                    robot_state::RobotState robot_state = start_state;
                    setStateToPlanJointTrajectoryEnd(robot_state, valid_grasps[i]);

                    // Append to postgrasp_pose with grasp. This ensures that
                    // the postgrasp pose is computed as a postgrasp.
                    postgrasp_pose.actions.insert(postgrasp_pose.actions.begin(), valid_grasps[i].actions.back());
                    for (int k = 0; k < postgrasp_pose.actions.size(); k++) {
                        // IMPORTANT!@!!!!!
                        postgrasp_pose.actions[k].frame_key = "";
                        postgrasp_pose.actions[k].object_key = "";

                        postgrasp_pose.actions[k].object_id = postgrasp_pose.actions[0].object_id;
                        postgrasp_pose.actions[k].attached_link_id = postgrasp_pose.actions[0].attached_link_id;
                        postgrasp_pose.actions[k].object_trajectory = postgrasp_pose.actions[0].object_trajectory;
                        postgrasp_pose.actions[k].grasp = postgrasp_pose.actions[0].grasp;
                    }

                    // Compute the trajectory from starting pose to bin. If it
                    // works, add the plan from starting pose to bin pose to the
                    // valid bin poses.
                    try {
                        ROS_DEBUG("Testing postgrasp pose: %s", postgrasp_pose.plan_name.c_str());
                        // On failure an exception is thrown.
                        computePlan(postgrasp_pose, robot_state, world_state,
                                    omp_get_thread_num() % _compute_dense_motion_clients.size());

                        // This line will only be reached if the bin pose was valid.

                        // Append valid grasp to vector.
#pragma omp critical
                        {
                            valid = true;
                            valid_postgrasps.push_back(apc_msgs::PrimitivePlan());
                            valid_postgrasps.back() = valid_grasps[i];
                            valid_postgrasps.back().actions.insert(valid_postgrasps.back().actions.end(),
                                                               postgrasp_pose.actions.begin(),
                                                               postgrasp_pose.actions.end());
                        }

                    } catch (apc_exception::Exception& error) {
                        ROS_DEBUG("Postgrasp pose failed with %s", error.what());

                        apc_msgs::PrimitivePlan& grasp_score = grasp_x_postgrasp[j * valid_grasps.size() + i];
                        grasp_score = valid_grasps[i];
                        grasp_score.actions.insert(grasp_score.actions.end(),
                                                   postgrasp_pose.actions.begin(),
                                                   postgrasp_pose.actions.end());
                    }
                }
            }
        }

        APC_ASSERT_PLAN_VECTOR(valid_postgrasps.size() > 0, grasp_x_postgrasp,
                               "No reachable postgrasp poses!");
    }

    void MotionPlanningFrame::retrieveScoreWithItemPoses(std::vector<apc_msgs::PrimitivePlan>& score_with_item_poses)
    {
        score_with_item_poses = findMatchingPlansAny("", ".*", ".*", "order_bin", ".*", ".*", false);
    }

    // void MotionPlanningFrame::computeSetItemButtonClicked()
    // {
    //     // Get the currently highlighted action.

    // }

    void MotionPlanningFrame::setItemPoseFromAction(KeyPoseMap& world_state,
                                                    const apc_msgs::PrimitiveAction& action,
                                                    const robot_state::RobotState&   state)
    {
        if (!(apc_planning::is_action_grasp(action) || apc_planning::is_action_postgrasp(action))) {
            ROS_WARN("Trying to set item pose from a non-grasp");
            return;
        }

        APC_ASSERT(!action.object_id.empty(),
                   "Cannot set item pose without an item ID\n%s",
                   apc_planning::toStringNoArr(action).c_str());
        APC_ASSERT(!action.object_key.empty(),
                   "Cannot set item pose without an item key\n%s",
                   apc_planning::toStringNoArr(action).c_str());
        APC_ASSERT(action.object_trajectory.poses.size() > 0,
                   "Cannot set item pose without poses\n%s",
                   apc_planning::toStringNoArr(action).c_str());
        APC_ASSERT(!action.attached_link_id.empty(),
                   "Cannot set item pose without a link to attach to\n%s",
                   apc_planning::toStringNoArr(action).c_str());
        // Set robot to pose.
        robot_state::RobotState robot_state = state;
        setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
        Eigen::Affine3d T_link_world = robot_state.getGlobalLinkTransform(action.attached_link_id);
        Eigen::Affine3d T_object_link;
        tf::poseMsgToEigen(action.object_trajectory.poses.back(), T_object_link);
        world_state[action.object_key] = T_link_world * T_object_link;
    }

    void MotionPlanningFrame::computeReachableScoreWithItemPoses(std::vector<apc_msgs::PrimitivePlan>& valid_scores,
                                                                 const std::vector<apc_msgs::PrimitivePlan>& valid_grasps,
                                                                 const robot_state::RobotState& start,
                                                                 const KeyPoseMap& world)
    {
        // Clear valid scores.
        valid_scores.clear();

        // Get the scoring poses.
        std::vector<apc_msgs::PrimitivePlan> score_poses;
        retrieveScoreWithItemPoses(score_poses);

        // Debugging.
        std::vector<apc_msgs::PrimitivePlan> grasp_x_score;
        grasp_x_score.resize(valid_grasps.size() * score_poses.size());

        //
        // HACK Attach object to robot state so that the planner
        // will know we start in a grasp.
        robot_state::RobotState start_state = start;
        {
            const apc_msgs::PrimitiveAction& action = valid_grasps[0].actions.back();
            setAttachedObjectFromAction(start_state, world, action, 0);
        }

#pragma omp parallel num_threads(8)
        {
#pragma omp for
            for (int i = 0; i < valid_grasps.size(); i++) {
                if (valid_scores.size() > 0) {
                    continue;
                }

                bool valid = false;

                for (int j = 0; j < score_poses.size(); j++) {
                    if (valid)
                        continue;

                    apc_msgs::PrimitivePlan score_pose = score_poses[j];

                    robot_state::RobotState robot_state = start_state;
                    setStateToPlanJointTrajectoryEnd(robot_state, valid_grasps[i]);

                    // Add grasped items to the scoring pose.
                    {
                        typedef apc_msgs::PrimitivePlan Plan;
                        typedef apc_msgs::PrimitiveAction Action;
                        // Find the correct attach object pose.
                        Plan gc = valid_grasps[i]; // copy of grasp plan.
                        Action query_grasp;
                        for (int i = gc.actions.size() - 1; i >=0; i--) {
                            if (apc_planning::is_action_grasp(gc.actions[i]) ||
                                apc_planning::is_action_postgrasp(gc.actions[i])) {
                                query_grasp = gc.actions[i];
                            }
                        }
                        // Attach object for the entire duration of the trajectory.
                        for (std::vector<Action>::iterator action = score_pose.actions.begin();
                             action != score_pose.actions.end(); ++action) {
                            action->object_id = query_grasp.object_id;
                            action->object_key = query_grasp.object_key;
                            action->object_trajectory = query_grasp.object_trajectory;
                            action->attached_link_id = query_grasp.attached_link_id;
                            action->grasp = query_grasp.grasp;
                        }
                    }

                    // Compute the trajectory from starting pose to bin. If it
                    // works, add the plan from starting pose to bin pose to the
                    // valid bin poses.
                    try {
                        ROS_DEBUG("Testing scoring pose: %s", score_pose.plan_name.c_str());
                        // On failure an exception is thrown.
                        computePlan(score_pose, robot_state,
                                    world, omp_get_thread_num() % _compute_dense_motion_clients.size());

                        // This line will only be reached if the bin pose was valid.
                        valid = true;

                        // Append valid grasp to vector.
#pragma omp critical
                        {
                            valid_scores.push_back(apc_msgs::PrimitivePlan());
                            valid_scores.back() = valid_grasps[i];
                            valid_scores.back().actions.insert(valid_scores.back().actions.end(),
                                                               score_pose.actions.begin(),
                                                               score_pose.actions.end());
                        }

                    } catch (apc_exception::Exception& error) {
                        ROS_DEBUG("Scoring pose failed with %s", error.what());

                        apc_msgs::PrimitivePlan& grasp_score = grasp_x_score[j * valid_grasps.size() + i];
                        grasp_score = valid_grasps[i];
                        grasp_score.actions.insert(grasp_score.actions.end(),
                                                   score_pose.actions.begin(),
                                                   score_pose.actions.end());
                    }
                }
            }
        }

        APC_ASSERT_PLAN_VECTOR(valid_scores.size() > 0, grasp_x_score,
                               "No reachable scoring poses!");
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

        if (use_vision) {
            try {
                computeRunVision(world_state);
            } catch (apc_exception::Exception& error) {
                ROS_ERROR("Caught exception in\n%s", error.what());
            }
        }

        for (int i = 0; i < work_order.size(); i++) {
            // Get the target bin and target item.
            bin_id = work_order[i].first;
            item_id = work_order[i].second;

            if (execute) {
                // Get the current state of the robot.
                const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
                robot_state = ps->getCurrentState();
            }

            // Compute a pick and place plan for the item.
            try {

                computePickAndPlaceForItem(item_plan, bin_id, item_id, robot_state, world_state);



                if (execute) {
                    // Call execution code.
                    computeExecute(item_plan);
                }

            } catch (apc_exception::Exception& error) {
                ROS_ERROR("Caught exception\n%s", error.what());

                ROS_INFO_STREAM(apc_planning::toStringNoArr(error.what_plan));
            }

            // Compute the expected states after executing the plan. FIXME Remove.
            // world_state = computeExpectedWorldState(item_plan, robot_state, world_state);
            // robot_state = computeExpectedRobotState(item_plan, robot_state, world_state);

            // Store pick and place into record plan.
            work_plan.actions.insert(work_plan.actions.end(),
                                     item_plan.actions.begin(),
                                     item_plan.actions.end());
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

        // Move to next item in work order.
        if (single_step) {
        }

        // Run APC.
        apc_msgs::PrimitivePlan work_plan;
        // Get the current states.
        robot_state::RobotState robot_state = planning_display_->getPlanningSceneRO()->getCurrentState();
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        try {
            // Run APC.
            computeRunAPC(work_plan, work_order, robot_state, world_state, use_vision, execute);

            if (!primitive_plan_) {
                primitive_plan_.reset(new apc_msgs::PrimitivePlan);
            }
            *primitive_plan_ = work_plan;

        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Caught exception in %s", error.what());

            ROS_INFO_STREAM(apc_planning::toStringNoArr(error.what_plan));

            work_plan = error.what_plan;
        }

        // If check plan, overwrite plan to active actions list.
        if (check_plan) {
            // ROS_DEBUG_STREAM("work_plan:\n" << work_plan);

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
