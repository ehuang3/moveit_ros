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
#include <apc_msgs/ComputeDenseMotion.h>
#include <boost/xpressive/xpressive.hpp>
#include <apc_path/Path.h>
#include <apc_path/Trajectory.h>


#include <apc/eigen.h>
#include <apc/planning.h>


#include <apc_msgs/CheckCollisions.h>
#include <apc_msgs/ComputePreGrasps.h>

#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/variate_generator.hpp>
#include <ctime>


using namespace apc_planning;


namespace moveit_rviz_plugin
{
    KeyPoseMap MotionPlanningFrame::computeWorldKeyPoseMap()
    {
        KeyPoseMap map;

        // Get a read-only copy of the world state.
        planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
        APC_ASSERT(ps, "Failed to get locked planning scene");
        const collision_detection::WorldConstPtr world = ps->getWorld();
        APC_ASSERT(world, "Failed to get collision world");

        // Store all object poses into the key/pose map.
        const std::vector<std::string>& object_keys = world->getObjectIds();
        for (int i = 0; i < object_keys.size(); i++)
            map[object_keys[i]] = world->getObject(object_keys[i])->shape_poses_[0];

        // Construct the bin poses of the KIVA pod.
        if (_kiva_pod) {
            // APC_ASSERT(_kiva_pod, "Failed to get KIVA pod model");
            for (char c = 'A'; c <= 'L'; c++) {
                std::string bin = std::string("bin_") + c;
                map[bin] = map["kiva_pod"] * _kiva_pod->getGlobalTransform(bin);
            }
        }

        return map;
    }

    std::string MotionPlanningFrame::computeNearestFrameKey(const std::string& frame_id,
                                                            const std::string& link_id,
                                                            const robot_state::RobotState& robot,
                                                            const KeyPoseMap& world)
    {
        std::string nearest_frame = "";
        double min_dist = 1e9;
        Eigen::Affine3d T_link_world = robot.getGlobalLinkTransform(link_id);
        for (KeyPoseMap::const_iterator iter = world.begin(); iter != world.end(); ++iter) {
            // This should allow us to handle the ambiguous cases of
            // "bin" or duplicate "objects" because the more specific
            // keys are constructed by appending to "bin" or "object".
            if (iter->first.find(frame_id) == 0) {
                double dist = (iter->second.translation() - T_link_world.translation()).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_frame = iter->first;
                }
            }
        }
        APC_ASSERT(nearest_frame.size() > 0,
                   "Failed to find nearest %s in world", frame_id.c_str());
        return nearest_frame;
    }

    bool MotionPlanningFrame::setStateFromPoint(robot_state::RobotState& robot,
                                                const std::vector<std::string>& joint_names,
                                                const trajectory_msgs::JointTrajectoryPoint& point)
    {
        APC_ASSERT(joint_names.size() == point.positions.size(),
                   "Mismatch between joint names and positions");
        for (int i = 0; i < joint_names.size(); i++)
            robot.setJointPositions(joint_names[i], &point.positions[i]);
        robot.update();
        return true;
    }

    bool MotionPlanningFrame::setStateFromIK(robot_state::RobotState& robot,
                                             const std::string& link_id,
                                             const std::string& group_id,
                                             const std::string& frame_id,
                                             const Eigen::Affine3d& T_frame_world,
                                             const geometry_msgs::Pose& pose_link_frame,
                                             bool use_symmetries)
    {
        APC_ASSERT(link_id.size() > 0, "Failed to provide input link");
        APC_ASSERT(group_id.size() > 0, "Failed to provide input group");
        APC_ASSERT(frame_id.size() > 0, "Failed to provide input frame");

        EigenSTL::vector_Affine3d frame_symmetries;
        if (use_symmetries) {
            getItemSymmetriesCached(frame_id, frame_symmetries);
        } else {
            frame_symmetries.push_back(Eigen::Affine3d::Identity());
        }

        // If the group does not have an IK solver, return false.
        bool solver = robot.getJointModelGroup(group_id)->getSolverInstance();
        APC_ASSERT(solver,
                   "Failed to find IK solver for group %s", group_id.c_str());

        Eigen::Affine3d T_symmetry_world_min = Eigen::Affine3d::Identity();
        double min_x = 1e9;
        double max_z = 0;
        for (int i = 0; i < frame_symmetries.size(); i++) {
            // Copy the robot state for IK.
            robot_state::RobotState ik_robot = robot;

            // Back out the desired link transform in global coordinates.
            Eigen::Affine3d T_symmetry_frame = frame_symmetries[i];
            // Eigen::Affine3d T_frame_link = T_symmetry_link * T_symmetry_frame.inverse();
            // Eigen::Affine3d T_link_world = T_frame_world * T_frame_link.inverse();
            Eigen::Affine3d T_symmetry_world = T_frame_world * T_symmetry_frame;

            // Back out the desired link transform in global coordinates.
            Eigen::Affine3d T_frame_link;
            tf::poseMsgToEigen(pose_link_frame, T_frame_link);
            Eigen::Affine3d T_link_world = T_symmetry_world * T_frame_link.inverse();

            // Snap to the frame using IK.
            const moveit::core::JointModelGroup* jmg = robot.getJointModelGroup(group_id);
            geometry_msgs::Pose pose_link_world;
            tf::poseEigenToMsg(T_link_world, pose_link_world);
            ik_robot.setFromIK(jmg, pose_link_world, link_id);
            // APC_ASSERT(robot.setFromIK(jmg, pose_link_world, link_id),
            //            "Failed to set %s to pose using %s IK", link_id.c_str(), group_id.c_str());
            ik_robot.update();

            Eigen::Affine3d T_ik = ik_robot.getGlobalLinkTransform(link_id);

            if (apc_eigen::elementWiseMatrixNorm(T_ik, T_link_world) < 1e-2) {

                double delta_x = std::abs(T_link_world.translation().x()) - min_x;
                double delta_z = std::abs(T_link_world.translation().z()) - max_z;

                if (std::abs(delta_x) < 3e-2) {
                    if (delta_z > 0) {
                        min_x = std::abs(T_link_world.translation().x());
                        max_z = std::abs(T_link_world.translation().z());
                        T_symmetry_world_min = T_symmetry_world;
                    }
                }
                else if(delta_x < 0) {
                    min_x = std::abs(T_link_world.translation().x());
                    max_z = std::abs(T_link_world.translation().z());
                    T_symmetry_world_min = T_symmetry_world;
                }
            }

            // Manually assert whether the new state places the end-effector at the
            // desired IK position.
            if (!use_symmetries) {
                robot = ik_robot;
                APC_ASSERT(apc_eigen::elementWiseMatrixNorm(T_ik, T_link_world) < 1e-2,
                           "Failed to IK group %s to pose; error is %.6f", group_id.c_str(),
                           apc_eigen::elementWiseMatrixNorm(T_ik, T_link_world));
            }
        }

        if (use_symmetries) {
            setStateFromIK(robot, link_id, group_id, frame_id, T_symmetry_world_min, pose_link_frame, false);
        }

        return true;
    }

    bool MotionPlanningFrame::setAttachedObjectFromAction(robot_state::RobotState& robot_state,
                                                          const KeyPoseMap& world_state,
                                                          const apc_msgs::PrimitiveAction& action,
                                                          const int index)
    {
        // 0. Index can be either a valid pos, invalid pos, or -1.
        APC_ASSERT(index >= -1,
                   "Failed to set state from action %s with index %d", action.action_name.c_str(), index);
        APC_ASSERT(!action.object_key.empty(),
                   "Missing object key for object ID %s in action %s", action.object_id.c_str(), action.action_name.c_str());
        APC_ASSERT(world_state.count(action.object_key) > 0,
                   "Missing object key %s in world state", action.object_key.c_str());
        // Clear attached bodies.
        robot_state.clearAttachedBodies();

        // Get the actual transform from world to object and the
        // desired transform from end-effector to object.
        Eigen::Affine3d T_object_world = world_state.find(action.object_key)->second;
        Eigen::Affine3d T_object_eef;
        APC_ASSERT(action.object_trajectory.poses.size() > 0,
                   "Missing a non-empty object trajectory for object key %s in action %s",
                   action.object_key.c_str(), action.action_name.c_str());
        if (index >= 0) {
            APC_ASSERT(action.object_trajectory.poses.size() > index,
                       "Index %d out of bounds for object trajectory in action %s",
                       index, action.action_name.c_str());
            tf::poseMsgToEigen(action.object_trajectory.poses[index], T_object_eef);
        }
        else if (index == -1) {
            tf::poseMsgToEigen(action.object_trajectory.poses.back(), T_object_eef);
        }

        // Get object shapes and shape poses from the world and convert
        // them to the desired end-effector link frame.
        planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
        collision_detection::CollisionWorld::ObjectConstPtr object = ps->getWorld()->getObject(action.object_key);
        APC_ASSERT(object,
                   "Failed to find object key %s in world", action.object_key.c_str());
        std::vector<shapes::ShapeConstPtr> object_shapes = object->shapes_;
        EigenSTL::vector_Affine3d object_poses = object->shape_poses_;
        for (int i = 0; i < object_poses.size(); i++)
            object_poses[i] = T_object_eef * object_poses[0].inverse() * object_poses[i];
        // Attach object to state.
        moveit_msgs::AttachedCollisionObject aco; // For dummy arguments.
        robot_state.attachBody(action.object_id, object_shapes, object_poses, aco.touch_links,
                               action.attached_link_id, aco.detach_posture);
        robot_state.update();
    }

    bool MotionPlanningFrame::setStateFromAction(robot_state::RobotState& robot_state,
                                                 const KeyPoseMap& world_state,
                                                 const apc_msgs::PrimitiveAction& action,
                                                 const int _index,
                                                 bool use_joint_angles_if_no_ik_solver)
    {
        // 0. Index can be either a valid pos, invalid pos, or -1.
        int index = _index;
        APC_ASSERT(index >= -1,
                   "Failed to set state from action with index %d", index);

        // 1. If there is no frame ID, set state based on the joint
        // angles provided in the action.
        if (action.frame_id.empty()) {
            APC_ASSERT(action.joint_trajectory.points.size() > 0,
                       "Failed to set state with an empty joint trajectory in action %s", action.action_name.c_str());
            if (index >= 0) {
                APC_ASSERT(action.joint_trajectory.points.size() > index,
                           "Index %d out of bounds for joint trajectory in action %s",
                           index, action.action_name.c_str());
                setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points[index]);
            }
            else if (index == -1) {
                setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
            }
        }

        // 2. If there is a frame ID, set the state based on IK to the
        // desired pose. The IK may not reach all poses, so we should
        // also set joint angles of the joints not IK-able.
        if (!action.frame_id.empty()) {
            APC_ASSERT(!action.frame_key.empty(),
                       "Missing frame key for frame ID %s in action %s", action.frame_id.c_str(), action.action_name.c_str());
            kinematics::KinematicsBaseConstPtr solver = robot_state.getJointModelGroup(action.group_id)->getSolverInstance();
            // If no solver exists, try searching the subgroups for a solver.
            if (!solver) {
                std::vector<const moveit::core::JointModelGroup*> subgroups;
                robot_state.getJointModelGroup(action.group_id)->getSubgroups(subgroups);
                for (int i = 0; i < subgroups.size(); i++)
                    if (solver = subgroups[i]->getSolverInstance())
                        break;
            }
            APC_ASSERT(use_joint_angles_if_no_ik_solver || solver,
                       "Missing IK solver for group %s in action %s", action.group_id.c_str(), action.action_name.c_str());
            if (solver) {
                APC_ASSERT(action.joint_trajectory.points.size() > 0,
                           "Failed to set state with an empty joint trajectory in action %s", action.action_name.c_str());

                // For each joint not solveable by the IK solver, set that joint's position manually.
                trajectory_msgs::JointTrajectory no_ik;
                no_ik.points.resize(1);
                const std::vector<std::string>& group_joint_names = action.joint_trajectory.joint_names;
                const std::vector<std::string>& solver_joint_names = solver->getJointNames();
                for (int i = 0; i < action.joint_trajectory.joint_names.size(); i++) {
                    if (std::find(solver_joint_names.begin(), solver_joint_names.end(), group_joint_names[i]) ==
                        solver_joint_names.end()) {
                        no_ik.joint_names.push_back(group_joint_names[i]);
                        if (index >= 0) {
                            APC_ASSERT(action.joint_trajectory.points.size() > index,
                                       "Index %d out of bounds for joint trajectory in action %s",
                                       index, action.action_name.c_str());
                            no_ik.points[0].positions.push_back(action.joint_trajectory.points[index].positions[i]);
                        }
                        else if (index == -1) {
                            no_ik.points[0].positions.push_back(action.joint_trajectory.points.back().positions[i]);
                        }
                    }
                }
                if (no_ik.joint_names.size() > 0) {
                    // ROS_DEBUG_STREAM("No IK for\n" << no_ik);
                    setStateFromPoint(robot_state, no_ik.joint_names, no_ik.points[0]);
                }
                // Set the remaining joints using IK.
                APC_ASSERT(!action.eef_link_id.empty(),
                           "Missing IK link for frame ID %s in action %s", action.frame_id.c_str(), action.action_name.c_str());
                APC_ASSERT(world_state.count(action.frame_key) > 0,
                           "Missing frame key %s in world state", action.frame_key.c_str());
                APC_ASSERT(action.eef_trajectory.poses.size() > 0,
                           "Missing a non-empty eef trajectory for frame key %s in action %s",
                           action.frame_key.c_str(), action.action_name.c_str());

                if (index >= 0) {
                    APC_ASSERT(action.eef_trajectory.poses.size() > index,
                               "Index %d out of bounds for end-effector trajectory in action %s",
                               index, action.action_name.c_str());
                    setStateFromIK(robot_state, action.eef_link_id, solver->getGroupName(), action.frame_id,
                                   world_state.find(action.frame_key)->second, action.eef_trajectory.poses[index]);
                }
                else if (index == -1) {
                    setStateFromIK(robot_state, action.eef_link_id, solver->getGroupName(), action.frame_id,
                                   world_state.find(action.frame_key)->second, action.eef_trajectory.poses.back());
                }

            } else if (use_joint_angles_if_no_ik_solver) {
                APC_ASSERT(action.joint_trajectory.points.size() > 0,
                           "Failed to set state with an empty joint trajectory in action %s", action.action_name.c_str());
                if (index >= 0) {
                    APC_ASSERT(action.joint_trajectory.points.size() > index,
                               "Index %d out of bounds for joint trajectory in action %s",
                               index, action.action_name.c_str());
                    setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points[index]);
                }
                else if (index == -1) {
                    setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
                }
            }
        }

        // 3. If there is an object to attach, attach that object. TODO Replace with is_action_<blank>() function call
        if (is_action_grasp(action) || is_action_postgrasp(action) || is_action_nonprehensile(action)) {
        // if (!action.object_id.empty() && !action.object_key.empty()) {
            // 4. Special case: the object is in the object_id and object_key,
            // but there is no object_trajectory. This only happens when the
            // trajectory is an autogenerated pregrasp. The autogenerated
            // actions have name "vvvvv".
            // if (action.object_trajectory.poses.size() == 0 && action.action_name.find("vvvvv") == 0) {
            //     robot_state.clearAttachedBodies();
            // }
            // Else, do 3.
            // else {
            if (!action.object_key.empty()) {
                setAttachedObjectFromAction(robot_state, world_state, action, index);
            } else {
                robot_state.clearAttachedBodies();
            }
            // }
        } else {
            robot_state.clearAttachedBodies();
        }
    }

    std::string MotionPlanningFrame::computeNearestObjectKey(const std::string& object_id,
                                                             const std::string& link_id,
                                                             const robot_state::RobotState& robot,
                                                             const KeyPoseMap& world)
    {
        return computeNearestFrameKey(object_id, link_id, robot, world);
    }

    KeyPoseMap MotionPlanningFrame::computeNearestFrameAndObjectKeys(const robot_state::RobotState& start,
                                                                     const KeyPoseMap& world,
                                                                     apc_msgs::PrimitivePlan& plan)
    {
        // Throw an exception if keys already exist in the plan.
        for (int i = 0; i < plan.actions.size(); i++)
            APC_ASSERT(plan.actions[i].frame_key.empty() && plan.actions[i].object_key.empty(),
                       "Plan frame/object keys already exist\n"
                       "  frame_key: %s\n"
                       " object_key: %s\n",
                       plan.actions[i].frame_key.c_str(), plan.actions[i].object_key.c_str());
        // Compute the nearest keys.
        return computeNearestFrameAndObjectKeysPartial(start, world, plan);
    }

    KeyPoseMap MotionPlanningFrame::computeNearestFrameAndObjectKeysPartial(const robot_state::RobotState& start,
                                                                            const KeyPoseMap& world,
                                                                            apc_msgs::PrimitivePlan& plan)
    {
        // Copy starting conditions.
        robot_state::RobotState robot_state = start;
        KeyPoseMap world_state = world;

        // For each action..
        for (int i = 0; i < plan.actions.size(); i++) {
            apc_msgs::PrimitiveAction& action = plan.actions[i];

            // Compute a frame key if a frame ID is specified.
            if (!action.frame_id.empty() && action.frame_key.empty()) {
                action.frame_key = computeNearestFrameKey(action.frame_id, action.eef_link_id, robot_state, world_state);
            }

            // Set the state using IK if the frame ID is specified, using joint angles otherwise.
            setStateFromAction(robot_state, world_state, action, 0, true);

            // Compute an object key if an object ID is specified.
            if (!action.object_id.empty() && action.object_key.empty()) {
                action.object_key = computeNearestObjectKey(action.object_id, action.attached_link_id, robot_state, world_state);
                // TODO Assert that the object is at the correct location to attach.
            }

            // Move robot state to goal position.
            int num_points = action.joint_trajectory.points.size();
            setStateFromAction(robot_state, world_state, action, -1, true);

            // We want to move the object to the goal position only when
            // the action is grasp, post-grasp, or non-prehensile.
            if (is_action_grasp(action) || is_action_postgrasp(action) || is_action_nonprehensile(action)) {
            // if (!action.object_id.empty() && action.object_trajectory.poses.size() > 0) {
                Eigen::Affine3d T_link_world = robot_state.getGlobalLinkTransform(action.attached_link_id);
                Eigen::Affine3d T_object_link;
                tf::poseMsgToEigen(action.object_trajectory.poses.back(), T_object_link);
                world_state[action.object_key] = T_link_world * T_object_link;
            }
        }

        // Assert that all frames and object keys have been filled out.
        for (int i = 0; i < plan.actions.size(); i++) {
            const apc_msgs::PrimitiveAction& action = plan.actions[i];
            APC_ASSERT(action.frame_id.empty() || !action.frame_key.empty(),
                       "Failed to assign a frame key to frame ID %s for action %s",
                       action.frame_id.c_str(), action.action_name.c_str());
            APC_ASSERT(action.object_id.empty() || !action.object_key.empty(),
                       "Failed to assign a object key to object ID %s for action %s",
                       action.object_id.c_str(), action.action_name.c_str());
        }

        return world_state;
    }

    void MotionPlanningFrame::setStateToActionJointTrajectory(const robot_state::RobotState& state,
                                                              apc_msgs::PrimitiveAction& action,
                                                              int index)
    {
        APC_ASSERT(action.joint_trajectory.joint_names.size() > 0,
                   "Missing joint names in action %s joint trajectory", action.action_name.c_str());
        APC_ASSERT(0 <= index && index < action.joint_trajectory.points.size(),
                   "Index out of bounds for action %s joint trajectory", action.action_name.c_str());
        trajectory_msgs::JointTrajectoryPoint point;
        std::vector<std::string> joint_names = action.joint_trajectory.joint_names;
        for (int i = 0; i < joint_names.size(); i++) {
            int index = state.getJointModel(joint_names[i])->getFirstVariableIndex();
            double angle = state.getVariablePosition(index);
            point.positions.push_back(angle);
        }
        action.joint_trajectory.points[index] = point;
    }

    KeyPoseMap MotionPlanningFrame::computeActionJointTrajectoryPoints(const robot_state::RobotState& start,
                                                                       const KeyPoseMap& world,
                                                                       apc_msgs::PrimitivePlan& plan)
    {
        // Throw an exception if keys are missing from the plan.
        for (int i = 0; i < plan.actions.size(); i++) {
            if (!plan.actions[i].frame_id.empty())
                APC_ASSERT(!plan.actions[i].frame_key.empty(), "Missing frame key for %s in action %s",
                           plan.actions[i].frame_id.c_str(), plan.actions[i].action_name.c_str());
            if (!plan.actions[i].object_id.empty())
                APC_ASSERT(!plan.actions[i].object_key.empty(), "Missing object key for %s in action %s",
                           plan.actions[i].object_id.c_str(), plan.actions[i].action_name.c_str());
        }

        // Copy starting conditions.
        robot_state::RobotState robot_state = start;
        KeyPoseMap world_state = world;

        // For each action..
        for (int i = 0; i < plan.actions.size(); i++) {
            apc_msgs::PrimitiveAction& action = plan.actions[i];

            // Move robot state through end-effector trajectory and
            // save joint angles to joint trajectory.
            if (!action.frame_id.empty()) {
                if (action.eef_trajectory.poses.size() > 2)                // we probably linterp'd
                    continue;

                std::stringstream ss;
                ss << action;
                APC_ASSERT(action.eef_trajectory.poses.size() > 0,
                           "Missing end-effector trajectory for %s", action.action_name.c_str());
                if (action.eef_trajectory.poses.size() != action.joint_trajectory.points.size())
                    continue;
                APC_ASSERT(action.eef_trajectory.poses.size() == action.joint_trajectory.points.size(),
                           "Mismatch between end-effector trajectory points and joint trajectory points for %s",
                           ss.str().c_str());
                // Rebuild joint trajectory.
                for (int j = 0; j < action.eef_trajectory.poses.size(); j++) {
                    // If the start and goal states are the same, do not
                    // recompute IK!
                    if (j > 0 && action.eef_locked) {
                        action.joint_trajectory.points[j] = action.joint_trajectory.points[0];
                        continue;
                    }

                    // Try setting state from IK, reverting to joint angles on failure.
                    setStateFromAction(robot_state, world_state, action, j, true);
                    // Write joint angles to action.
                    setStateToActionJointTrajectory(robot_state, action, j);
                }
            }

            // Move robot state to goal position.
            if (action.frame_id.empty()) {
                setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
            }

            // Move object to goal position.
            if (is_action_grasp(action) || is_action_postgrasp(action) || is_action_nonprehensile(action)) {
            // if (!action.object_id.empty()) {
                Eigen::Affine3d T_link_world = robot_state.getGlobalLinkTransform(action.attached_link_id);
                Eigen::Affine3d T_object_link;
                tf::poseMsgToEigen(action.object_trajectory.poses.back(), T_object_link);
                world_state[action.object_key] = T_link_world * T_object_link;
            }
        }

        return world_state;
    }

    void MotionPlanningFrame::setWorldKeyPoseToWorldStateMessage(const KeyPoseMap& keypose,
                                                                 apc_msgs::WorldState& state)
    {
        state.frames.clear();
        state.objects.clear();
        for (KeyPoseMap::const_iterator iter = keypose.begin(); iter != keypose.end(); ++iter) {
            // Get key.
            std::string key = iter->first;

            // Get ID. Note that this regex is not great, it will fail to
            // produce the right ID when given "stanley_66_052" (instead of
            // "stanley_66_052_0") because it looks for a trailing "_[0-9]+" to
            // distinguish keys from IDs. A temporary fix would be to limit the
            // distinguishing token to "_[0-9]{1,2}".
            using namespace boost::xpressive;
            sregex rex = sregex::compile("^([A-Za-z]+(\\_[A-Za-z0-9]+)*?)([\\_][0-9]{1,2})?");
            smatch what;
            APC_ASSERT(regex_match(iter->first, what, rex),
                       "Failed to extract ID of %s", iter->first.c_str());
            std::string id = what[1];
            // ROS_DEBUG_STREAM("key: " << key << " id: " << id);

            // Construct frame state message.
            apc_msgs::FrameState frame;
            frame.frame_id = id;
            frame.frame_key = key;
            tf::poseEigenToMsg(iter->second, frame.frame_pose);
            state.frames.push_back(frame);

            // If the frame corresponds to an object, construct the object message.
            if (iter->first.find("bin") != 0) {
                apc_msgs::ObjectState object;
                object.object_id = id;
                object.object_key = key;
                tf::poseEigenToMsg(iter->second, object.object_pose);
                state.objects.push_back(object);
            }
        }
    }

    void MotionPlanningFrame::computeFullyConnectedPlan(const robot_state::RobotState& start,
                                                        apc_msgs::PrimitivePlan& plan,
                                                        bool start_grasped)
    {
        // Insert additional actions to connect the previous robot
        // state to the next start state.
        std::vector<apc_msgs::PrimitiveAction> actions;
        robot_state::RobotState prev_state = start;
        robot_state::RobotState next_state = start;
        for (int i = 0; i < plan.actions.size(); i++) {

            setStateFromPoint(next_state, plan.actions[i].joint_trajectory.joint_names,
                              plan.actions[i].joint_trajectory.points.front());

            if (next_state.distance(prev_state) > 1e-10) {
                apc_msgs::PrimitiveAction action;
                action.action_name = "vvvvv";
                action.group_id = plan.actions[i].group_id;
                action.joint_trajectory.joint_names = plan.actions[i].joint_trajectory.joint_names;

                appendStateToAction(action, prev_state);
                appendStateToAction(action, next_state);

                // check robot state for attached objects.
                if (!plan.actions[i].object_id.empty() && start_grasped) {
                    action.grasp = true;
                    action.object_id = plan.actions[i].object_id;
                    action.object_key = plan.actions[i].object_key;
                    action.attached_link_id = plan.actions[i].attached_link_id;
                    action.object_trajectory = plan.actions[i].object_trajectory;
                }
                // If the previous action was a pregrasp, stay in pregrasp.
                if (i > 0 && is_action_pregrasp(plan.actions[i-1])) {
                    action.object_id = plan.actions[i].object_id;
                    action.object_key = plan.actions[i].object_key;
                    action.attached_link_id = plan.actions[i].attached_link_id;
                }

                // If the next action is a grasp, the connecting action we've
                // created may come into contact with the grasp object. To avoid
                // this problem, we set object frame and key into the connecting
                // action. FIXME
                if (is_action_grasp(plan.actions[i])) {
                // if (plan.actions[i].grasp) {
                    // action.grasp = plan.actions[i].grasp;
                    action.object_id = plan.actions[i].object_id;
                    action.object_key = plan.actions[i].object_key;
                    action.attached_link_id = plan.actions[i].attached_link_id;
                }
                // If both the previous and next actions are grasping,
                // then this should be a post-grasp trajectory.
                if (i > 0 && (is_action_grasp(plan.actions[i-1]) || is_action_postgrasp(plan.actions[i-1])) &&
                             (is_action_grasp(plan.actions[i])   || is_action_postgrasp(plan.actions[i]))) {
                    action.grasp = true;
                    action.object_id = plan.actions[i].object_id;
                    action.object_key = plan.actions[i].object_key;
                    action.attached_link_id = plan.actions[i].attached_link_id;
                    action.object_trajectory = plan.actions[i].object_trajectory;
                }

                ROS_DEBUG("Inserting action %s to connect previous robot state with next robot state",
                          action.action_name.c_str());

                setStateFromPoint(prev_state, action.joint_trajectory.joint_names,
                                  action.joint_trajectory.points.back());

                // Compute distance between previous and next states.
                // Assert that they are less than epsilon.
                const std::vector<const moveit::core::JointModel*> joint_models =
                    prev_state.getRobotModel()->getActiveJointModels();
                for (int j = 0; j < joint_models.size(); j++) {
                    int pi = joint_models[j]->getFirstVariableIndex();
                    int ni = joint_models[j]->getFirstVariableIndex();
                    double pd = prev_state.getVariablePosition(pi);
                    double nd = next_state.getVariablePosition(pi);
                    std::stringstream iss;
                    iss << action << "next action\n" << plan.actions[i];
                    APC_ASSERT(joint_models[j]->distance(&pd, &nd) <= 1e-10,
                               "Failed to connect previous robot state with next state\n"
                               "joint %s prev %f next %f\n"
                               "action\n %s",
                               joint_models[j]->getName().c_str(), pd, nd, iss.str().c_str());
                }

                actions.push_back(action);
            }
            // The next start state is not epsilon away, but the end state of
            // that trajectory may be distant. Therefore, update the next state
            // to the values at the end of the trajectory.
            else {
                setStateFromPoint(next_state, plan.actions[i].joint_trajectory.joint_names,
                                  plan.actions[i].joint_trajectory.points.back());
            }

            setStateFromPoint(next_state, plan.actions[i].joint_trajectory.joint_names,
                              plan.actions[i].joint_trajectory.points.back());
            setStateFromPoint(prev_state, plan.actions[i].joint_trajectory.joint_names,
                              plan.actions[i].joint_trajectory.points.back());
            actions.push_back(plan.actions[i]);

            APC_ASSERT(next_state.distance(prev_state) < 1e-7,
                       "Failed to fulfill post-condition: previous state matching next state");
        }
        plan.actions = actions;
    }

    bool MotionPlanningFrame::doesActionMoveAnItem(const apc_msgs::PrimitiveAction& action)
    {
        // If we do not have sufficient information to reason about
        // this, return false.
        if (action.object_id.empty() || action.object_key.empty() ||
            action.attached_link_id.empty() || action.object_trajectory.poses.empty())
            return false;
        // APC_ASSERT(false,
        //            "Remind Eric to write this piece of code");
        return true;
    }

    void MotionPlanningFrame::setBinStatesToBinStatesMessage(std::vector<apc_msgs::BinState>& bin_states,
                                                             const KeyPoseMap& world_state)
    {
        bin_states.clear();
        std::vector<std::string> item_keys;
        std::vector<std::string> item_ids;
        for (char c = 'A'; c <= 'L'; c++) {
            std::string bin_id = std::string("bin_") + c;
            if (world_state.find(bin_id) == world_state.end())
                return;
            item_keys = findItemKeysInBin(bin_id, world_state);
            item_ids.clear();
            for (int i = 0; i < item_keys.size(); i++) {
                item_ids.push_back(computeItemIdFromItemKey(item_keys[i]));
            }
            apc_msgs::BinState bin_state;
            bin_state.bin_name = bin_id;
            bin_state.object_list.resize(item_keys.size());
            for (int i = 0; i < item_keys.size(); i++) {
                apc_msgs::ObjectState object_state;
                bin_state.object_list[i].object_id  = item_ids[i];
                bin_state.object_list[i].object_key = item_keys[i];
            }
        }
    }

    KeyPoseMap MotionPlanningFrame::computeDenseMotionPlan(const robot_state::RobotState& start,
                                                           const KeyPoseMap& world,
                                                           apc_msgs::PrimitivePlan& plan,
                                                           int client_index)
    {
        // Copy starting conditions.
        robot_state::RobotState robot_state = start;
        robot_state.enforceBounds(); // Do this because if we start outside of
                                     // the joint limits, we should still drive
                                     // the motors back inside the joint limits.
        KeyPoseMap world_state = world;

        // Create motion planning service message.
        apc_msgs::ComputeDenseMotion srv;
        setWorldKeyPoseToWorldStateMessage(world_state, srv.request.world_state);
        setBinStatesToBinStatesMessage(srv.request.bin_states, world_state);

        // For each action..
        for (int i = 0; i < plan.actions.size(); i++) {
            apc_msgs::PrimitiveAction& action = plan.actions[i];

            ROS_DEBUG("Computing dense trajectory for action %s", action.action_name.c_str());

            // Fill out dense motion planning request.
            robot_state::robotStateToRobotStateMsg(robot_state, srv.request.robot_state);
            setWorldKeyPoseToWorldStateMessage(world_state, srv.request.world_state);
            srv.request.action = action;

            APC_ASSERT(_compute_dense_motion_clients[client_index].call(srv),
                       "Failed call to dense motion planning service");

            // Copy returned action to plan.
            action = srv.response.action;

            // On failure, resize to hold up to the failed action. We do this
            // for visualization of the failed trajectory.
            if (!srv.response.collision_free || !srv.response.valid) {
                plan.actions.resize(i + 1);
            }

            APC_ASSERT(srv.response.collision_free,
                       "Failed to find collision free trajectory");
            APC_ASSERT(srv.response.valid,
                       "Failed to find valid trajectory");

            // Move robot state to goal position.
            setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());

            // Move object to goal position.
            if (is_action_grasp(action) || is_action_postgrasp(action) || is_action_nonprehensile(action)) {
            // if (doesActionMoveAnItem(action)) {
                Eigen::Affine3d T_object_world = world_state[action.object_key];
                Eigen::Affine3d T_link_world = robot_state.getGlobalLinkTransform(action.attached_link_id);
                Eigen::Affine3d T_object_link;
                tf::poseMsgToEigen(action.object_trajectory.poses.back(), T_object_link);
                world_state[action.object_key] = T_link_world * T_object_link;
                ROS_DEBUG_STREAM("Moving object %s from\n" << T_object_world.matrix() <<
                                 "to\n" << world_state[action.object_key].matrix());
            }
        }

        return world_state;
    }

    void MotionPlanningFrame::computeSmoothedPath(apc_msgs::PrimitivePlan& plan)
    {
        #pragma omp parallel for
        for (int i = 0; i < plan.actions.size(); i++) {
            apc_msgs::PrimitiveAction& action = plan.actions[i];
            // Get the number of degrees of freedom requested in this action.
            const int n_dof = action.joint_trajectory.joint_names.size();
            // Convert trajectory into a path, a.k.a. list of eigen vectors.
            std::list<Eigen::VectorXd> P;
            // Add the trajectory.
            Eigen::VectorXd p(n_dof);
            {
                const trajectory_msgs::JointTrajectory& T = action.joint_trajectory;
                for (int i = 0; i < T.points.size(); i++)
                {
                    // Convert trajectory point to eigen vector.
                    for (int j = 0; j < p.rows(); j++)
                        p[j] = T.points[i].positions[j];
                    P.push_back(p);
                }
            }
            // Create velocity and acceleration limits.
            const Eigen::VectorXd max_vel   = 1.0 * Eigen::VectorXd::Ones(n_dof);
            const Eigen::VectorXd max_accel = 0.5 * Eigen::VectorXd::Ones(n_dof);
            // Pass path through Toby's code.
            Trajectory T(Path(P, 0.1), max_vel, max_accel);
            // Abort if the trajectory is not valid.
            if (!T.isValid()) {
                ROS_ERROR("Failed to smooth trajectory for action %s in plan %s",
                          action.action_name.c_str(), plan.plan_name.c_str());
                continue;
            }
            // Insert smoothed trajectory back into aciton.
            const trajectory_msgs::JointTrajectoryPoint start = action.joint_trajectory.points.front();
            const trajectory_msgs::JointTrajectoryPoint end = action.joint_trajectory.points.back();
            action.joint_trajectory.points.clear();
            const double dt = 0.10;
            const double duration = T.getDuration();
            double t = 0.0;
            action.joint_trajectory.points.push_back(start);
            while ( (t += dt) < duration) {
                trajectory_msgs::JointTrajectoryPoint point;
                p = T.getPosition(t);
                for (int i = 0; i < p.size(); i++)
                    point.positions.push_back(p[i]);
                action.joint_trajectory.points.push_back(point);
            }
            action.joint_trajectory.points.push_back(end);
        }
    }

    void MotionPlanningFrame::loadPlanToPreview(const moveit_msgs::RobotState& start_state,
                                                const apc_msgs::PrimitivePlan& plan)
    {
        // Get a robot model.
        const robot_model::RobotModelConstPtr& robot_model = planning_display_->getRobotModel();

        // Construct a new robot trajectory.
        robot_trajectory::RobotTrajectoryPtr display_trajectory(new robot_trajectory::RobotTrajectory(robot_model, ""));

        // Accumulate joint trajectory over entire plan.
        trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
        for (int i = 0; i < plan.actions.size(); i++)
            if (!apc_planning::is_action_stationary(plan.actions[i]))
                appendToTrajectory(trajectory, plan.actions[i].joint_trajectory);

        // Copy current plan over to robot trajectory.
        display_trajectory->setRobotTrajectoryMsg(planning_display_->getPlanningSceneRO()->getCurrentState(),
                                                  trajectory);

        // Swap the plan trajectory into our planning display.
        planning_display_->setTrajectoryToDisplay(display_trajectory);

        // Display trail.
        previewButtonClicked();
    }

    void MotionPlanningFrame::appendToTrajectory(trajectory_msgs::JointTrajectory& first,
                                                 const trajectory_msgs::JointTrajectory& second)
    {
        // If the first plan is empty, copy the second over to the
        // first.
        if (first.joint_names.size() == 0)
        {
            first = second;
            return;
        }
        // Build a map from joint names to joint indexes and of
        // uniqueness of joint names between the two trajectory.
        int j_index = 0;                       // New joint index.
        std::map<std::string, int>  j_map;     // Map from joint name to new index.
        std::map<std::string, bool> j1_unique; // Uniqueness of joint to first trajectory.
        std::map<std::string, bool> j2_unique; // Uniqueness of joint to second trajectory.
        std::vector<std::string>    j_names;   // New order of joint names.
        const std::vector<std::string>& j1 = first.joint_names;
        for (int j = 0; j < j1.size(); j++)
        {
            if (j_map.count(j1[j]) == 0)
                j_map[j1[j]] = j_index++;
            j1_unique[j1[j]] = true;
            j2_unique[j1[j]] = false;
            j_names.push_back(j1[j]);
        }
        const std::vector<std::string>& j2 = second.joint_names;
        for (int j = 0; j < j2.size(); j++)
        {
            if (j_map.count(j2[j]) == 0)
            {
                j_map[j2[j]] = j_index++;
                j2_unique[j2[j]] = true;
                j_names.push_back(j2[j]);
            }
            j1_unique[j2[j]] = false;
        }
        // If velocities and forces are unset, fill them in for the first trajectory.
        int n_dof_first = first.joint_names.size();
        for (int i = 0; i < first.points.size(); i++)
        {
            if (!first.points[i].velocities.size())
                first.points[i].velocities.resize(n_dof_first);
            if (!first.points[i].accelerations.size())
                first.points[i].accelerations.resize(n_dof_first);
        }
        std::map<std::string, int> j2_map; // Map from joint name to joint index in second start state.
        const trajectory_msgs::JointTrajectoryPoint& j2_start = second.points[0];
        for (int i = 0; i < second.joint_names.size(); i++)
            j2_map[second.joint_names[i]] = i;
        // Append the new joint names to the first plan.
        first.joint_names = j_names;
        // Fill in missing joints from the second plan to the first plan.
        typedef std::map<std::string, bool>::const_iterator UniqueJointIterator;
        std::vector<trajectory_msgs::JointTrajectoryPoint>& p1 = first.points;
        if (j2_unique.size() > 0)
            for (int i = 0; i < p1.size(); i++)
                for (UniqueJointIterator iter = j2_unique.begin(); iter != j2_unique.end(); ++iter)
                    if (iter->second)
                    {
                        p1[i].positions.push_back(j2_start.positions[j2_map[iter->first]]);
                        p1[i].velocities.push_back(0);
                        p1[i].accelerations.push_back(0);
                        // p1[i].effort.push_back(0); // The planner does not return effort!
                    }
        // Append the second plan to the first plan.
        const std::vector<trajectory_msgs::JointTrajectoryPoint>& p2 = second.points;
        trajectory_msgs::JointTrajectoryPoint point = p1.back();
        const ros::Duration p1_time = point.time_from_start;
        for (int i = 0; i < p2.size(); i++)
        {
            for (int j = 0; j < j2.size(); j++)
            {
                point.positions[j_map[j2[j]]] = p2[i].positions[j];
                if (p2[i].velocities.size() > 0)
                    point.velocities[j_map[j2[j]]] = p2[i].velocities[j];
                else
                    point.velocities[j_map[j2[j]]] = 0;
                if (p2[i].accelerations.size() > 0)
                    point.accelerations[j_map[j2[j]]] = p2[i].accelerations[j];
                else
                    point.accelerations[j_map[j2[j]]] = 0;
                // point.effort[j_map[j2[j]]] = p2[i].effort[j]; // The planner does not return effort!
            }
            point.time_from_start = p1_time + p2[i].time_from_start;
            p1.push_back(point);
        }

        return;
    }

    apc_msgs::PrimitivePlan MotionPlanningFrame::getPrimitivePlanFromActiveActions()
    {
        // Get the list of active goals (waypoints) to follow.
        QListWidget* active_actions = ui_->active_actions_list;

        // Create an empty plan.
        apc_msgs::PrimitivePlan plan;

        // Appeach each active action to the plan.
        for (int i = 0; i < active_actions->count(); i++)
        {
            // Get the plan stored in the active action item.
            apc_msgs::PrimitiveAction stored_action =
                getMessageFromUserData<apc_msgs::PrimitiveAction>(active_actions->item(i)->data(Qt::UserRole));

            // Append the active action item if it is not an connecting action.
            // if (stored_action.action_name != "vvvvv")
                plan.actions.push_back(stored_action);
        }

        return plan;
    }

    void MotionPlanningFrame::loadPrimitivePlanToActiveActions(const apc_msgs::PrimitivePlan& plan)
    {
        QListWidget* active_actions = ui_->active_actions_list;
        active_actions->clear();
        std::vector<QVariant> data;
        saveActionToData(plan.actions, data);
        for (int i = 0; i < plan.actions.size(); i++) {
            const apc_msgs::PrimitiveAction& action = plan.actions[i];
            if (action.action_name == "vvvvv")
                continue;
            QListWidgetItem* item = new QListWidgetItem;
            item->setFlags(item->flags() | Qt::ItemIsEditable);
            item->setData(Qt::UserRole, data[i]);
            item->setText(QString::fromStdString(action.action_name));
            active_actions->insertItem(i, item);
        }
    }

    void MotionPlanningFrame::stripStationaryActions(apc_msgs::PrimitivePlan& plan)
    {
        apc_msgs::PrimitivePlan stripped_plan;
        stripped_plan.plan_name = plan.plan_name;
        for (int i = 0; i < plan.actions.size(); i++) {
            if (is_action_stationary(plan.actions[i]))
                continue;
            stripped_plan.actions.push_back(plan.actions[i]);
        }
        plan = stripped_plan;
    }

    void MotionPlanningFrame::computePlan(apc_msgs::PrimitivePlan& plan,
                                          const robot_state::RobotState start_state,
                                          const KeyPoseMap& world_state,
                                          int client_index,
                                          bool start_grasped)
    {
        apc_planning::resetPlanJointTrajectories(plan);
        computeNearestFrameAndObjectKeysPartial(start_state, world_state, plan);
        computeActionJointTrajectoryPoints(start_state, world_state, plan);
        computeFullyConnectedPlan(start_state, plan, start_grasped);

        // computeLinearInterpolatedTrajectory(plan, start_state, world_state);
        apc_planning::partitionPlanBySubgroups(plan, start_state);

        apc_planning::validatePlanningArguements(plan);
        apc_planning::clampJointLimitsInPlan(plan, start_state);

        apc_planning::assertPlanningPreconditions(plan, start_state, world_state);

        computeDenseMotionPlan(start_state, world_state, plan, client_index);
    }

    void MotionPlanningFrame::computePlanButtonClicked()
    {
        // Reset last computed plan.
        primitive_plan_.reset(new apc_msgs::PrimitivePlan);

        // Aggregate the active actions into a plan.
        apc_msgs::PrimitivePlan plan = getPrimitivePlanFromActiveActions();

        // Clear all keys in the plan.
        if (!ui_->plan_raw_checkbox->isChecked()) {
            for (int i = 0; i < plan.actions.size(); i++) {
                plan.actions[i].frame_key = "";
                plan.actions[i].object_key = "";
            }
        }

        // Get the starting state.
        robot_state::RobotState start_state = planning_display_->getPlanningSceneRO()->getCurrentState();

        // If live starts are disabled, set the start state to the joint angles in the first action.
        if (!ui_->live_start_checkbox->isChecked()) {
            apc_msgs::PrimitivePlan bananas;
            bananas.actions.push_back(plan.actions[0]);
            KeyPoseMap world_state = computeWorldKeyPoseMap();
            computeNearestFrameAndObjectKeys(start_state, world_state, bananas);
            setStateFromAction(start_state, world_state, bananas.actions[0], 0);
        }

        // Compute the motion plan.
        try {
            KeyPoseMap world_state = computeWorldKeyPoseMap();

            // If the 'plan raw' checkbox is not checked, do the full planning
            // pipeline computation.
            if (!ui_->plan_raw_checkbox->isChecked()) {
                computePlan(plan, start_state, world_state);
            }
            // If the 'plan raw' checkbox is checked however, strip the joint
            // trajectories to the minimum and resend the plan for planning.
            else {
                // apc_planning::resetPlanJointTrajectories(plan);
                // apc_planning::validatePlanningArguements(plan);
                computeDenseMotionPlan(start_state, world_state, plan, 0);
            }

            computeSmoothedPath(plan);
            *primitive_plan_ = plan;
            // planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::loadPrimitivePlanToActiveActions, this, plan),
            //                                     "Load plan to active actions");
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Caught exception %s", error.what());

            // Still display trajectory on exception.
            computeSmoothedPath(plan);
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(start_state, start_state_msg);
            loadPlanToPreview(start_state_msg, plan);
            // loadPlanToActiveActions(plan);
            return;
        }

        // Copy trajectory over to display.
        {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(start_state, start_state_msg);
            loadPlanToPreview(start_state_msg, plan);
            // loadPlanToActiveActions(plan);
        }
    }

    void MotionPlanningFrame::computeExecute(const apc_msgs::PrimitivePlan& plan)
    {
        if (!execute_client_)
        {
            typedef actionlib::SimpleActionClient<apc_msgs::FollowPrimitivePlanAction> ActionClient;

            // Create an action client that spins its own thread.
            execute_client_.reset(new ActionClient("/crichton/trajectory_bridge", true));
        }

        if (!execute_client_->isServerConnected())
            if (!execute_client_->waitForServer(ros::Duration(1.0)))
            {
                ROS_ERROR("Unable to connect to action server.");
                return;
            }
            else
                planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::initExecuteProgressLabel, this));

        // Get goal.
        apc_msgs::FollowPrimitivePlanGoal goal;
        goal.plan.actions.resize(1);

        for (int i = 0; i < plan.actions.size(); i++) {
            // Write the action.
            goal.plan.actions[0] = plan.actions[i];

            // Sanitize the plan.
            apc_planning::preprocessPlanBeforeExecution(goal.plan,
                                                        *planning_display_->getQueryGoalState());

            // Send the goal.
            execute_client_->sendGoal(goal,
                                      boost::bind(&MotionPlanningFrame::executeDoneCallback, this, _1, _2),
                                      boost::bind(&MotionPlanningFrame::executeActiveCallback, this),
                                      boost::bind(&MotionPlanningFrame::executeFeedbackCallback, this, _1));

            // Verify that the goal has been executed in < 30 seconds.
            try {
                APC_ASSERT(execute_client_->waitForResult(ros::Duration(30.0)),
                           "Failed to complete action in less than 30 seconds\n"
                           "          plan name: %s\n"
                           "    action[%d] name: %s\n"
                           "       action group: %s\n",
                           goal.plan.plan_name.c_str(), i, goal.plan.actions[0].action_name.c_str(),
                           goal.plan.actions[0].group_id.c_str());
            } catch (apc_exception::Exception& error) {
                execute_client_->cancelAllGoals();
                throw error;
            }
        }
    }

    void MotionPlanningFrame::computeExecuteButtonClicked()
    {
        if (!primitive_plan_)
        {
            ROS_ERROR("No plan to execute");
            return;
        }

        computeExecute(*primitive_plan_);
    }

    void MotionPlanningFrame::executeActiveCallback()
    {
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateExecuteActive, this));
    }

    void MotionPlanningFrame::executeFeedbackCallback(const apc_msgs::FollowPrimitivePlanFeedbackConstPtr& feedback)
    {
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateExecuteFeedback,
                                                      this,
                                                      *feedback));
    }

    void MotionPlanningFrame::executeDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                  const apc_msgs::FollowPrimitivePlanResultConstPtr& result)
    {
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateExecuteDone,
                                                      this,
                                                      state,
                                                      *result));
    }


    void MotionPlanningFrame::computeLinearInterpolatedTrajectory(apc_msgs::PrimitivePlan& plan,
                                                         const robot_state::RobotState& start_state,
                                                         const KeyPoseMap& world_state)
    {
        typedef apc_msgs::PrimitiveAction Action;
        typedef apc_msgs::PrimitivePlan Plan;
        typedef trajectory_msgs::JointTrajectoryPoint Point;
        typedef std::vector<std::string> JointNames;
        // PRECONDITIONS:
        // The plan is fully connected.
        // The robot is already IK'd to the correct locations.
        robot_state::RobotState robot_state = start_state;
        for (int i = 0; i < plan.actions.size(); i++) {
            computeLinearInterpolatedTrajectory(plan.actions[i], start_state, world_state);
            // Post condition; robot state is upto date.
            setStateFromPoint(robot_state, plan.actions[i].joint_trajectory.joint_names, plan.actions[i].joint_trajectory.points.back());
            // Make sure that the endpoints between the two
            // trajectories stay connected
            if (i == plan.actions.size()-1)
                continue;
            Action& prev = plan.actions[i];
            Action& next = plan.actions[i+1];
            robot_state::RobotState next_state = robot_state;
            setStateFromPoint(next_state, next.joint_trajectory.joint_names, next.joint_trajectory.points.front());
            if (next_state.distance(robot_state) > 1e-10) {
                // APC_ASSERT(prev.group_id == next.group_id, "Disconnect between group ids?");
                // overwrite starting joint angles with end of the prev state.
                Action tmp = next;
                appendStateToAction(tmp, robot_state);
                // setting starting joint angles to the end of the prev state.
                next.joint_trajectory.points[0] = tmp.joint_trajectory.points.back();
            }
        }
    }

    void MotionPlanningFrame::computeLinearInterpolatedTrajectory(apc_msgs::PrimitiveAction& action,
                                                                 const robot_state::RobotState& start_state,
                                                                 const KeyPoseMap& world_state)
    {
        // PRECONDITIONS:
        // Operate only on joint trajectory.
        // Robot is already IK'd to the correct locations.
        if (is_action_stationary(action))
            return;
        robot_state::RobotState robot_state = start_state;
        // Get group for IK
        kinematics::KinematicsBaseConstPtr solver = robot_state.getJointModelGroup(action.group_id)->getSolverInstance();
        std::string group_id = action.group_id;
        // If no solver exists, try searching the subgroups for a solver.
        if (!solver) {
            std::vector<const moveit::core::JointModelGroup*> subgroups;
            robot_state.getJointModelGroup(action.group_id)->getSubgroups(subgroups);
            for (int i = 0; i < subgroups.size(); i++)
                if (solver = subgroups[i]->getSolverInstance()) {
                    group_id = subgroups[i]->getName();
                    break;
                }
        }
        if (!solver) {
            return;
        }
        APC_ASSERT(solver,
                   "Missing IK solver for group %s in action %s", action.group_id.c_str(), action.action_name.c_str());
        // Get eef link.
        std::string eef_link = computeEefLink(action.group_id);
        // Read off starting eef transform.
        setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.front());
        Eigen::Affine3d T_start = robot_state.getGlobalLinkTransform(eef_link);
        // Read off ending eef transform.
        setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
        Eigen::Affine3d T_end = robot_state.getGlobalLinkTransform(eef_link);
        // Set robot state to the front of the trajectory.
        setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.front());
        // Interp between points.
        apc_msgs::PrimitiveAction linear_action = action;
        linear_action.joint_trajectory.points.clear();
        int num_steps = 10;
        // linear_action.eef_trajectory.poses.resize(num_steps);
        linear_action.eef_trajectory.poses.clear();
        for (int i = 0; i < num_steps; i++) {
            double t = (double) i / (double) (num_steps - 1);
            // Lerp the translation component.
            Eigen::Vector3d v_t = (1 - t) * T_start.translation() + (t) * T_end.translation();
            // Slerp the rotation component.
            Eigen::Quaterniond q_start(T_start.linear());
            Eigen::Quaterniond q_end(T_end.linear());
            Eigen::Quaterniond q_t = q_start.slerp(t, q_end);
            // Recreate the affine transform.
            Eigen::Affine3d T_t;
            T_t.translation() = v_t;
            T_t.linear() = q_t.matrix();
            // IK the robot to the next state.
            geometry_msgs::Pose pose_identity;
            pose_identity.position.x = 0;
            pose_identity.position.y = 0;
            pose_identity.position.z = 0;
            pose_identity.orientation.x = 0;
            pose_identity.orientation.y = 0;
            pose_identity.orientation.z = 0;
            pose_identity.orientation.w = 1;
            const std::string frame_id = "test_frame";
            setStateFromIK(robot_state, eef_link, group_id, frame_id, T_t, pose_identity, false);
            // Save IK'd state into the action.
            appendStateToAction(linear_action, robot_state);
            // Set eef pose into action. NOPE
            // tf::poseEigenToMsg(T_t, linear_action.eef_trajectory.poses[i]);
        }
        linear_action.interpolate_cartesian = true;
        action = linear_action;
    }

    void MotionPlanningFrame::computeOffsetGrasps(std::vector<apc_msgs::PrimitivePlan>& offset_grasps,
                                                  const apc_msgs::PrimitivePlan& grasp_,
                                                  const robot_state::RobotState& start,
                                                  const KeyPoseMap& world)
    {
        // Precondition:
        // Grasp is reachable and has been IK'd to.
        // Grasp IK has been saved to joint angles.
        // All grasps are two finger grasps.
        APC_ASSERT(grasp_.actions.size() == 1, "Grasp more than one action");
        apc_msgs::PrimitiveAction grasp = grasp_.actions[0];
        APC_ASSERT(is_action_grasp(grasp), "Not a grasp!");
        std::stringstream ss;
        ss << grasp;
        APC_ASSERT(is_action_stationary(grasp), "Not a sationary! %s", ss.str().c_str());
        robot_state::RobotState robot_state = start;
        // Set grasp to robot.
        setStateFromPoint(robot_state, grasp.joint_trajectory.joint_names, grasp.joint_trajectory.points.front());
        // Compute the finger grasp frame.
        std::string group_id = grasp.group_id;
        kinematics::KinematicsBaseConstPtr solver = robot_state.getJointModelGroup(group_id)->getSolverInstance();
        // If no solver exists, try searching the subgroups for a solver.
        if (!solver) {
            std::vector<const moveit::core::JointModelGroup*> subgroups;
            robot_state.getJointModelGroup(group_id)->getSubgroups(subgroups);
            for (int i = 0; i < subgroups.size(); i++)
                if (solver = subgroups[i]->getSolverInstance()) {
                    group_id = subgroups[i]->getName();
                    break;
                }
        }
        if (!solver) {
            return;
        }
        APC_ASSERT(solver,
                   "Missing IK solver for group %s in action %s", group_id.c_str(), grasp.action_name.c_str());
        // Determine which side of the robot we are using.
        using namespace boost::xpressive;
        sregex rex = sregex::compile("^.*(left|right).*");
        smatch what;
        APC_ASSERT(regex_match(group_id, what, rex), "Can't determine side of group");
        std::string side = what[1];
        // Get the finger positions.
        std::string l1 = "crichton_" + side + "_finger_13_link";
        std::string l2 = "crichton_" + side + "_finger_23_link";
        Eigen::Affine3d T_l1 = robot_state.getGlobalLinkTransform(l1);
        Eigen::Affine3d T_l2 = robot_state.getGlobalLinkTransform(l2);
        // ROS_INFO_STREAM("T_l1\n" << T_l1.matrix());
        // ROS_INFO_STREAM("T_l2\n" << T_l2.matrix());
        // Compute connecting line and midpoint between the two links.
        Eigen::Vector3d pt_mid = 0.5 * (T_l1.translation() + T_l2.translation());
        // ROS_INFO_STREAM("pt_mid " << pt_mid.transpose());
        Eigen::Vector3d l_g = (T_l2.translation() - T_l1.translation()).normalized();
        // Compute the grasp center frame.
        Eigen::Affine3d T_g_w;
        T_g_w.linear() = T_l1.linear();
        T_g_w.translation() = pt_mid;
        // ROS_INFO_STREAM("T_g_w\n" << T_g_w.matrix());
        // Compute eef in world frame.
        Eigen::Affine3d T_e_w = robot_state.getGlobalLinkTransform(grasp.eef_link_id);
        // ROS_INFO_STREAM("T_e_w\n" << T_e_w.matrix());
        APC_ASSERT(world.count(grasp.object_key) > 0, "Failed to find object key %s in world", grasp.object_key.c_str());
        Eigen::Affine3d T_o_w = world.find(grasp.object_key)->second;
        // ROS_INFO_STREAM("T_o_w\n" << T_o_w.matrix());
        // Compute normal to midpoint (towards the hand).
        Eigen::Vector3d n_p = -T_l1.linear().col(2);
        // Generate random rotations around that cone.
        int num_samples = 10;
        double phi = 20.0 / 180.0 * M_PI; // Width of cone from center line.
        typedef boost::random::mt19937 gen_type;
        gen_type zgen;
        zgen.seed(static_cast<unsigned int>(std::time(0)));
        boost::uniform_real<> zuni(cos(phi), 1);
        boost::variate_generator<gen_type&, boost::uniform_real<double> > zrand(zgen, zuni);
        gen_type tgen;
        tgen.seed(static_cast<unsigned int>(std::time(0)));
        boost::uniform_real<> tuni(0, 2 * M_PI);
        boost::variate_generator<gen_type&, boost::uniform_real<double> > trand(tgen, tuni);
        for (int i = 0; i < num_samples; i++) {
            // Generate random vector direction.
            double z = zrand();
            double t = trand();
            Eigen::Vector3d rdir;
            rdir << sqrt(1 - z*z)*cos(t), sqrt(1 - z*z)*sin(t), z;
            // Get north pole.
            Eigen::Vector3d npole;
            npole << 0, 0, 1;
            // Compute shortest rotation from npole to rdir.
            Eigen::Vector3d a = npole.cross(rdir);
            Eigen::Quaterniond q;
            q.x() = a.x();
            q.y() = a.y();
            q.z() = a.z();
            q.w() = 1 + npole.dot(rdir);
            q.normalize();
            Eigen::Affine3d T_r_g = Eigen::Affine3d::Identity();
            T_r_g.linear() = q.matrix();
            // ROS_INFO_STREAM("T_r_g\n" << T_r_g.matrix());
            // Rotate hand about grasp point.
            // Move eef position into grasp frame.
            Eigen::Affine3d T_e_g = T_g_w.inverse() * T_e_w;
            // ROS_INFO_STREAM("T_e_g\n" << T_e_g.matrix());
            // Rotate eef position by random.
            Eigen::Affine3d T_e_r = T_r_g.inverse() * T_e_g;
            // ROS_INFO_STREAM("T_e_r\n" << T_e_r.matrix());
            // Move eef position back into world frame.
            Eigen::Affine3d T_new_w = T_g_w * T_e_r;
            // ROS_INFO_STREAM("T_new_w\n" << T_new_w.matrix());
            // IK to new position.
            robot_state::RobotState copy = robot_state;
            geometry_msgs::Pose pose_id;
            pose_id.position.x = 0; pose_id.position.y = 0; pose_id.position.z = 0;
            pose_id.orientation.x = 0; pose_id.orientation.y = 0; pose_id.orientation.z = 0;
            pose_id.orientation.w = 1.0;
            setStateFromIK(copy, grasp.eef_link_id, group_id, grasp.frame_id, T_new_w, pose_id, false);
            apc_msgs::PrimitiveAction offset_grasp = grasp;
            offset_grasp.joint_trajectory.points.clear();
            appendStateToAction(offset_grasp, copy);
            appendStateToAction(offset_grasp, copy);
            offset_grasp.frame_id = "";
            offset_grasp.frame_key = "";
            apc_msgs::PrimitivePlan offset_grasp_plan;
            offset_grasp_plan.actions.push_back(offset_grasp);
            offset_grasps.push_back(offset_grasp_plan);
            // ROS_INFO("Computed an offset grasp");
        }

    }

    void MotionPlanningFrame::computeEnter(std::vector<apc_msgs::PrimitivePlan>& pregrasps,
                                           const apc_msgs::PrimitivePlan& bin_pose_,
                                           const robot_state::RobotState& start,
                                           const KeyPoseMap& world)
    {
        typedef apc_msgs::PrimitivePlan Plan;
        typedef std::vector<Plan> PlanList;
        typedef apc_msgs::PrimitiveAction Action;
        // Set robot to end of trajectoy.
        robot_state::RobotState robot_state = start;
        setStateToPlanJointTrajectoryEnd(robot_state, bin_pose_);
        // Compute enter trajecotyrs.
        PlanList interpd_plans;
        for (int i = 0; i < pregrasps.size(); i++) {
            Action bin_pose = bin_pose_.actions.back(); // might be okay robot state will be at correct location
            Action pregrasp = pregrasps[i].actions.front();
            // Partition plan so that the fingers open first, then we linearly interpolate.
            Action pregrasp_hand = apc_planning::getSubgroupAction(".*hand", pregrasp, robot_state);
            Action pregrasp_pose = apc_planning::getSubgroupAction(".*arm", pregrasp, robot_state);
            Plan pose_to_pregrasp;
            pose_to_pregrasp.actions.push_back(bin_pose);
            pose_to_pregrasp.actions.push_back(pregrasp_hand);
            pose_to_pregrasp.actions.push_back(pregrasp_pose);
            for (int j =1 ; j < pregrasps[i].actions.size(); j++) {
                const Action& grasp = pregrasps[i].actions[j];
                Action grasp_hand = apc_planning::getSubgroupAction(".*hand", grasp, robot_state); // split so that linear interp will not overwrite hand movements.
                Action grasp_pose = apc_planning::getSubgroupAction(".*arm", grasp, robot_state);
                pose_to_pregrasp.actions.push_back(grasp_hand);
                pose_to_pregrasp.actions.push_back(grasp_pose);
            }
            // Fill out things
            try {
                computeNearestFrameAndObjectKeysPartial(robot_state, world, pose_to_pregrasp);
                computeActionJointTrajectoryPoints(robot_state, world, pose_to_pregrasp);
                computeFullyConnectedPlan(robot_state, pose_to_pregrasp);
                // interpt
                computeLinearInterpolatedTrajectory(pose_to_pregrasp, robot_state, world);
            } catch (apc_exception::Exception& e) {
                ROS_DEBUG("Failed to interpolate this %s", e.what());
                continue;
            }
            //
            interpd_plans.push_back(pose_to_pregrasp);
        }
        pregrasps = interpd_plans;
    }

    void MotionPlanningFrame::computeExit(std::vector<apc_msgs::PrimitivePlan>& grasps,
                                           const apc_msgs::PrimitivePlan& bin_pose_,
                                           const robot_state::RobotState& start,
                                           const KeyPoseMap& world)
    {
        // Preconditions:
        // grasps is just the grasp?
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;
        //
        using namespace boost::xpressive;
        sregex rex = sregex::compile("^.*(left|right).*");
        smatch what;
        APC_ASSERT(regex_match(grasps[0].actions[0].group_id, what, rex), "Can't determine side of group");
        std::string side = what[1];
        // Get the arm only pose of the end bin state.
        Action exit_pose;
        bool found_exit = false;
        for (int i = bin_pose_.actions.size()-1; i >= 0; i--) {
            Action bin_pose = bin_pose_.actions[i];
            try {
                std::stringstream ss;
                ss << ".*" << side << ".*arm";
                std::string expr = ss.str();
                ROS_INFO_STREAM("expr: "<< expr);
                exit_pose = apc_planning::getSubgroupAction(expr, bin_pose, start);
                found_exit = true;
                break;
            } catch (apc_exception::Exception& e) {
                ROS_DEBUG("%s", e.what());
            }
        }
        APC_ASSERT(found_exit, "Failed to find last arm pose before bin");
        for (int i = 0; i < grasps.size(); i++) {
            // Set robot to end of grasp.
            robot_state::RobotState robot_state = start;
            setStateToPlanJointTrajectoryEnd(robot_state, grasps[i]);
            // last grasp pose
            Action grasp_end = grasps[i].actions.back();
            Plan grasp_to_pose;
            grasp_to_pose.actions.push_back(grasp_end);
            grasp_to_pose.actions.push_back(exit_pose);
            // set object into the exit pose! can't forget we're still grasping :)
            {
                Action& exit = grasp_to_pose.actions.back();
                exit.object_id = grasp_end.object_id;
                exit.object_key = grasp_end.object_key;
                exit.object_trajectory = grasp_end.object_trajectory;
                exit.attached_link_id = grasp_end.attached_link_id;
                exit.grasp = grasp_end.grasp;
                exit.action_name = "exit_" + exit.action_name;
            }
            // Fill out things
            try {
                computeNearestFrameAndObjectKeysPartial(robot_state, world, grasp_to_pose);
                computeActionJointTrajectoryPoints(robot_state, world, grasp_to_pose);
                computeFullyConnectedPlan(robot_state, grasp_to_pose);
                // interpt
                computeLinearInterpolatedTrajectory(grasp_to_pose, robot_state, world);
            } catch (apc_exception::Exception& e) {
                ROS_DEBUG("Failed to interpolate this %s", e.what());
                continue;
            }
            grasps[i].actions.insert(grasps[i].actions.end(),
                                     grasp_to_pose.actions.begin() + 1,
                                     grasp_to_pose.actions.end());
        }
    }

    void MotionPlanningFrame::stripToJointAngleTrajectoryActionsOnly(apc_msgs::PrimitivePlan& plan)
    {
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;
        for (int i = 0; i < plan.actions.size(); i++) {
            Action& action = plan.actions[i];
            action.frame_id = "";
            action.frame_key = "";
            action.eef_link_id = "";
            action.eef_trajectory.poses.clear();
            if (action.object_trajectory.poses.size() != 0 &&
                action.object_trajectory.poses.size() != action.joint_trajectory.points.size()) {
                action.object_trajectory.poses.resize(action.joint_trajectory.points.size(),
                                                      action.object_trajectory.poses.back());
            }
        }
    }

    void MotionPlanningFrame::computeCheckCollisions(std::vector<apc_msgs::PrimitivePlan>& plans,
                                                     const robot_state::RobotState& start,
                                                     const KeyPoseMap& world)
    {
        typedef apc_msgs::CheckCollisions CC;
        std::vector<CC> ccs;
        int num_cc = check_collisions_clients_.size();
        int num_p = plans.size();
        int num_s = std::ceil((double)num_p / (double)num_cc);
        int p = 0;
        for (int i = 0; i < num_cc; i++) {
            CC srv;
            setWorldKeyPoseToWorldStateMessage(world, srv.request.world_state);
            setBinStatesToBinStatesMessage(srv.request.bin_states, world);
            robot_state::robotStateToRobotStateMsg(start, srv.request.robot_state);
            while (p < i * num_s && p < num_p) {
                srv.request.plans.push_back(plans[p++]);
            }
            ccs.push_back(srv);
        }
        //
        ROS_INFO("Checking collisions...");
#pragma omp parallel num_threads(8)
        {
#pragma omp for
            for (int i = 0; i < ccs.size(); i++) {
                try {
                    int index = omp_get_thread_num() % check_collisions_clients_.size();
                    APC_ASSERT(check_collisions_clients_[index].call(ccs[i]),
                               "Failed call to check collisions service");
                } catch (apc_exception::Exception& error) {
                    ROS_ERROR("Error\n %s", error.what());
                }
            }
        }
        ROS_INFO("...done");
        // Place valid plans back into the larger list.
        plans.clear();
        for (int i =0; i < ccs.size(); i++) {
            plans.insert(plans.end(),
                         ccs[i].response.plans.begin(),
                         ccs[i].response.plans.end());
        }
    }

    void MotionPlanningFrame::computePreGrasps(std::vector<apc_msgs::PrimitivePlan>& grasps,
                                               const robot_state::RobotState& start,
                                               const KeyPoseMap& world)
    {
        apc_msgs::ComputePreGrasps pregrasp_srv;
        pregrasp_srv.request.grasps = grasps;
        setWorldKeyPoseToWorldStateMessage(world, pregrasp_srv.request.world_state);
        setBinStatesToBinStatesMessage(pregrasp_srv.request.bin_states, world);
        try {
            APC_ASSERT(compute_pregrasps_client_.call(pregrasp_srv),
                       "Failed call to compute pregrasp client");
            // pregrasps.actions = pregrasp_srv.response.pregrasps;
            // planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::loadPlanToActiveActions, this, pregrasps));
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Caught error\n%s", error.what());
            throw error;
        }
        grasps = pregrasp_srv.response.pregrasps;
        // add some flavor to the names so that we can distinguish them later on.
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;
        for (int i = 0; i < grasps.size(); i++) {
            Plan& G = grasps[i];
            for (int j = 0; j < G.actions.size();j++) {
                if (j == 0)
                    G.actions[j].action_name = "pregrasp_" + G.actions[j].action_name;
                else
                    G.actions[j].action_name = "grasp_" + G.actions[j].action_name;
            }
        }
    }

    void MotionPlanningFrame::computePick(std::vector<apc_msgs::PrimitivePlan>& picks,
                                          const std::string& item_id,
                                          const std::string& bin_id,
                                          const apc_msgs::PrimitivePlan& bin_pose,
                                          const robot_state::RobotState& start,
                                          const KeyPoseMap& world)
    {
        // get bin bppose
                        typedef std::vector<apc_msgs::PrimitivePlan> PlanList;
        typedef apc_msgs::PrimitivePlan Plan;
        typedef apc_msgs::PrimitiveAction Action;

        ROS_WARN_STREAM(bin_pose);

        // PlanList bin_poses;
        // retrieveBinPoses(bin_poses, bin_id);
        // ik to bin pose
        robot_state::RobotState robot_state = start;
        setStateToPlanJointTrajectoryEnd(robot_state,
                                         bin_pose);
        KeyPoseMap world_state = world;
        //
        // Get all grasps for an item.
        std::vector<apc_msgs::PrimitivePlan> db_grasps;
        retrieveItemGrasps(db_grasps, item_id);
        //
        std::vector<apc_msgs::PrimitivePlan> item_grasps;
        int vanilla_grasp_count = db_grasps.size();
        for (int i = 0; i < vanilla_grasp_count; i++) {
            apc_msgs::PrimitivePlan grasp = db_grasps[i];
            try {
                computeNearestFrameAndObjectKeys(robot_state, world, grasp);
                computeActionJointTrajectoryPoints(robot_state, world, grasp);
                // ROS_INFO("grasp key %s", grasp.actions[0].object_key.c_str());
                item_grasps.push_back(grasp);
                computeOffsetGrasps(item_grasps, grasp, robot_state, world);
            } catch (apc_exception::Exception& error) {
                // ROS_WARN("Skipping over grasp %s\n%s", db_grasps[i].plan_name.c_str(), error.what());
            }
        }

        // FIXME Something smarter.
        // if (item_grasps.size() > 20)
        //     item_grasps.resize(20);

        // apc_planning::assertPlanningPreconditions(item_grasps, robot_state, world_state);

        try {
            computeCheckCollisions(item_grasps,
                                   robot_state,
                                   world_state);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("%s", error.what());
            throw error;
        }

        // apc_planning::assertPlanningPreconditions(item_grasps, robot_state, world_state);

        try {
            computePreGrasps(item_grasps,
                             robot_state,
                             world_state);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("%s", error.what());
            throw error;
        }

        // apc_planning::assertPlanningPreconditions(item_grasps, robot_state, world_state);

        try {
            computeCheckCollisions(item_grasps,
                                   robot_state,
                                   world_state);
        } catch (apc_exception::Exception& error) {
            ROS_ERROR("%s", error.what());
            throw error;
        }

        // apc_planning::assertPlanningPreconditions(item_grasps, robot_state, world_state);

        PlanList grasp_plans = item_grasps;
        // enter and exit
        try {
            computeEnter(grasp_plans, bin_pose, robot_state, world_state);

            {
                robot_state::RobotState test_robot = robot_state;
                setStateToPlanJointTrajectoryEnd(test_robot, bin_pose);
                apc_planning::assertPlanningPreconditions(grasp_plans, test_robot, world_state);
            }

            computeExit(grasp_plans, bin_pose, robot_state, world_state);

            {
                robot_state::RobotState test_robot = robot_state;
                setStateToPlanJointTrajectoryEnd(test_robot, bin_pose);
                apc_planning::assertPlanningPreconditions(grasp_plans, test_robot, world_state);
            }

        } catch (apc_exception::Exception& error) {
            ROS_ERROR("error %s", error.what());
            throw error;
        }

        // FIXME Something smarter.
        if (grasp_plans.size() > 20)
            grasp_plans.resize(20);

        robot_state::RobotState bin_robot = robot_state;
        setStateToPlanJointTrajectoryEnd(bin_robot, bin_pose);
        PlanList valid_picks;
#pragma omp parallel num_threads(8)
        {
#pragma omp for
            for (int i = 0; i < grasp_plans.size(); i++) {
                try {
                    computeDenseMotionPlan(bin_robot,
                                           world_state,
                                           grasp_plans[i],
                                           omp_get_thread_num() % _compute_dense_motion_clients.size());
#pragma omp critical
                    {
                        valid_picks.push_back(grasp_plans[i]);
                    }
                } catch (apc_exception::Exception& e) {
                    ROS_DEBUG("rejected: %s", e.what());
                }
            }
        }

        apc_msgs::PrimitivePlan diplsying;
        for (int i = 0; i < picks.size(); i++) {
            diplsying.actions.insert(diplsying.actions.end(), valid_picks[i].actions.begin(),
                               valid_picks[i].actions.end());
        }
        loadPlanToActiveActions(diplsying);
        picks = valid_picks;
        // strip all frames. to avoid problmes later on
        typedef std::vector<Plan> PlanList;
        typedef std::vector<Action> ActionList;
        for (PlanList::iterator plan = picks.begin(); plan != picks.end(); plan++) {
            stripToJointAngleTrajectoryActionsOnly(*plan);
        }

    }

}
