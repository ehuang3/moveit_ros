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

#include <moveit/motion_planning_rviz_plugin/apc_eigen_helpers.h>


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
                                             const Eigen::Affine3d& T_frame_world,
                                             const geometry_msgs::Pose& pose_link_frame)
    {
        APC_ASSERT(link_id.size() > 0, "Failed to provide input link");

        // If the group does not have an IK solver, return false.
        bool solver = robot.getJointModelGroup(group_id)->getSolverInstance();
        APC_ASSERT(solver,
                   "Failed to find IK solver for group %s", group_id.c_str());

        // Back out the desired link transform in global coordinates.
        Eigen::Affine3d T_frame_link;
        tf::poseMsgToEigen(pose_link_frame, T_frame_link);
        Eigen::Affine3d T_link_world = T_frame_world * T_frame_link.inverse();

        // Snap to the frame using IK.
        const moveit::core::JointModelGroup* jmg = robot.getJointModelGroup(group_id);
        geometry_msgs::Pose pose_link_world;
        tf::poseEigenToMsg(T_link_world, pose_link_world);
        robot.setFromIK(jmg, pose_link_world, link_id);
        // APC_ASSERT(robot.setFromIK(jmg, pose_link_world, link_id),
        //            "Failed to set %s to pose using %s IK", link_id.c_str(), group_id.c_str());
        robot.update();

        // Manually check whether the new state places the end-effector at the
        // desired IK position.
        Eigen::Affine3d T_ik = robot.getGlobalLinkTransform(link_id);
        APC_ASSERT(apc_eigen::elementWiseMatrixNorm(T_ik, T_link_world) < 1e-2,
                   "Failed to IK group %s to pose; error is %.6f", group_id.c_str(),
                   apc_eigen::elementWiseMatrixNorm(T_ik, T_link_world));
        return true;
    }

    bool MotionPlanningFrame::setAttachedObjectFromAction(robot_state::RobotState& robot_state,
                                                          const KeyPoseMap& world_state,
                                                          const apc_msgs::PrimitiveAction& action,
                                                          const int index)
    {
        APC_ASSERT(!action.object_key.empty(),
                   "Missing object key for object ID %s in action %s", action.object_id.c_str(), action.action_name.c_str());
        APC_ASSERT(world_state.count(action.object_key) > 0,
                   "Missing object key %s in world state", action.object_key.c_str());
        APC_ASSERT(action.object_trajectory.poses.size() > index,
                   "Index %d out of bounds for object trajectory in action %s",
                   index, action.action_name.c_str());
        // Clear attached bodies.
        robot_state.clearAttachedBodies();
        // Get the actual transform from world to object and the
        // desired transform from end-effector to object.
        Eigen::Affine3d T_object_world = world_state.find(action.object_key)->second;
        Eigen::Affine3d T_object_eef;
        tf::poseMsgToEigen(action.object_trajectory.poses[index], T_object_eef);
        // Get object shapes and shape poses from the world and conver
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
        int index = _index;
        // 1. If there is no frame ID, set state based on the joint
        // angles provided in the action.
        if (action.frame_id.empty()) {
            APC_ASSERT(action.joint_trajectory.points.size() > index,
                       "Index %d out of bounds for joint trajectory in action %s",
                       index, action.action_name.c_str());
            setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points[index]);
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
                // For each joint not solveable by the IK solver, set that joint's position manually.
                trajectory_msgs::JointTrajectory no_ik;
                no_ik.points.resize(1);
                const std::vector<std::string>& group_joint_names = action.joint_trajectory.joint_names;
                const std::vector<std::string>& solver_joint_names = solver->getJointNames();
                for (int i = 0; i < action.joint_trajectory.joint_names.size(); i++) {
                    if (std::find(solver_joint_names.begin(), solver_joint_names.end(), group_joint_names[i]) ==
                        solver_joint_names.end()) {
                        APC_ASSERT(action.joint_trajectory.points.size() > index,
                                   "Index %d out of bounds for joint trajectory in action %s",
                                   index, action.action_name.c_str());
                        no_ik.joint_names.push_back(group_joint_names[i]);
                        no_ik.points[0].positions.push_back(action.joint_trajectory.points[index].positions[i]);
                    }
                }
                if (no_ik.joint_names.size() > 0) {
                    // ROS_DEBUG_STREAM("No IK for\n" << no_ik);
                    setStateFromPoint(robot_state, no_ik.joint_names, no_ik.points[0]);
                }
                // Set the remaining joints using IK.
                APC_ASSERT(!action.eef_link_id.empty(),
                           "Missing IK link for frame ID %s in action %s", action.frame_id.c_str(), action.action_name.c_str());
                if (action.eef_trajectory.poses.size() < index) {
                    index = action.eef_trajectory.poses.size() - 1;
                    ROS_WARN("Clamping eef index down");
                }
                APC_ASSERT(action.eef_trajectory.poses.size() > index,
                           "Index %d out of bounds for end-effector trajectory in action %s",
                           index, action.action_name.c_str());
                APC_ASSERT(world_state.count(action.frame_key) > 0,
                           "Missing frame key %s in world state", action.frame_key.c_str());
                setStateFromIK(robot_state, action.eef_link_id, solver->getGroupName(), world_state.find(action.frame_key)->second,
                               action.eef_trajectory.poses[index]);
            } else if (use_joint_angles_if_no_ik_solver) {
                APC_ASSERT(action.joint_trajectory.points.size() > index,
                           "Index %d out of bounds for joint trajectory in action %s",
                           index, action.action_name.c_str());
                setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points[index]);
            }
        }

        // 3. If there is an object to attach, attach that object.
        if (!action.object_id.empty() && !action.object_key.empty()) {
            setAttachedObjectFromAction(robot_state, world_state, action, index);
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
                       "Plan frame/object keys already exist");
        // Compute the nearest keys.
        return computeNearestFrameAndObjectKeysPartial(start, world, plan);
    }

    // TODO Write partial version of this function.
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
            setStateFromAction(robot_state, world_state, action, num_points - 1, true);

            // Move object to goal position.
            if (!action.object_id.empty()) {
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
                APC_ASSERT(action.eef_trajectory.poses.size() > 0,
                           "Missing end-effector trajectory for %s", action.action_name.c_str());
                APC_ASSERT(action.eef_trajectory.poses.size() == action.joint_trajectory.points.size(),
                           "Mismatch between end-effector trajectory points and joint trajectory points for %s",
                           action.action_name.c_str());
                // Rebuild joint trajectory.
                for (int j = 0; j < action.eef_trajectory.poses.size(); j++) {
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
            if (!action.object_id.empty()) {
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
                                                        apc_msgs::PrimitivePlan& plan)
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

                // If the next action is a grasp, the connecting action we've
                // created may come into contact with the grasp object. To avoid
                // this problem, we set object frame and key into the connecting
                // action. FIXME
                if (plan.actions[i].grasp) {
                    // action.grasp = plan.actions[i].grasp;
                    action.object_id = plan.actions[i].object_id;
                    action.object_key = plan.actions[i].object_key;
                }
                // If both the previous and next actions are grasping,
                // then this should be a post-grasp trajectory.
                if (i > 0 && plan.actions[i-1].grasp && plan.actions[i].grasp) {
                    action.grasp = true;
                    action.attached_link_id = plan.actions[i].attached_link_id;
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

                ROS_DEBUG_STREAM("Action connecting:\n" << action);

                actions.push_back(action);
            }
            // The next start state is not epsilon away, but the end state of
            // that trajectory may be distant. Therefore, update the next state
            // to the values at the end of the trajectory.
            else {
                setStateFromPoint(next_state, plan.actions[i].joint_trajectory.joint_names,
                                  plan.actions[i].joint_trajectory.points.back());
            }

            ROS_DEBUG_STREAM("Action to connect to:\n" << plan.actions[i]);

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
            APC_ASSERT(srv.response.collision_free,
                       "Failed to find collision free trajectory");
            APC_ASSERT(srv.response.valid,
                       "Failed to find valid trajectory");

            // Copy returned action to plan.
            action = srv.response.action;

            // Move robot state to goal position.
            // if (action.frame_id.empty()) {
            setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
            // }

            // Move object to goal position.
            if (doesActionMoveAnItem(action)) {
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
            APC_ASSERT(T.isValid(),
                       "Failed to smooth trajectory for %s", action.action_name.c_str());
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
            appendToTrajectory(trajectory, plan.actions[i].joint_trajectory);

        // Copy current plan over to robot trajectory.
        display_trajectory->setRobotTrajectoryMsg(planning_display_->getPlanningSceneRO()->getCurrentState(),
                                                  // FIXME start_state,
                                                  trajectory);

        // Swap the plan trajectory into our planning display.
        planning_display_->setTrajectoryToDisplay(display_trajectory);

        // Display trail. FIXME This doesn't accomplish anything actually.
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
            if (stored_action.action_name != "vvvvv")
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

    void MotionPlanningFrame::computePlan(apc_msgs::PrimitivePlan& plan,
                                          const robot_state::RobotState start_state,
                                          const KeyPoseMap& world_state,
                                          int client_index)
    {
        computeNearestFrameAndObjectKeys(start_state, world_state, plan);
        computeActionJointTrajectoryPoints(start_state, world_state, plan);
        computeFullyConnectedPlan(start_state, plan);
        computeDenseMotionPlan(start_state, world_state, plan, client_index);
    }

    void MotionPlanningFrame::computePlanButtonClicked()
    {
        // Reset last computed plan.
        primitive_plan_.reset(new apc_msgs::PrimitivePlan);

        // Aggregate the active actions into a plan.
        apc_msgs::PrimitivePlan plan = getPrimitivePlanFromActiveActions();

        // Clear all keys in the plan.
        for (int i = 0; i < plan.actions.size(); i++) {
            plan.actions[i].frame_key = "";
            plan.actions[i].object_key = "";
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
            computePlan(plan, start_state, world_state);
            computeSmoothedPath(plan);
            *primitive_plan_ = plan;
            // planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::loadPrimitivePlanToActiveActions, this, plan),
            //                                     "Load plan to active actions");
        } catch (std::logic_error& error) {
            ROS_ERROR("Caught exception %s", error.what());
            return;
        }

        // Copy trajectory over to display.
        {
            moveit_msgs::RobotState start_state_msg;
            robot_state::robotStateToRobotStateMsg(start_state, start_state_msg);
            loadPlanToPreview(start_state_msg, plan);
        }
    }

    void MotionPlanningFrame::computeExecuteButtonClicked()
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
        if (!primitive_plan_)
        {
            ROS_ERROR("No plan to execute");
            return;
        }
        goal.plan = *primitive_plan_;

        // Send the goal.
        execute_client_->sendGoal(goal,
                                  boost::bind(&MotionPlanningFrame::executeDoneCallback, this, _1, _2),
                                  boost::bind(&MotionPlanningFrame::executeActiveCallback, this),
                                  boost::bind(&MotionPlanningFrame::executeFeedbackCallback, this, _1));
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

}
