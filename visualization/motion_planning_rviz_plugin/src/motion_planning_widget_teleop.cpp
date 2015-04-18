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
#include <apc_msgs/GetMotionPlan.h>
#include <apc_msgs/ComputeDenseMotion.h>
#include <boost/xpressive/xpressive.hpp>


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
        APC_ASSERT(_kiva_pod, "Failed to get KIVA pod model");
        for (char c = 'A'; c <= 'L'; c++) {
            std::string bin = std::string("bin_") + c;
            map[bin] = map["kiva_pod"] * _kiva_pod->getGlobalTransform(bin);
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

    void MotionPlanningFrame::setStateFromPoint(robot_state::RobotState& robot,
                                                const std::vector<std::string>& joint_names,
                                                const trajectory_msgs::JointTrajectoryPoint& point)
    {
        APC_ASSERT(joint_names.size() == point.positions.size(),
                   "Mismatch between joint names and positions");
        for (int i = 0; i < joint_names.size(); i++)
            robot.setJointPositions(joint_names[i], &point.positions[i]);
        robot.update();
    }

    void MotionPlanningFrame::setStateFromIK(robot_state::RobotState& robot,
                                             const std::string& link_id,
                                             const std::string& group_id,
                                             const Eigen::Affine3d& T_frame_world,
                                             const geometry_msgs::Pose& pose_link_frame)
    {
        APC_ASSERT(link_id.size() > 0, "Failed to provide input link");

        // Back out the desired link transform in global coordinates.
        Eigen::Affine3d T_frame_link;
        tf::poseMsgToEigen(pose_link_frame, T_frame_link);
        Eigen::Affine3d T_link_world = T_frame_world * T_frame_link.inverse();

        // Snap to the frame using IK.
        const moveit::core::JointModelGroup* jmg = robot.getJointModelGroup(group_id);
        geometry_msgs::Pose pose_link_world;
        tf::poseEigenToMsg(T_link_world, pose_link_world);
        APC_ASSERT(robot.setFromIK(jmg, pose_link_world, link_id),
                   "Failed to set %s to pose using %s IK", link_id.c_str(), group_id.c_str());
        robot.update();
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

        // Copy starting conditions.
        robot_state::RobotState robot_state = start;
        KeyPoseMap world_state = world;

        // For each action..
        for (int i = 0; i < plan.actions.size(); i++) {
            apc_msgs::PrimitiveAction& action = plan.actions[i];

            if (action.frame_id.empty()) {
                setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.front());
            } else {
                action.frame_key = computeNearestFrameKey(action.frame_id, action.eef_link_id, robot_state, world_state);
                setStateFromIK(robot_state, action.eef_link_id, action.group_id, world_state[action.frame_key],
                               action.eef_trajectory.poses.front());
            }

            if (!action.object_id.empty()) {
                action.object_key = computeNearestObjectKey(action.object_id, action.attached_link_id, robot_state, world_state);
                // if (action.frame_key != action.object_key)
                // TODO Assert
            }

            // Move robot state to goal position.
            if (action.frame_id.empty()) {
                setStateFromPoint(robot_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
            } else {
                setStateFromIK(robot_state, action.eef_link_id, action.group_id, world_state[action.frame_key],
                               action.eef_trajectory.poses.back());
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

    void MotionPlanningFrame::appendNamedJointsToActionJointTrajectory(const robot_state::RobotState& state,
                                                                       apc_msgs::PrimitiveAction& action)
    {
        APC_ASSERT(action.joint_trajectory.joint_names.size() > 0,
                   "Missing joint names in action %s joint trajectory", action.action_name.c_str());
        trajectory_msgs::JointTrajectoryPoint point;
        const std::vector<std::string>& joint_names = action.joint_trajectory.joint_names;
        for (int i = 0; joint_names.size(); i++) {
            int index = state.getJointModel(joint_names[i])->getFirstVariableIndex();
            double angle = state.getVariablePosition(index);
            point.positions.push_back(angle);
            point.velocities.push_back(0);
            point.accelerations.push_back(0);
            point.effort.push_back(0);
        }
        action.joint_trajectory.points.push_back(point);
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
                APC_ASSERT(action.eef_trajectory.poses.size() > 0,
                           "Missing joint model group for %s", action.action_name.c_str());
                // Clear existing joint trajectory points.
                action.joint_trajectory.points.clear();
                // Rebuild joint trajectory.
                for (int j = 0; j < action.eef_trajectory.poses.size(); j++) {
                    setStateFromIK(robot_state, action.eef_link_id, action.group_id, world_state[action.frame_key],
                                   action.eef_trajectory.poses[j]);
                    appendNamedJointsToActionJointTrajectory(robot_state, action);
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

            // Get ID.
            using namespace boost::xpressive;
            sregex rex = sregex::compile("^([:alpha:_]+)(_[:digit:]+)??");
            smatch what;
            APC_ASSERT(regex_match(iter->first, what, rex),
                       "Failed to extract ID of %s", iter->first.c_str());
            std::string id = what[1];

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

    KeyPoseMap MotionPlanningFrame::computeDenseMotionPlan(const robot_state::RobotState& start,
                                                           const KeyPoseMap& world,
                                                           apc_msgs::PrimitivePlan& plan)
    {
        // Copy starting conditions.
        robot_state::RobotState robot_state = start;
        KeyPoseMap world_state = world;

        // Create motion planning service message.
        apc_msgs::ComputeDenseMotion srv;
        setWorldKeyPoseToWorldStateMessage(world_state, srv.request.world_state);

        // Insert additional actions to connect the previous robot
        // state to the next start state.
        {
            std::vector<apc_msgs::PrimitiveAction> actions;
            robot_state::RobotState prev_state = start;
            robot_state::RobotState next_state = start;
            for (int i = 0; i < plan.actions.size(); i++) {

                setStateFromPoint(next_state, plan.actions[i].joint_trajectory.joint_names,
                                  plan.actions[i].joint_trajectory.points.front());

                if (next_state.distance(prev_state) > 1e-10) {
                    ROS_DEBUG("Inserting action to connect previous robot state with next robot state");
                    apc_msgs::PrimitiveAction action;
                    action.action_name = "vvvvv";
                    action.group_id = plan.actions[i].group_id;
                    action.joint_trajectory.joint_names = plan.actions[i].joint_trajectory.joint_names;

                    appendNamedJointsToActionJointTrajectory(prev_state, action);
                    appendNamedJointsToActionJointTrajectory(next_state, action);

                    setStateFromPoint(prev_state, action.joint_trajectory.joint_names, action.joint_trajectory.points.back());
                    APC_ASSERT(next_state.distance(prev_state) <= 1e-10,
                               "Failed to connect previous robot state with next state");

                    actions.push_back(action);
                }

                setStateFromPoint(prev_state, plan.actions[i].joint_trajectory.joint_names,
                                  plan.actions[i].joint_trajectory.points.back());
                actions.push_back(plan.actions[i]);
            }
            plan.actions = actions;
        }

        // For each action..
        for (int i = 0; i < plan.actions.size(); i++) {
            apc_msgs::PrimitiveAction& action = plan.actions[i];

            // Fill out dense motion planning request.
            robot_state::robotStateToRobotStateMsg(robot_state, srv.request.robot_state);
            setWorldKeyPoseToWorldStateMessage(world_state, srv.request.world_state);
            srv.request.action = action;

            APC_ASSERT(motion_plan_client_.call(srv),
                       "Failed to call dense motion planning service");
            APC_ASSERT(srv.response.collision_free,
                       "Failed to find collision free trajectory");
            APC_ASSERT(srv.response.valid,
                       "Failed to find valid trajectory");

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

    void MotionPlanningFrame::computePlanButtonClicked()
    {
        // Reset last computed plan.
        primitive_plan_.reset(new apc_msgs::PrimitivePlan);

        // Get the list of active goals (waypoints) to follow.
        QListWidget* active_actions = ui_->active_actions_list;

        // Create an empty plan.
        apc_msgs::PrimitivePlan plan;

        // Appeach each active action to the plan.
        for (int i = 0; i < active_actions->count(); i++)
        {
            // Get the plan stored in the active action item.
            apc_msgs::PrimitivePlan stored_plan =
                getMessageFromUserData<apc_msgs::PrimitivePlan>(active_actions->item(i)->data(Qt::UserRole));

            // Append each action stored in the active action item.
            for (int j = 0; j < stored_plan.actions.size(); j++)
                plan.actions.push_back(stored_plan.actions[j]);
        }

        // Compute trajectory and save on success.
        if (computePlan(plan))
            *primitive_plan_ = plan;
        else
        {
            ROS_ERROR("Failed to compute TRAJOPT plan for active actions");
        }

        // Copy trajectory over to display.
        {
            moveit_msgs::RobotState start_state;
            robot_state::robotStateToRobotStateMsg(*planning_display_->getQueryStartState(), start_state);
            loadPlanToPreview(start_state, plan);
        }
    }



    bool MotionPlanningFrame::computePlan(apc_msgs::PrimitivePlan& plan)
    {
        // Create a motion plan service object.
        apc_msgs::GetMotionPlan srv;

        ROS_INFO("Computing motion plan using trajectory optimization");

        // Get the planning scene message.
        {
            const planning_scene_monitor::LockedPlanningSceneRO& locked_scene = planning_display_->getPlanningSceneRO();
            locked_scene->getPlanningSceneMsg(srv.request.scene);
        }

        // Set start state to the current state.
        robot_state::RobotState start_state = *planning_display_->getQueryStartState();
        {
            const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
            start_state = ps->getCurrentState();
        }
        planning_display_->setQueryStartState(start_state);

        // Create goal state.
        robot_state::RobotState goal_state = *planning_display_->getQueryGoalState();

        // Run plan through trajectory optimization.
        for (int i = 0; i < plan.actions.size(); i++)
        {
            // Construct robot goal state.
            // FIXME loadStateFromAction(goal_state, plan.actions[i]);

            // Write planning arguments into service message.
            robot_state::robotStateToRobotStateMsg(start_state, srv.request.start_state);
            robot_state::robotStateToRobotStateMsg(goal_state,  srv.request.goal_state);

            // Call motion planning service and wait.
            if(!motion_plan_client_.call(srv))
            {
                ROS_ERROR("Failed to call service GetMotionPlan");
                return false;
            }

            if (!srv.response.valid)
            {
                ROS_ERROR("Failed to compute valid TRAJOPT plan");
                return false;
            }

            // Write plan to output.
            plan.actions[i].joint_trajectory = srv.response.plan.actions[0].joint_trajectory;

            // Set next start state to this goal state.
            start_state = goal_state;
        }

        return true;
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
