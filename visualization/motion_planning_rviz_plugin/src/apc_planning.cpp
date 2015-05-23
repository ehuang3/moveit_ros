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
#include <apc/planning.h>
#include <apc/exception.h>
#include <apc/eigen.h>
#include <ros/ros.h>
#include <sstream>
#include <boost/xpressive/xpressive.hpp>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <moveit/warehouse/primitive_plan_storage.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>


bool apc_planning::_is_robot_moving_(const apc_msgs::PrimitiveAction& action)
{
    const double TOLERANCE = 1e-7;
    const trajectory_msgs::JointTrajectoryPoint& start = action.joint_trajectory.points.front();
    for (int i = 0; i < action.joint_trajectory.points.size(); i++) {
        const trajectory_msgs::JointTrajectoryPoint& current = action.joint_trajectory.points[i];
        for (int j = 0; j < current.positions.size(); j++)
            if (std::abs(start.positions[j] - current.positions[j]) > TOLERANCE) {
                // ROS_DEBUG("Joint %s greater than tolerance", action.joint_trajectory.joint_names[j].c_str());
                return true;
            }
    }
    return false;
}

bool apc_planning::_is_object_moving_(const apc_msgs::PrimitiveAction& action)
{
    if (action.object_id.empty()) {
        return false;
    }
    if (action.object_trajectory.poses.size() == 0) {
        return false;
    }
    const double TOLERANCE = 1e-7;
    for (int i = 0; i < action.object_trajectory.poses.size(); i++) {
        const geometry_msgs::Pose& start = action.object_trajectory.poses.front();
        const geometry_msgs::Pose& current = action.object_trajectory.poses[i];
        Eigen::Affine3d T_start;
        Eigen::Affine3d T_current;
        tf::poseMsgToEigen(start, T_start);
        tf::poseMsgToEigen(current, T_current);
        if ((T_start.translation() - T_current.translation()).norm() > TOLERANCE)
            return true;
        Eigen::Quaterniond q_start(T_start.rotation());
        Eigen::Quaterniond q_current(T_current.rotation());
        double angle = q_start.angularDistance(q_current);
        if (std::abs(angle) > TOLERANCE)
            return true;
    }
    return false;
}

std::string apc_planning::_action_type_(const apc_msgs::PrimitiveAction& action)
{
    bool robot_moving  = _is_robot_moving_(action);
    bool object_moving = _is_object_moving_(action);
    bool grasping = action.grasp;
    bool has_object = !action.object_id.empty();
    // Compute action types.
    bool transit       =                                                   !has_object;
    bool pregrasp      = (( robot_moving && !grasping && !object_moving &&  has_object) ||
                          (!robot_moving && !grasping && !object_moving &&  has_object));
    bool grasp         =   !robot_moving &&  grasping && !object_moving &&  has_object;
    bool postgrasp     =    robot_moving &&  grasping                   &&  has_object;
    bool nonprehensile =    robot_moving && !grasping &&  object_moving &&  has_object;
    APC_ASSERT(transit ^ pregrasp ^ grasp ^ postgrasp ^ nonprehensile,
               "Requested impossible action type\n"
               "   transit: %d\n"
               "  pregrasp: %d\n"
               "     grasp: %d\n"
               " postgrasp: %d\n"
               "prehensile: %d\n"
               "\n"
               "   robot_moving: %d\n"
               "  object_moving: %d\n"
               "       grasping: %d\n"
               "     has_object: %d\n",
               transit, pregrasp, grasp, postgrasp, nonprehensile,
               robot_moving, object_moving, grasping, has_object);
    if (transit)
        return "transit";
    if (pregrasp)
        return "pregrasp";
    if (grasp)
        return "grasp";
    if (postgrasp)
        return "postgrasp";
    if (nonprehensile)
        return "nonprehensile";
}

bool apc_planning::is_action_transit(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "transit";
}

bool apc_planning::is_action_pregrasp(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "pregrasp";
}

bool apc_planning::is_action_grasp(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "grasp";
}

bool apc_planning::is_action_postgrasp(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "postgrasp";
}

bool apc_planning::is_action_nonprehensile(const apc_msgs::PrimitiveAction& action)
{
    return _action_type_(action) == "nonprehensile";
}

bool apc_planning::is_action_stationary(const apc_msgs::PrimitiveAction& action)
{
    return !_is_robot_moving_(action);
}

void apc_planning::copyJointTrajectoryRestrictedToGroup(apc_msgs::PrimitiveAction& target,
                                                        const apc_msgs::PrimitiveAction& source,
                                                        const robot_state::RobotState& robot_state)
{
    APC_ASSERT(target.group_id != source.group_id,
               "Why are you copying the same group from source to target?");
    typedef std::vector<std::string> JointNames;
    JointNames target_names = robot_state.getJointModelGroup(target.group_id)->getVariableNames();
    JointNames source_names = source.joint_trajectory.joint_names;
    trajectory_msgs::JointTrajectory target_traj = target.joint_trajectory;
    trajectory_msgs::JointTrajectory source_traj = source.joint_trajectory;
    target_traj.joint_names.clear();
    target_traj.points.clear();
    target_traj.points.resize(source_traj.points.size());
    for (int i = 0; i < source_names.size(); i++) {
        if (std::find(target_names.begin(), target_names.end(), source_names[i]) != target_names.end()) {
            target_traj.joint_names.push_back(source_names[i]);
            for (int j = 0; j < source_traj.points.size(); j++) {
                target_traj.points[j].positions.push_back(source_traj.points[j].positions[i]);
            }
        }
    }
    APC_ASSERT(target_names.size() == target_traj.joint_names.size(),
               "Failed to find some variable name in source %s", source.action_name.c_str());
    target.joint_trajectory = target_traj;
}


apc_msgs::PrimitiveAction apc_planning::getSubgroupAction(const std::string& subgroup_expr,
                                                          const apc_msgs::PrimitiveAction& action,
                                                          const robot_state::RobotState& robot_state)
{
    typedef apc_msgs::PrimitivePlan Plan;
    typedef apc_msgs::PrimitiveAction Action;
    std::vector<const moveit::core::JointModelGroup*> subgroups;
    robot_state.getJointModelGroup(action.group_id)->getSubgroups(subgroups);
    subgroups.push_back(robot_state.getJointModelGroup(action.group_id));
    for(int i =0; i < subgroups.size();i ++){
        using namespace boost::xpressive;
        sregex rex = sregex::compile(subgroup_expr);
        smatch what;
        std::string subgroup_id = subgroups[i]->getName();
        // ROS_INFO_STREAM("subgroup_id " << subgroup_id);
        if (regex_match(subgroup_id, what, rex)) {
            Action subaction = action;
            subaction.group_id = subgroup_id;
            if (subaction.group_id == action.group_id)
                return subaction;
            else
                copyJointTrajectoryRestrictedToGroup(subaction, action, robot_state);
            return subaction;
        }
    }
    APC_ASSERT(false, "Failed to find a matching %s subgroup action for group %s",
               subgroup_expr.c_str(), action.group_id.c_str());
}

std::string apc_planning::toStringNoArr(int index, const apc_msgs::PrimitivePlan& plan)
{
    std::stringstream ss;
    ss << "   plan name: " << plan.plan_name << std::endl;
    for (int i = 0; i < plan.actions.size(); i++) {
    ss << "   action[" << i << "]:" << plan.actions[i].action_name << std::endl;
    }
    ss << "action index: " << index << std::endl;
    ss << toStringNoArr(plan.actions[index]);
    return ss.str();
}

std::string apc_planning::toStringNoArr(const apc_msgs::PrimitivePlan& plan)
{
    std::stringstream ss;
    ss << "   plan name: " << plan.plan_name << std::endl;
    for (int i = 0; i < plan.actions.size(); i++) {
        ss << "   action[" << i << "]:" << std::endl;
        ss << toStringNoArr(plan.actions[i]);
    }
    return ss.str();
}

std::string apc_planning::toStringNoArr(const apc_msgs::PrimitiveAction& action)
{
    std::stringstream ss;
    ss << " action name: " << action.action_name << std::endl
       << "    group ID: " << action.group_id << std::endl
       << "    frame ID: " << action.frame_id << std::endl
       << "   frame key: " << action.frame_key << std::endl
       << " frame poses: " << action.eef_trajectory.poses.size() << std::endl
       << "  frame link: " << action.eef_link_id << std::endl
       << "   object ID: " << action.object_id << std::endl
       << "  object key: " << action.object_key << std::endl
       << "object poses: " << action.object_trajectory.poses.size() << std::endl
       << " object link: " << action.attached_link_id << std::endl
       << "     joint T: " << action.joint_trajectory.points.size() << std::endl;
    Eigen::MatrixXd T;
    T.resize(action.joint_trajectory.joint_names.size(), action.joint_trajectory.points.size());
    for (int c = 0; c < T.cols(); c++) {
        for (int r = 0; r < T.rows(); r++) {
            T(r,c) = action.joint_trajectory.points[c].positions[r];
        }
    }
    ss << "       grasp: " << (action.grasp ? "True" : "False") << std::endl
       << "      locked: " << (action.eef_locked ? "True" : "False") << std::endl
       << "    duration: " << action.duration << std::endl
       << "   cartesian: " << (action.interpolate_cartesian ? "True" : "False") << std::endl
       << " action type: " << _action_type_(action) << std::endl
       << "           T:\n" << T << std::endl;
    return ss.str();
}

void apc_planning::partitionPlanBySubgroups(apc_msgs::PrimitivePlan& _plan,
                                            const robot_state::RobotState& robot_state)
{
    const apc_msgs::PrimitivePlan input_plan = _plan;
    apc_msgs::PrimitivePlan output_plan;
    // Partition...
    for (int i = 0; i < input_plan.actions.size(); i++) {
        bool partition = false;
        kinematics::KinematicsBaseConstPtr solver =
            robot_state.getJointModelGroup(input_plan.actions[i].group_id)->getSolverInstance();
        if (solver) {
            partition = false;
        } else {
            // Check for subgroup disparity in kinematics sovlers.
            std::vector<const moveit::core::JointModelGroup*> subgroups;
            robot_state.getJointModelGroup(input_plan.actions[i].group_id)->getSubgroups(subgroups);
            int d = 0;
            for (int j = 0; j < subgroups.size(); j++) {
                solver = subgroups[j]->getSolverInstance();
                int s = solver ? 1 : -1;
                if (d == 0)
                    d = solver ? 1 : -1;
                else if (s != d) {
                    partition = true;
                    break;
                }
            }
        }

        try {
            if (partition) {
                typedef apc_msgs::PrimitiveAction Action;
                // First plan for the groups with a solver (i.e. the arm).
                Action arm_action = getSubgroupAction(".*arm", input_plan.actions[i], robot_state);
                // Next plan for groups without a solver (i.e. the hand).
                Action hand_action = getSubgroupAction(".*hand", input_plan.actions[i], robot_state);
                output_plan.actions.push_back(arm_action);
                output_plan.actions.push_back(hand_action);
            } else {
                output_plan.actions.push_back(input_plan.actions[i]);
            }
        } catch (apc_exception::Exception& error) {
            output_plan.actions.push_back(input_plan.actions[i]);
        }
    }
    output_plan.plan_name = input_plan.plan_name;
    APC_ASSERT(output_plan.actions.size() >= input_plan.actions.size(),
               "Failed to at least match input plan");
    _plan = output_plan;
}

void apc_planning::preprocessPlanBeforeExecution(apc_msgs::PrimitivePlan& plan,
                                                 const robot_state::RobotState& robot_state)
{
    // Ensure that only joints which correspond to motors are sent down to execute.
    for (int i = 0; i < plan.actions.size(); i++) {
        apc_msgs::PrimitiveAction& action = plan.actions[i];
        const std::vector<std::string>& A = robot_state.getJointModelGroup(action.group_id)->getActiveJointModelNames();
        const std::vector<std::string>& O = action.joint_trajectory.joint_names;
        trajectory_msgs::JointTrajectory        N;
        const trajectory_msgs::JointTrajectory& M = action.joint_trajectory;
        N.points.resize(M.points.size());
        for (int j = 0; j < O.size(); j++)
            if (std::find(A.begin(), A.end(), O[j]) != A.end()) {
                N.joint_names.push_back(O[j]);
                for (int k = 0; k < M.points.size(); k++)
                    N.points[k].positions.push_back(M.points[k].positions[j]);
            }
        action.joint_trajectory = N;
    }
}

void apc_planning::resetPlanJointTrajectories(apc_msgs::PrimitivePlan& plan)
{
    for (int i = 0; i < plan.actions.size(); i++) {
        trajectory_msgs::JointTrajectoryPoint back = plan.actions[i].joint_trajectory.points.back();
        plan.actions[i].joint_trajectory.points.resize(2);
        plan.actions[i].joint_trajectory.points[1] = back;
    }
}

void apc_planning::formatUniqueIndex(std::string& format, const std::vector<std::string>& existing)
{
    using namespace boost::xpressive;
    std::string expr = format;
    boost::replace_all(expr, "%i", "([0-9]+)");
    sregex rex = sregex::compile(expr);
    smatch what;
    int max = 0;
    for (int i = 0; i < existing.size(); i++) {
        if (regex_match(existing[i], what, rex)) {
            APC_ASSERT(what.size() != 1,
                       "Existing plan with name %s", what[0].str().c_str());
            int n = std::atoi(what[1].str().c_str());
            if (n >= max) {
                max = n + 1;
            }
        }
    }
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(3) << max;
    boost::replace_all(format, "%i", ss.str());
}

std::vector<std::string> apc_planning::getExistingPlanNames(QTreeWidget* plan_tree,
                                                            boost::shared_ptr<moveit_warehouse::PrimitivePlanStorage> primitive_plan_storage_)
{
    // Get all plan names from storage. FIXME We don't need to use this.
    // std::vector<std::string> stored_names;
    // primitive_plan_storage_->getKnownPrimitivePlans(plan_names);

    // Get all plan names from the current list.
    std::vector<std::string> active_names;
    for (int i = 0; i < plan_tree->topLevelItemCount(); i++) {
        active_names.push_back(plan_tree->topLevelItem(i)->text(0).toStdString());
    }
    return active_names;
}

void apc_planning::validatePlanningArguements(const apc_msgs::PrimitivePlan& plan)
{
    for (int i = 0; i < plan.actions.size(); i++) {
        const apc_msgs::PrimitiveAction& action = plan.actions[i];
        APC_ASSERT((!action.object_id.empty() && !action.object_key.empty()) ||
                   ( action.object_id.empty() &&  action.object_key.empty()),
                   "Failed ID-KEY check on action %s in plan %s\n"
                   "     object_id: %s\n"
                   "    object_key: %s",
                   action.action_name.c_str(), plan.plan_name.c_str(),
                   action.object_id.c_str(), action.object_key.c_str());
    }
}

void apc_planning::clampJointLimitsInPlan(apc_msgs::PrimitivePlan& plan,
                                          const robot_state::RobotState& robot_state)
{
    for (int i = 0; i < plan.actions.size(); i++) {
        apc_msgs::PrimitiveAction& action = plan.actions[i];
        // APC_ASSERT(robot_state.getJointModelGroup(action.group_id)->getSubgroupNames().size() == 0,
        //            "Failed to assert that no subgroups exist for action %s in plan %s",
        //            action.action_name.c_str(), plan.plan_name.c_str());
        APC_ASSERT(action.joint_trajectory.points.size() > 0,
                   "Failed to find joint trajectory for action %s in plan %s",
                   action.action_name.c_str(), plan.plan_name.c_str());
        const std::vector<std::string>& joint_names = action.joint_trajectory.joint_names;
        for (int j = 0; j < joint_names.size(); j++) {
            double max_pos = robot_state.getRobotModel()->getVariableBounds(action.joint_trajectory.joint_names[j]).max_position_;
            double min_pos = robot_state.getRobotModel()->getVariableBounds(action.joint_trajectory.joint_names[j]).min_position_;
            trajectory_msgs::JointTrajectory& T = action.joint_trajectory;
            for (int k = 0; k < T.points.size(); k++) {
                double& q = T.points[k].positions[j];
                if (q < min_pos || q > max_pos) {
                    ROS_DEBUG_STREAM("Clamping " << joint_names[j]);
                    q = std::min(std::max(q, min_pos), max_pos);
                }
            }
        }
    }
}

void apc_planning::assertPlanningPreconditions(const std::vector<apc_msgs::PrimitivePlan>& plans,
                                               const robot_state::RobotState& start_state,
                                               const KeyPoseMap& world_state)
{
    for (int i = 0; i < plans.size(); i++) {
        assertPlanningPreconditions(plans[i], start_state, world_state);
    }
}

void apc_planning::setRobotStateToPoint(robot_state::RobotState& robot_state,
                                        const std::vector<std::string> joint_names,
                                        const trajectory_msgs::JointTrajectoryPoint& point)
{
    APC_ASSERT(joint_names.size() == point.positions.size(),
               "Mismatch between joint names and positions");
    for (int i = 0; i < joint_names.size(); i++)
        robot_state.setJointPositions(joint_names[i], &point.positions[i]);
    robot_state.update();
}

std::string apc_planning::toStringJointDiff(const robot_state::RobotState& prev_state,
                                            const robot_state::RobotState& next_state)
{
    std::stringstream ss;
    std::vector<std::string> variable_names = prev_state.getVariableNames();
    for (int i = 0; i < variable_names.size(); i++) {
        double p = prev_state.getVariablePosition(variable_names[i]);
        double n = next_state.getVariablePosition(variable_names[i]);
        if (std::abs(p - n) > 1e-7) {
            ss << variable_names[i] << ": " << p << " - " << n << std::endl;
        }
    }
    return ss.str();
}

void apc_planning::assertPlanningPreconditions(const apc_msgs::PrimitivePlan& plan,
                                               const robot_state::RobotState& start_state,
                                               const KeyPoseMap& world_start)
{
    typedef apc_msgs::PrimitiveAction Action;
    typedef apc_msgs::PrimitivePlan Plan;
    // assert that start state matches the starting point of the first action.
    APC_ASSERT_PLAN(plan.actions.size() > 0, plan,
                    "Missing actions for plan %s", plan.plan_name.c_str());
    const Action& first_action = plan.actions.front();
    robot_state::RobotState robot_state = start_state;
    setRobotStateToPoint(robot_state,
                         first_action.joint_trajectory.joint_names,
                         first_action.joint_trajectory.points.front());
    APC_ASSERT_PLAN(robot_state.distance(start_state) < 1e-7, plan,
               "Start state does not match starting action\n%s",
               toStringNoArr(0, plan).c_str());
    // for actions...
    robot_state::RobotState prev_state = start_state;
    robot_state::RobotState next_state = start_state;
    KeyPoseMap world_state = world_start;
    bool grasped = false;
    for (int i = 0; i < plan.actions.size(); i++) {
        const Action& action = plan.actions[i];
        APC_ASSERT_PLAN(!(action.object_id.empty() ^ action.object_key.empty()), plan,
                   "Mismatching object ids and object keys\n%s",
                   toStringNoArr(i, plan).c_str());
        APC_ASSERT_PLAN(!(action.frame_id.empty() ^ action.frame_key.empty()), plan,
                   "Mismatching frame ids and frame keys\n%s",
                   toStringNoArr(i, plan).c_str());
        // fully connected
        setRobotStateToPoint(next_state,
                             action.joint_trajectory.joint_names,
                             action.joint_trajectory.points.front());
        APC_ASSERT_PLAN(prev_state.distance(next_state) < 1e-7, plan,
                   "Disconnect between previous and next states in plan\n%s\n%s",
                   toStringNoArr(i, plan).c_str(),
                   toStringJointDiff(prev_state, next_state).c_str());
        // that only the joint names associated with the joint group are in the
        // trajectory.
        {
            const moveit::core::JointModelGroup* jmg = next_state.getJointModelGroup(action.group_id);
            std::vector<std::string> variable_names = jmg->getVariableNames();
            for (int i = 0; i < action.joint_trajectory.joint_names.size(); i++) {
                std::string jn = action.joint_trajectory.joint_names[i];
                APC_ASSERT_PLAN(std::find(variable_names.begin(), variable_names.end(), jn) != variable_names.end(),
                                plan, "Extraneous joint name %s found\n%s",
                                jn.c_str(), toStringNoArr(i, plan).c_str());
            }
        }
        // only if we go from pregrasp to grasp, allow the object location to jump!
        if (!grasped && action.grasp) {
            grasped = true;
            Eigen::Affine3d T_object_world = world_state[action.object_key];
            Eigen::Affine3d T_link_world = next_state.getGlobalLinkTransform(action.attached_link_id);
            Eigen::Affine3d T_object_link;
            tf::poseMsgToEigen(action.object_trajectory.poses.front(), T_object_link);
            world_state[action.object_key] = T_link_world * T_object_link;
        }
        // that once we grasp, we don't let go.
        APC_ASSERT_PLAN(!grasped || (grasped && action.grasp), plan,
                   "Letting go of a grasp mid trajectory\n%s",
                   toStringNoArr(i, plan).c_str());
        // that obj matches offset locations at start of trajectory.
        if (action.object_trajectory.poses.size() > 0 && action.grasp) {
            APC_ASSERT_PLAN(!action.object_key.empty(), plan,
                       "Failed to get object key for action\n%s",
                       toStringNoArr(i, plan).c_str());
            const std::string link_id = action.attached_link_id;
            Eigen::Affine3d T_link_world_1 = next_state.getGlobalLinkTransform(link_id);
            Eigen::Affine3d T_frame_world = world_state[action.object_key];
            Eigen::Affine3d T_frame_link;
            tf::poseMsgToEigen(action.object_trajectory.poses.front(), T_frame_link);
            Eigen::Affine3d T_link_world_2 = T_frame_world * T_frame_link.inverse();
            APC_ASSERT_PLAN(apc_eigen::elementWiseMatrixNorm(T_link_world_1, T_link_world_2) < 1e-2, plan,
                            "Object world pose does not match intended\n%s",
                            toStringNoArr(i, plan).c_str());
        }
        // that eef matches frame location at start of trajectory.
        if (action.eef_trajectory.poses.size() > 0) {
            // do not check if this group is not an arm group
            using namespace boost::xpressive;
            sregex rex = sregex::compile(".*arm.*");
            smatch what;
            if (!regex_match(action.group_id, what, rex)) {
                // do nothing
            } else {
                APC_ASSERT_PLAN(!action.frame_key.empty(), plan,
                                "Failed to get frame key for action\n%s",
                                toStringNoArr(i, plan).c_str());
                const std::string link_id = action.eef_link_id;
                Eigen::Affine3d T_link_world_1 = next_state.getGlobalLinkTransform(link_id);
                Eigen::Affine3d T_frame_world = world_state[action.frame_key];
                Eigen::Affine3d T_frame_link;
                tf::poseMsgToEigen(action.eef_trajectory.poses.front(), T_frame_link);
                Eigen::Affine3d T_link_world_2 = T_frame_world * T_frame_link.inverse();
                APC_ASSERT_PLAN(apc_eigen::elementWiseMatrixNorm(T_link_world_1, T_link_world_2) < 1e-2, plan,
                           "Frame world pose does not match intended\n%s",
                           toStringNoArr(i, plan).c_str());
            }
        }
        // TODO that hands include the mimic knuckle joint.
        // TODO no torso group, only hand and arm
        {
            using namespace boost::xpressive;
            sregex rex = sregex::compile(".*torso");
            smatch what;
            if (regex_match(action.group_id, what, rex))
                ROS_WARN("Detected arm_torso group before planning\n%s",
                         toStringNoArr(i, plan).c_str());
        }
        // update to end points
        setRobotStateToPoint(prev_state,
                             action.joint_trajectory.joint_names,
                             action.joint_trajectory.points.back());
        setRobotStateToPoint(next_state,
                             action.joint_trajectory.joint_names,
                             action.joint_trajectory.points.back());
        // update objects to end points
        if (is_action_grasp(action) || is_action_postgrasp(action)) {
            Eigen::Affine3d T_object_world = world_state[action.object_key];
            Eigen::Affine3d T_link_world = next_state.getGlobalLinkTransform(action.attached_link_id);
            Eigen::Affine3d T_object_link;
            tf::poseMsgToEigen(action.object_trajectory.poses.back(), T_object_link);
            world_state[action.object_key] = T_link_world * T_object_link;
        }
    }
}
