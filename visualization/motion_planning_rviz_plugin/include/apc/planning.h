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
#pragma once
#include <apc_msgs/PrimitiveAction.h>
#include <apc_msgs/PrimitivePlan.h>
#include <moveit/robot_state/robot_state.h>
#include <QTreeWidget>

namespace moveit_warehouse
{
class PrimitivePlanStorage;
}

namespace apc_planning
{

    typedef std::map<std::string, Eigen::Affine3d, std::less<std::string>,
                     Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > >
    KeyPoseMap;

    bool _is_robot_moving_(const apc_msgs::PrimitiveAction& action);
    bool _is_object_moving_(const apc_msgs::PrimitiveAction& action);
    bool _is_object_moving_(const apc_msgs::PrimitiveAction& action);
    std::string _action_type_(const apc_msgs::PrimitiveAction& action);
    bool is_action_transit(const apc_msgs::PrimitiveAction& action);
    bool is_action_pregrasp(const apc_msgs::PrimitiveAction& action);
    bool is_action_grasp(const apc_msgs::PrimitiveAction& action);
    bool is_action_postgrasp(const apc_msgs::PrimitiveAction& action);
    bool is_action_nonprehensile(const apc_msgs::PrimitiveAction& action);
    bool is_action_stationary(const apc_msgs::PrimitiveAction& action);

    void copyJointTrajectoryRestrictedToGroup(apc_msgs::PrimitiveAction& target,
                                              const apc_msgs::PrimitiveAction& source,
                                              const robot_state::RobotState& robot_state);

    void partitionPlanBySubgroups(apc_msgs::PrimitivePlan& plan,
                                  const robot_state::RobotState& robot_state);

    void preprocessPlanBeforeExecution(apc_msgs::PrimitivePlan& plan,
                                       const robot_state::RobotState& robot_state);

    void resetPlanJointTrajectories(apc_msgs::PrimitivePlan& plan);

    void formatUniqueIndex(std::string& format, const std::vector<std::string>& existing);

    std::vector<std::string> getExistingPlanNames(QTreeWidget* plan_tree,
                                                  boost::shared_ptr<moveit_warehouse::PrimitivePlanStorage> primitive_plan_storage_);

    void validatePlanningArguements(const apc_msgs::PrimitivePlan& plan);

    void clampJointLimitsInPlan(apc_msgs::PrimitivePlan& plan,
                                const robot_state::RobotState& robot_state);
apc_msgs::PrimitiveAction getSubgroupAction(const std::string& subgroup_expr,
                                                          const apc_msgs::PrimitiveAction& action,
                                            const robot_state::RobotState& robot_state);

    std::string toStringNoArr(const apc_msgs::PrimitiveAction& action);
    std::string toStringNoArr(int index, const apc_msgs::PrimitivePlan& plan);
    void assertPlanningPreconditions(const std::vector<apc_msgs::PrimitivePlan>& plans,
                                     const robot_state::RobotState& start_state,
                                     const KeyPoseMap& world_state);
    void assertPlanningPreconditions(const apc_msgs::PrimitivePlan& plan,
                                     const robot_state::RobotState& start_state,
                                     const KeyPoseMap& world_start);
    void setRobotStateToPoint(robot_state::RobotState& robot_state,
                              const std::vector<std::string> joint_names,
                              const trajectory_msgs::JointTrajectoryPoint& point);

    std::string toStringJointDiff(const robot_state::RobotState& prev_state,
                                  const robot_state::RobotState& next_state);
    std::string toStringNoArr(const apc_msgs::PrimitivePlan& plan);
    void assertGraspPreconditions(const std::vector<apc_msgs::PrimitivePlan>& grasps);
    void assertGraspPreconditions(const apc_msgs::PrimitivePlan& grasp);
    void fixGrasp(std::vector<apc_msgs::PrimitivePlan>& grasps,
                  const robot_state::RobotState& robot_state,
                  const KeyPoseMap& world_state);
    void fixGrasp(apc_msgs::PrimitivePlan& grasp,
                  const robot_state::RobotState& robot,
                  const KeyPoseMap& world);
    void convertPlanListToPlanActions(const std::vector<apc_msgs::PrimitivePlan>& input,
                                      apc_msgs::PrimitivePlan& output);
}
