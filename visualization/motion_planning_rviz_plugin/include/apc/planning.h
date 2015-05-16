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
}
