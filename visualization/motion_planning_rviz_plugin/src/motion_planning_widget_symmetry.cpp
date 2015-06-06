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
#include <apc_msgs/ItemSymmetry.h>
#include <moveit/warehouse/item_symmetry_storage.h>


namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::reloadItemSymmetryCache()
    {
        // Names of the stored symmetrys.
        std::vector<std::string> symmetry_names;

        // Get all the stored symmetrys.
        try
        {
            item_symmetry_storage_->getKnownItemSymmetrys(symmetry_names);
        }
        catch (std::runtime_error &ex)
        {
            ROS_ERROR("Failed to reload symmetries from database");
            return;
        }

        // Clear the current symmetry cache.
        cached_item_symmetries_.clear();

        // Add symmetrys to the tree!
        for (int i = 0; i < symmetry_names.size(); i++)
        {
            moveit_warehouse::ItemSymmetryWithMetadata symmetry;
            bool got_symmetry = false;
            try
            {
                got_symmetry = item_symmetry_storage_->getItemSymmetry(symmetry, symmetry_names[i]);
            }
            catch(std::runtime_error &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
            if (!got_symmetry)
                continue;
            cached_item_symmetries_[symmetry->item_id].push_back(*symmetry);
        }
    }

    void MotionPlanningFrame::getItemSymmetriesCached(const std::string& item_id,
                                                      EigenSTL::vector_Affine3d& symmetries)
    {
        symmetries.clear();
        if (cached_item_symmetries_.count(item_id) > 0) {
            const std::vector<apc_msgs::ItemSymmetry>& S = cached_item_symmetries_[item_id];
            symmetries.resize(S.size());
            for (int i = 0; i < S.size(); i++) {
                tf::poseMsgToEigen(S[i].pose_symmetry_origin, symmetries[i]);
            }
        } else {
            ROS_DEBUG("Failed to find item id %s in symmetry database", item_id.c_str());
            symmetries.push_back(Eigen::Affine3d::Identity());

            // FIXME Don't reload as this function needs to be multi-thread safe.

            // reloadItemSymmetryCache();
            // if (cached_item_symmetries_.count(item_id) > 0) {
            //     getItemSymmetriesCached(item_id, symmetries);
            // }
            // else {
            //     ROS_DEBUG("Failed to find item id %s in symmetry database", item_id.c_str());
            //     symmetries.push_back(Eigen::Affine3d::Identity());
            // }
        }
    }
}
