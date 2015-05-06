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
#include <algorithm>
#include <apc_msgs/RunVision.h>
#include <eigen_conversions/eigen_msg.h>


namespace moveit_rviz_plugin
{

    std::vector<std::string> MotionPlanningFrame::computeObjectIdsInBinFromJson(const std::string& bin_id)
    {
        std::vector<std::string> object_ids;
        // Get the bin contents table widget.
        QTableWidget* widget = ui_->bin_contents_table_widget;
        for (int i = 0; i < widget->rowCount(); i++) {
            std::string bin = widget->item(i, 0)->text().toStdString();
            if (bin.compare(bin_id) == 0) {
                object_ids.push_back(widget->item(i,1)->text().toStdString());
            }
        }
        return object_ids;
    }

    void MotionPlanningFrame::KinectRGBSubscriberCallback()
    {
    }

    void MotionPlanningFrame::KinectPointCloudSubscriberCallback()
    {
    }

    void MotionPlanningFrame::computeRunDpmButtonClicked()
    {
        // Read in image.

        // Build bin object list.
        // std::vector<std::string> bin_objects = computeObjectIdsInBinFromJson(bin_id);
    }

    void MotionPlanningFrame::computeRunDpm(const sensor_msgs::Image& image,
                                            const std::vector<std::string>& target_object_ids,
                                            const std::vector<std::string>& bin_object_ids,
                                            const std::string& bin_id)
    {
        // Downsample image to bin area.

        // Call service.
        computeRunDpm(image, target_object_ids, bin_object_ids);
    }

    void MotionPlanningFrame::computeRunDpm(const sensor_msgs::Image& image,
                                            const std::vector<std::string>& target_object_ids,
                                            const std::vector<std::string>& bin_object_ids)
    {
        apc_msgs::RunDPM srv;
        srv.request.image = image;
        srv.request.target_objects = target_object_ids;
        srv.request.bin_objects = bin_object_ids;
        computeRunDpm(srv);
    }

    void MotionPlanningFrame::computeRunDpm(apc_msgs::RunDPM& run_dpm_srv)
    {

    }

    void MotionPlanningFrame::computeIcp(apc_msgs::RunICP& run_icp_srv)
    {

    }

    void MotionPlanningFrame::computeRunVisionButtonClicked()
    {
        try {
            computeRunVision();
        } catch (std::exception& error) {
            ROS_ERROR("Caught exception in %s", error.what());
        }
    }

    void MotionPlanningFrame::computeRunVision()
    {
        APC_ASSERT(_kiva_pod,
                   "Failed to get KIVA pod model");
        KeyPoseMap world_state = computeWorldKeyPoseMap();
        apc_msgs::RunVision run_vision;
        std::string target_frame = "crichton_origin";
        std::string source_frame = "crichton_origin";
        // Get crichton origin to kinect.
        // tf::StampedTransform tf_optical_world;
        // ros::Time t = ros::Time::now();
        // APC_ASSERT(_tf_listener.waitForTransform(target_frame, source_frame, t, ros::Duration(2.0)),
        //            "Failed to wait for transform");
        // _tf_listener.lookupTransform(target_frame, source_frame, t, tf_optical_world);
        // geometry_msgs::TransformStamped spose_optical_world;
        // tf::transformStampedTFToMsg(tf_optical_world, spose_optical_world);
        // Eigen::Affine3d T_optical_world;
        // tf::transformMsgToEigen(spose_optical_world.transform, T_optical_world);
        // Build bin information.
        APC_ASSERT(ui_->bin_contents_table_widget->rowCount() > 0,
                   "Failed to find any items in the bins");
        Eigen::Affine3d T_pod_world;
        {
            T_pod_world = planning_display_->getPlanningSceneRO()->getWorld()->getObject("kiva_pod")->shape_poses_[0];
        }

        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        for (int i = 0; i < bin_contents->rowCount(); i++) {
            std::string bin_id = bin_contents->item(i, 0)->text().toStdString();
            int bin_index = bin_id[4] - 'A';
            APC_ASSERT(0 <= bin_index && bin_index < 12,
                       "Failed to compute bin index for bin %s", bin_id.c_str());
            std::string item_id = bin_contents->item(i, 1)->text().toStdString();
            apc_msgs::Object object_msg;
            object_msg.object_id = item_id;
            run_vision.request.bins[bin_index].object_list.push_back(object_msg);
            run_vision.request.bins[bin_index].bin_name = bin_id;
            shapes::Box* box = dynamic_cast<shapes::Box*>(_kiva_pod->getLink(bin_id)->getState().shapes[0].get());
            APC_ASSERT(box,
                       "Failed to get geometry for bin %s", bin_id.c_str());
            run_vision.request.bins[bin_index].bin_size.x = box->size[0];
            run_vision.request.bins[bin_index].bin_size.y = box->size[1];
            run_vision.request.bins[bin_index].bin_size.z = box->size[2];
            Eigen::Affine3d T_bin_pod = _kiva_pod->getGlobalTransform(bin_id);
            Eigen::Affine3d T_bin_world = T_pod_world.inverse() * _kiva_pod->getGlobalTransform(bin_id);
            // Eigen::Affine3d T_bin_optical = T_optical_world.inverse() * T_bin_world;
            tf::poseEigenToMsg(T_bin_world, run_vision.request.bins[bin_index].bin_pose);
            run_vision.request.bins[bin_index].header.frame_id = target_frame;
        }
        run_vision.request.camera_id = "kinect_lower";
        // Run service.
        APC_ASSERT(_run_vision_client.call(run_vision),
                   "Failed call run vision service");
    }

}
