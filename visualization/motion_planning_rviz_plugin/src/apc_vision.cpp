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
#include <ros/ros.h>
#include <apc/vision.h>
#include <apc/exception.h>
#include <eigen_conversions/eigen_msg.h>


void apc_vision::getBinItemKeys(BinKeyMap& bin_item_keys,
                                const QTableWidget* bin_contents)
{
    bin_item_keys.clear();
    for (int i = 0; i < bin_contents->rowCount(); i++) {
        std::string bin_id = bin_contents->item(i, 0)->text().toStdString();
        std::string item_key = bin_contents->item(i, 1)->data(Qt::UserRole).toString().toStdString();
        bin_item_keys[bin_id].push_back(item_key);
    }
}

void apc_vision::assignItemPosesFromBinStates(KeyPoseMap& world_state,
                                              const apc_msgs::RunVisionResponse& response,
                                              const BinKeyMap& bin_item_keys)
{
    typedef std::vector<std::string> Keys;
    typedef std::map<std::string, bool> Dirt;
    // Copy objects to their bin poses in the world state.
    for (int i = 0; i < response.bin_contents.size(); i++) {
        const apc_msgs::BinState& B = response.bin_contents[i];
        std::string bin_id = B.bin_name;
        const Keys& K = bin_item_keys.find(bin_id)->second;
        Dirt D;
        for (Keys::const_iterator k = K.begin(); k != K.end(); ++k) {
            D[*k] = true;
        }
        for (int j = 0; j < B.object_list.size(); j++) {
            for (Dirt::iterator d = D.begin(); d != D.end(); ++d) {
                if (d->second == false)
                    continue;
                const apc_msgs::ObjectState& O = B.object_list[j];
                if (d->first.find(O.object_id) == 0) {
                    d->second = false;
                    tf::poseMsgToEigen(O.object_pose, world_state[d->first]);
                }
            }
        }
        for (Dirt::const_iterator d = D.begin(); d != D.end(); ++d) {
            if (d->second == true)
                ROS_WARN("Failed to set item key %s pose from vision", d->first.c_str());
        }
    }
}
