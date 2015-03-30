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

namespace moveit_rviz_plugin
{

    void MotionPlanningFrame::loadOptionsToView(QList<QListWidgetItem*> items, bool enable)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            data.push_back(items[i]->data(Qt::UserRole));
        loadOptionsToView(data, enable);
    }

    void MotionPlanningFrame::loadOptionsToView(QList<QTreeWidgetItem*> items, bool enable)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            if (items[i]->childCount() > 0)
                for (int j = 0; j < items[i]->childCount(); j++)
                    data.push_back(items[i]->child(j)->data(0, Qt::UserRole));
            else
                data.push_back(items[i]->data(0, Qt::UserRole));
        loadOptionsToView(data, enable);
    }

    void MotionPlanningFrame::loadOptionsToView(std::vector<QVariant>& data, bool enable)
    {
        // The number of checkboxes.
        const int num_checkboxes = 7;

        // Array of checkbox pointers.
        class QCheckBox* checkbox[num_checkboxes] = { ui_->relative_to_object_checkbox,
                                                      ui_->relative_to_pose_checkbox,
                                                      ui_->eef_trajectory_checkbox,
                                                      ui_->dense_trajectory_checkbox,
                                                      ui_->monitor_contact_checkbox,
                                                      ui_->monitor_profile_checkbox,
                                                      ui_->cartesian_interpolate_checkbox };

        // Clear and disable all the options.
        for (int i = 0; i < num_checkboxes; i++)
        {
            checkbox[i]->setChecked(false);
            checkbox[i]->setEnabled(false);
            checkbox[i]->setTristate(false);
        }

        // Exit if there's nothing to display.
        if (data.size() == 0)
            return;

        // Set all options to enabled or disabled.
        for (int i = 0; i < num_checkboxes; i++)
        {
            checkbox[i]->setEnabled(enable);
        }

        // Copy the options over to view.
        bool init = false;
        for (int i = 0; i < data.size(); i++)
        {
            // Get the thing out of the data.
            apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[i]);
            for (int j = 0; j < plan.actions.size(); j++)
            {
                const apc_msgs::PrimitiveAction& action = plan.actions[j];
                unsigned char options[num_checkboxes] = { action.relative_to_object,
                                                          action.relative_to_previous_pose,
                                                          action.use_eef_trajectory,
                                                          action.dense_trajectory,
                                                          action.stop_on_contact,
                                                          action.use_haptic_profile,
                                                          action.interpolate_cartesian };
                // Set the tristate of the checkbox.
                for (int k = 0; k < num_checkboxes; k++)
                    setTristateCheckBox(checkbox[k], options[k], init);
                init = true;
            }
        }
    }

    void MotionPlanningFrame::setTristateCheckBox(QCheckBox* checkbox, bool b, bool init)
    {
        if (!init)
            checkbox->setChecked(b);
        else if ((int) checkbox->checkState() != 2 * (int) b)
        {
            checkbox->setTristate();
            checkbox->setChecked(Qt::PartiallyChecked);
            checkbox->setEnabled(false);
        }
        checkbox->update();
    }

    void MotionPlanningFrame::saveOptionsFromView(QList<QListWidgetItem*> items)
    {
        std::vector<QVariant> data;
        for (int i = 0; i < items.count(); i++)
            data.push_back(items[i]->data(Qt::UserRole));

        saveOptionsFromView(data);

        for (int i = 0; i < items.count(); i++)
            items[i]->setData(Qt::UserRole, data[i]);
    }

    void MotionPlanningFrame::saveOptionsFromView(std::vector<QVariant>& data)
    {
        if (!ui_->relative_to_object_checkbox->isEnabled())
            return;

        for (int i = 0; i < data.size(); i++)
        {
            // Get message from data.
            apc_msgs::PrimitivePlan plan = getMessageFromUserData<apc_msgs::PrimitivePlan>(data[i]);

            // Fill message options.
            for (int j = 0; j < plan.actions.size(); j++)
            {
                apc_msgs::PrimitiveAction& action = plan.actions[i];
                QCheckBox* checkbox[7] = { ui_->relative_to_object_checkbox,
                                           ui_->relative_to_pose_checkbox,
                                           ui_->eef_trajectory_checkbox,
                                           ui_->dense_trajectory_checkbox,
                                           ui_->monitor_contact_checkbox,
                                           ui_->monitor_profile_checkbox,
                                           ui_->cartesian_interpolate_checkbox };
                unsigned char* options[7] = { &action.relative_to_object,
                                              &action.relative_to_previous_pose,
                                              &action.use_eef_trajectory,
                                              &action.dense_trajectory,
                                              &action.stop_on_contact,
                                              &action.use_haptic_profile,
                                              &action.interpolate_cartesian };
                for (int k = 0; k < 7; k++)
                    if (checkbox[k]->checkState() == Qt::PartiallyChecked)
                        ROS_WARN("Skipping partially checked checkbox: %s", checkbox[k]->text().toStdString().c_str());
                    else
                        *options[k] = checkbox[k]->isChecked();
            }

            // Set message back into user data!
            setMessageToUserData<apc_msgs::PrimitivePlan>(data[i], plan);
        }
    }

}
