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
#include <QListWidget>
#include <apc_msgs/ItemSymmetry.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/warehouse/item_symmetry_storage.h>
#include <apc/planning.h>
#include <QMessageBox>


namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::connectSymmetrySlots()
    {
        connect( ui_->save_symmetries_button,    SIGNAL( clicked() ), this, SLOT( saveSymmetriesButtonClicked() ));
        connect( ui_->load_symmetries_button,    SIGNAL( clicked() ), this, SLOT( loadSymmetriesButtonClicked() ));
        connect( ui_->add_symmetry_button,       SIGNAL( clicked() ), this, SLOT( addSymmetryButtonClicked() ));
        connect( ui_->delete_symmetry_button,    SIGNAL( clicked() ), this, SLOT( deleteSymmetryButtonClicked() ));
        connect( this, SIGNAL( itemsLoadedHook() ), this, SLOT( loadItemsForSymmetryList() ));
        connect( ui_->items_for_symmetry_list, SIGNAL( itemClicked(QListWidgetItem*) ),
                 this, SLOT( itemForSymmetryClicked(QListWidgetItem*) ));
        connect( ui_->symmetries_list, SIGNAL( itemClicked(QListWidgetItem*) ),
                 this, SLOT( symmetryClicked(QListWidgetItem*) ));
    }

    void MotionPlanningFrame::loadItemsForSymmetryList()
    {
        QTableWidget* bin_contents = ui_->bin_contents_table_widget;
        std::vector<std::string> item_ids;
        for (int i = 0; i < bin_contents->rowCount(); i++) {
            std::string item_id = bin_contents->item(0, 1)->text().toStdString();
            if (std::find(item_ids.begin(), item_ids.end(), item_id) == item_ids.end())
                item_ids.push_back(item_id);
        }
        loadItemsForSymmetryList(item_ids);
    }

    void MotionPlanningFrame::loadItemsForSymmetryList(const std::vector<std::string>& item_ids)
    {
        QListWidget* item_list = ui_->items_for_symmetry_list;
        item_list->clear();
        for (int i = 0; i < item_ids.size(); i++) {
            QListWidgetItem* item = new QListWidgetItem;
            item->setText(QString::fromStdString(item_ids[i]));
            item->setFlags(item->flags() | Qt::ItemIsEditable);
            item_list->insertItem(i, item);
        }
    }

    void MotionPlanningFrame::itemForSymmetryClicked(QListWidgetItem* clicked_item)
    {
        std::vector<std::string> object_keys;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            object_keys = ps->getWorld()->getObjectIds();
        }

        // HACK Move all other items to the last bin by reloading them to 'bin_L'.
        for (int i = 0; i < object_keys.size(); i++) {
            if (testForItemKey(object_keys[i])) {
                addItemToScene("", object_keys[i], "bin_L");
            }
        }

        // Load new items to the world.
        std::string item_id = clicked_item->text().toStdString();
        std::string item_model_path = computeItemModelPath(item_id);
        std::string item_key_origin = item_id + "_origin";
        std::string item_key_symmetry = item_id + "_symmetry";

        addItemToScene(item_model_path,
                       item_key_origin,
                       "bin_A");
        addItemToScene(item_model_path,
                       item_key_symmetry,
                       "bin_A");

        // Create interactive marker for the symmetry marker.
        createInteractiveMarkerForItem(item_key_symmetry);
    }

    void MotionPlanningFrame::saveSymmetriesButtonClicked()

    {
        if (!item_symmetry_storage_) {
            ROS_ERROR("MISSING ITEM SYMMETRY STORAGE");
            return;
        }

        QListWidget* symmetries_list = ui_->symmetries_list;

        // List of plans in the database.
        std::vector<std::string> database_names;
        item_symmetry_storage_->getKnownItemSymmetrys(database_names);

        // Drop all old symmetries from the database. FIXME This might be very inefficient?
        for (int i = 0; i < database_names.size(); i++)
            try
            {
                item_symmetry_storage_->removeItemSymmetry(database_names[i]);
            }
            catch (std::runtime_error &ex)
            {
                ROS_ERROR("%s", ex.what());
            }

        for (int i = 0; i < symmetries_list->count(); i++)
        {
            // Get the toplevel item.
            QListWidgetItem* root = symmetries_list->item(i);

            // Get the primitive symmetry.
            apc_msgs::ItemSymmetry symmetry = getMessageFromUserData<apc_msgs::ItemSymmetry>(root->data(Qt::UserRole));

            // Store message in database.
            try
            {
                item_symmetry_storage_->addItemSymmetry(symmetry, symmetry.symmetry_id);
            }
            catch (std::runtime_error &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }
    }

    void MotionPlanningFrame::loadSymmetriesButtonClicked()
    {
        if (!item_symmetry_storage_)
            return;

        // Names of the stored symmetrys.
        std::vector<std::string> symmetry_names;

        // Get all the stored symmetrys.
        try
        {
            item_symmetry_storage_->getKnownItemSymmetrys(symmetry_names);
        }
        catch (std::runtime_error &ex)
        {
            QMessageBox::warning(this, "Cannot query the database", QString("Error: ").append(ex.what()));
            return;
        }

        // Clear the current stored symmetry tree.
        ui_->symmetries_list->clear();

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

            // Create tree widget item to hold the symmetry.
            QListWidgetItem* root = new QListWidgetItem;
            root->setFlags(root->flags() | Qt::ItemIsEditable);
            root->setText(QString(symmetry->symmetry_id.c_str()));

            // Store overall symmetry in root node.
            QVariant data;
            setMessageToUserData<apc_msgs::ItemSymmetry>(data, *symmetry);
            root->setData(Qt::UserRole, data);

            ui_->symmetries_list->insertItem(i, root);
        }

    }

    void MotionPlanningFrame::addSymmetryButtonClicked()
    {
        if (!item_symmetry_storage_) {
            ROS_ERROR("MISSING ITEM SYMMETRY STORAGE");
            return;
        }

        QListWidget* item_list = ui_->items_for_symmetry_list;
        QListWidgetItem* item = item_list->currentItem();
        if (!item) {
            ROS_ERROR("Can't add symmetry without an item");
            return;
        }
        std::string item_id = item->text().toStdString();
        std::string item_key_origin = item_id + "_origin";
        std::string item_key_symmetry = item_id + "_symmetry";
        std::string item_symmetry_id = item_id + "_%i";
        std::vector<std::string> existing_symmetries;
        item_symmetry_storage_->getKnownItemSymmetrys(existing_symmetries);
        apc_planning::formatUniqueIndex(item_symmetry_id, existing_symmetries);

        apc_msgs::ItemSymmetry item_symmetry;
        {
            planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
            Eigen::Affine3d T_origin_world = ps->getWorld()->getObject(item_key_origin)->shape_poses_[0];
            Eigen::Affine3d T_symmetry_world = ps->getWorld()->getObject(item_key_symmetry)->shape_poses_[0];
            Eigen::Affine3d T_symmetry_origin = T_origin_world.inverse() * T_symmetry_world;
            tf::poseEigenToMsg(T_symmetry_origin, item_symmetry.pose_symmetry_origin);
        }
        item_symmetry.item_id = item_id;
        item_symmetry.symmetry_id = item_symmetry_id;

        QVariant data;
        setMessageToUserData<apc_msgs::ItemSymmetry>(data, item_symmetry);

        QListWidgetItem* symmetry_item = new QListWidgetItem;
        symmetry_item->setFlags(symmetry_item->flags() | Qt::ItemIsEditable);
        symmetry_item->setData(Qt::UserRole, data);
        symmetry_item->setText(QString::fromStdString(item_symmetry.symmetry_id));

        int row = ui_->symmetries_list->count() + 1;
        ui_->symmetries_list->insertItem(row, symmetry_item);

        saveSymmetriesButtonClicked();
    }

    void MotionPlanningFrame::deleteSymmetryButtonClicked()
    {
        if (!item_symmetry_storage_) {
            ROS_ERROR("MISSING ITEM SYMMETRY STORAGE");
            return;
        }

        int row = ui_->symmetries_list->currentRow();
        QListWidgetItem* item = ui_->symmetries_list->takeItem(row);
        if (item)
            delete item;

        saveSymmetriesButtonClicked();
        loadSymmetriesButtonClicked();
    }

    void MotionPlanningFrame::symmetryClicked(QListWidgetItem* clicked_item)
    {
        apc_msgs::ItemSymmetry symmetry = getMessageFromUserData<apc_msgs::ItemSymmetry>(clicked_item->data(Qt::UserRole));
        ROS_INFO_STREAM(symmetry);
    }

}
