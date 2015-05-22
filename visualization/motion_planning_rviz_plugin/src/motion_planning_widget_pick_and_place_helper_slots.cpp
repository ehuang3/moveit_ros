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


namespace moveit_rviz_plugin
{
    void MotionPlanningFrame::connectPickAndPlaceHelperSlots()
    {
        connect( ui_->bin_contents_table_widget, SIGNAL( itemClicked(QTableWidgetItem*) ),
                 this, SLOT( binContentsItemClicked(QTableWidgetItem*) ));
        connect( ui_->test_pregrasps_button,    SIGNAL( clicked() ), this, SLOT( testPreGraspsButtonClicked() ));
        // connect( ui_->test_ik_button,    SIGNAL( clicked() ), this, SLOT( testIkButtonClicked() ));
        connect( ui_->test_collisions_button,    SIGNAL( clicked() ), this, SLOT( testCollisionsButtonClicked() ));
        connect( ui_->test_grasps_button,    SIGNAL( clicked() ), this, SLOT( testGraspsButtonClicked() ));

        connect( ui_->test_enter_button,    SIGNAL( clicked() ), this, SLOT( testEnterButtonClicked() ));
        connect( ui_->test_exit_button,    SIGNAL( clicked() ), this, SLOT( testExitButtonClicked() ));
        connect( ui_->test_plan_button,    SIGNAL( clicked() ), this, SLOT( testPlanButtonClicked() ));
        connect( ui_->test_trajopt_button,    SIGNAL( clicked() ), this, SLOT( testTrajoptButtonClicked() ));
    }

    void MotionPlanningFrame::updateBinContentsTableWidget(rapidjson::Document& doc)
    {
        // Get the bin contents table widget.
        QTableWidget* widget = ui_->bin_contents_table_widget;

        // Set bin contents into the table widget.
        const rapidjson::Value& bin_contents = doc["bin_contents"];
        char const * bin_ids[] = { "bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F",
                                   "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L" };
        int num_rows = 0;
        int num_cols = 2;
        for (int i = 0; i < 12; i++)
        {
            num_rows += bin_contents[bin_ids[i]].Size();
        }
        widget->setRowCount(num_rows);
        widget->setColumnCount(num_cols);
        int r = 0;
        for (int i = 0; i < 12; i++)
        {
            const rapidjson::Value& bin = bin_contents[bin_ids[i]];
            for (int j = 0; j < bin.Size(); j++)
            {
                QTableWidgetItem* new_bin = new QTableWidgetItem(bin_ids[i]);
                QTableWidgetItem* new_item = new QTableWidgetItem(bin[j].GetString());
                widget->setItem(r, 0, new_bin);
                widget->setItem(r, 1, new_item);
                r++;
            }
        }
        QStringList labels;
        labels.append("Bin");
        labels.append("Item");
        widget->setHorizontalHeaderLabels(labels);

        // Load bin contents into planning scene.
        computeLoadBinContentsToScene();
    }

    void MotionPlanningFrame::binContentsItemClicked(QTableWidgetItem* item)
    {
        std::string item_key = item->data(Qt::UserRole).toString().toStdString();
        bool create_marker = !_item_marker || _item_marker->getName() != ("marker_" + item_key);
        if (create_marker)
        {
            createInteractiveMarkerForItem(item_key);
            // Leave the object higher up in the combo box.
            updateFrameComboBox();
            updateObjectComboBox();
        }
        else
        {
            ui_->bin_contents_table_widget->clearSelection();
            _item_marker.reset();
        }
    }

    void MotionPlanningFrame::binContentsItemDoubleClicked(QTableWidgetItem* item)
    {
    }

    void MotionPlanningFrame::testPreGraspsButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestPreGraspsButtonClicked, this), "test pregraps");
    }

    void MotionPlanningFrame::testIkButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestIkButtonClicked, this), "test ik");
    }

    void MotionPlanningFrame::testCollisionsButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestCollisionsButtonClicked, this), "test ik");
    }

    void MotionPlanningFrame::testGraspsButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestGraspsButtonClicked, this), "test ik");
    }

    void MotionPlanningFrame::testEnterButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestEnterButtonClicked, this), "test ik");
    }
    void MotionPlanningFrame::testExitButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestExitButtonClicked, this), "test ik");
    }
    void MotionPlanningFrame::testPlanButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestPlanButtonClicked, this), "test ik");
    }
    void MotionPlanningFrame::testTrajoptButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTestTrajoptButtonClicked, this), "test ik");
    }
}
