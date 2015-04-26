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
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <boost/xpressive/xpressive.hpp>
#include <QTextStream>


namespace moveit_rviz_plugin
{
    bool parseUriRef(const std::string& uriRef,
                     std::string& scheme,
                     std::string& authority,
                     std::string& path,
                     std::string& query,
                     std::string& fragment) {
        using namespace boost::xpressive;
        sregex rex = sregex::compile("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        smatch what;
        if (regex_match(uriRef, what, rex))
        {
            scheme = what[2];
            authority = what[4];
            path = what[5];
            query = what[7];
            fragment = what[9];
            return true;
        }
        return false;
    }

    void MotionPlanningFrame::connectPickAndPlaceSlots()
    {
        connect( ui_->run_apc_button,         SIGNAL( clicked() ), this, SLOT( runAPCButtonClicked() ));
        connect( ui_->randomize_bins_button,  SIGNAL( clicked() ), this, SLOT( randomizeBinsButtonClicked() ));
        connect( ui_->randomize_order_button, SIGNAL( clicked() ), this, SLOT( randomizeOrderButtonClicked() ));
        connect( ui_->reload_json_button, SIGNAL( clicked() ), this, SLOT( reloadJsonButtonClicked() ));
        connect( ui_->next_json_button, SIGNAL( clicked() ), this, SLOT( nextJsonButtonClicked() ));
        connect( ui_->prev_json_button, SIGNAL( clicked() ), this, SLOT( previousJsonButtonClicked() ));
        connect( ui_->triple_integral_button, SIGNAL( clicked() ), this, SLOT( tripleIntegralButtonClicked() ));
    }

    void MotionPlanningFrame::runAPCButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeRunAPCButtonClicked, this), "run APC");
    }

    void MotionPlanningFrame::randomizeBinsButtonClicked()
    {
    }

    void MotionPlanningFrame::randomizeOrderButtonClicked()
    {
    }

    void MotionPlanningFrame::reloadJsonButtonClicked()
    {
        // Load JSON from line edit's URI.
        std::string uri = ui_->json_path_line_edit->text().toStdString();
        std::string scheme, authority, path, query, fragment;
        if (!parseUriRef(uri, scheme, authority, path, query, fragment))
        {
            ROS_ERROR("Malformed URI: %s", uri.c_str());
            return;
        }
        // Load package path.
        path = ros::package::getPath(authority) + path;
        // Load JSON.
        QFile file(QString::fromStdString(path));
        if (!file.open(QFile::ReadOnly | QFile::Text))
        {
            ROS_ERROR("Failed to open file: %s", file.errorString().toStdString().c_str());
            return;
        }
        QTextStream in(&file);
        QString json;
        while (!in.atEnd())
            json += in.readLine() + "\n";
        file.close();
        // Load JSON into list view.
        try
        {
            rapidjson::Document doc;
            doc.Parse(json.toStdString().c_str());
            updateWorkOrderTableWidget(doc);
            updateBinContentsTableWidget(doc);
            updateFrameComboBox();
            updateObjectComboBox();
        }
        catch (std::logic_error& e)
        {
            ROS_ERROR("Caught rapidjson error: %s", e.what());
            return;
        }
    }

    void MotionPlanningFrame::updateWorkOrderTableWidget(rapidjson::Document& doc)
    {
        const rapidjson::Value& work_order = doc["work_order"];
        ui_->json_table_widget->setRowCount(work_order.Size());
        ui_->json_table_widget->setColumnCount(2);
        QStringList labels;
        labels.append("Bin");
        labels.append("Item");
        ui_->json_table_widget->setHorizontalHeaderLabels(labels);
        for (int i = 0; i < work_order.Size(); i++)
        {
            QString bin  = work_order[i]["bin"].GetString();
            QString item = work_order[i]["item"].GetString();
            // ui_->json_table_view->
            QTableWidgetItem* new_bin = new QTableWidgetItem(bin);
            QTableWidgetItem* new_item = new QTableWidgetItem(item);
            ui_->json_table_widget->setItem(i, 0, new_bin);
            ui_->json_table_widget->setItem(i, 1, new_item);
        }
        ui_->json_table_widget->setCurrentCell(0,0);
    }

    void MotionPlanningFrame::previousJsonButtonClicked()
    {
    }

    void MotionPlanningFrame::nextJsonButtonClicked()
    {
    }

    void MotionPlanningFrame::tripleIntegralButtonClicked()
    {
    }
}
