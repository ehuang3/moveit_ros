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

    void MotionPlanningFrame::loadObjectsButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeLoadObjectsButtonClicked, this),
                                            "Load objects");
    }

    void MotionPlanningFrame::saveObjectsButtonClicked()
    {
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSaveObjectsButtonClicked, this),
                                            "Save objects");
    }

    void MotionPlanningFrame::objectSelectionChanged()
    {
        // Get selections.
        QList<QListWidgetItem*> selection = ui_->object_list_widget->selectedItems();
        if (selection.empty())
            return;

        // Set selection into collision object list. This allows us to hook into
        // the object tab functionality for generating interactive markers for
        // the objects.
        QListWidget* collision_objects = ui_->collision_objects_list;
        for (int i = 0; i < selection.size(); i++)
            for (int j = 0; j < collision_objects->count(); j++)
                if (collision_objects->item(j)->text() == selection[i]->text())
                    collision_objects->item(j)->setSelected(true);

        // Change selected collision object.
        selectedCollisionObjectChanged();
    }

    void MotionPlanningFrame::objectClicked(QListWidgetItem* item)
    {
        if (!item)
            return;
        // This should avoid the problems with itemChanged approach.

        // Determine whether the checked state has changed for the object.
        int index = item->type();
        bool checked = item->checkState() == Qt::Checked;
        bool toggled = attached_objects_[index].second != checked;
        if (!toggled)
            return;
        attached_objects_[index].second = checked;

        // Get object name.
        std::string object_name = item->text().toStdString();

        // Get the goal state.
        robot_state::RobotState state = *planning_display_->getQueryGoalState();

        // If the checked state has changed, attach or detach an object.
        if (checked)
        {
            // Ask user for link to attach object to.
            QStringList links;
            const std::vector<std::string> &links_std =
                planning_display_->getRobotModel()->
                getJointModelGroup(planning_display_->getCurrentPlanningGroup())->getLinkModelNames();
            for (std::size_t i = 0 ; i < links_std.size() ; ++i)
                links.append(QString::fromStdString(links_std[i]));
            bool ok = false;
            QString response = QInputDialog::getItem(this, tr("Select Link Name"), tr("Choose the link to attach to:"),
                                                     links, 0, false, &ok);
            std::string link_name = response.toStdString();

            computeAttachObjectToState(state, object_name, link_name);
        }
        else
        {
            computeDetachObjectFromState(state, object_name);
        }

        // One of these should work.
        planning_display_->setQueryGoalState(state);
        planning_display_->queueRenderSceneGeometry();
    }

}
