/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
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

/* Author: Ioan Sucan */

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_

#include <QWidget>
#include <QTreeWidgetItem>
#include <QListWidgetItem>
#include <QCheckBox>

#ifndef Q_MOC_RUN
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>
#include <moveit/semantic_world/semantic_world.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib/client/simple_action_client.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#endif


#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <apc_msgs/FollowPrimitivePlanAction.h>
#include <map>
#include <string>

namespace rviz
{
class DisplayContext;
}

namespace Ui
{
class MotionPlanningUI;
}

namespace moveit_warehouse
{
class PlanningSceneStorage;
class ConstraintsStorage;
class RobotStateStorage;
class PrimitivePlanStorage;
}


namespace moveit_rviz_plugin
{
class MotionPlanningDisplay;

const std::string OBJECT_RECOGNITION_ACTION = "/recognize_objects";

class MotionPlanningFrame : public QWidget
{
  friend class MotionPlanningDisplay;
  Q_OBJECT

public:
  MotionPlanningFrame(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent = 0);
  ~MotionPlanningFrame();

  void changePlanningGroup();
  void enable();
  void disable();
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

protected:
  static const int ITEM_TYPE_SCENE = 1;
  static const int ITEM_TYPE_QUERY = 2;

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq);

  void updateSceneMarkers(float wall_dt, float ros_dt);

  void updateExternalCommunication();

  MotionPlanningDisplay *planning_display_;
  rviz::DisplayContext* context_;
  Ui::MotionPlanningUI *ui_;

  boost::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;
  boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  boost::shared_ptr<moveit::semantic_world::SemanticWorld> semantic_world_;

  boost::shared_ptr<moveit::planning_interface::MoveGroup::Plan> current_plan_;
  boost::shared_ptr<apc_msgs::PrimitivePlan> primitive_plan_;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;
  boost::shared_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  boost::shared_ptr<moveit_warehouse::RobotStateStorage> robot_state_storage_;
  boost::shared_ptr<moveit_warehouse::PrimitivePlanStorage> primitive_plan_storage_;

  typedef std::map<std::string, boost::shared_ptr<moveit_warehouse::PrimitivePlanStorage> > PlanDatabaseMap;
  PlanDatabaseMap plan_databases_;

  boost::shared_ptr<rviz::InteractiveMarker> scene_marker_;

  typedef std::map<std::string, moveit_msgs::RobotState> RobotStateMap;
  typedef std::pair<std::string, moveit_msgs::RobotState> RobotStatePair;
  RobotStateMap robot_states_;

private Q_SLOTS:

  //Context tab
  void databaseConnectButtonClicked();
  void publishSceneButtonClicked();
  void planningAlgorithmIndexChanged(int index);
  void resetDbButtonClicked();
  void approximateIKChanged(int state);

  //Planning tab
  void planButtonClicked();
  void executeButtonClicked();
  void planAndExecuteButtonClicked();
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void allowExternalProgramCommunication(bool enable);
  void pathConstraintsIndexChanged(int index);
  void useStartStateButtonClicked();
  void useGoalStateButtonClicked();
  void onClearOctomapClicked();

  // Stored plans tab
  // Stored plans commands
  void planActiveGoalsButtonClicked();
  void pickAndPlaceButtonClicked();
  void planGoalsButtonClicked();
  void previewButtonClicked();
  void executePlansButtonClicked();
  void stopExecutionButtonClicked();
  // Stored plans active goals commands
  void setStartToCurrentButtonClicked();
  void setGoalToCurrentButtonClicked();
  void insertGoalButtonClicked();
  void deleteGoalButtonClicked();
  // Stored plans active goals list
  void activeGoalSelectionChanged();
  void activeGoalListClicked(const QModelIndex& index);
  void activeGoalItemClicked(QListWidgetItem* item);
  void activeGoalItemDoubleClicked(QListWidgetItem* item);
  // Stored plans stored plans tree
  void storedPlanTreeClicked(const QModelIndex& index);
  void storedPlanItemClicked(QTreeWidgetItem* item, int col);
  void storedPlanItemDoubleClicked(QTreeWidgetItem* item, int col);
  // Stored plans active <-> stored commands
  void activeToStoredPlansButtonClicked();
  void storedToActiveGoalsButtonClicked();
  // Stored plans stored plans database commands
  void planDatabaseComboBoxChanged();
  void savePlansButtonClicked();
  void loadPlansButtonClicked();
  void deleteStoredPlanButtonClicked();
  void planDatabaseNameChanged(const QString& text);
  // Stored plans options
  void optionsCheckBoxClicked();
  // Object panel group
  void loadObjectsButtonClicked();
  void saveObjectsButtonClicked();
  void objectClicked(QListWidgetItem* item);
  void objectSelectionChanged();

  //Scene Objects tab
  void importFileButtonClicked();
  void importUrlButtonClicked();
  void clearSceneButtonClicked();
  void sceneScaleChanged(int value);
  void sceneScaleStartChange();
  void sceneScaleEndChange();
  void removeObjectButtonClicked();
  void selectedCollisionObjectChanged();
  void objectPoseValueChanged(double value);
  void collisionObjectChanged(QListWidgetItem *item);
  void imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void copySelectedCollisionObject();
  void exportAsTextButtonClicked();
  void importFromTextButtonClicked();

  //Stored scenes tab
  void saveSceneButtonClicked();
  void planningSceneItemClicked();
  void saveQueryButtonClicked();
  void deleteSceneButtonClicked();
  void deleteQueryButtonClicked();
  void loadSceneButtonClicked();
  void loadQueryButtonClicked();
  void warehouseItemNameChanged(QTreeWidgetItem *item, int column);

  //States tab
  void loadStateButtonClicked();
  void saveStartStateButtonClicked();
  void saveGoalStateButtonClicked();
  void removeStateButtonClicked();
  void clearStatesButtonClicked();
  void setAsStartStateButtonClicked();
  void setAsGoalStateButtonClicked();

  //Pick and place
  void detectObjectsButtonClicked();
  void pickObjectButtonClicked();
  void placeObjectButtonClicked();
  void selectedDetectedObjectChanged();
  void detectedObjectChanged(QListWidgetItem *item);
  void selectedSupportSurfaceChanged();  

  //General
  void tabChanged(int index);

private:

  //Context tab
  void computeDatabaseConnectButtonClicked();
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeResetDbButtonClicked(const std::string &db);
  void populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc);

  //Planning tab
  void computePlanButtonClicked();
  void computeExecuteButtonClicked();
  void computePlanAndExecuteButtonClicked();
  void computePlanAndExecuteButtonClickedDisplayHelper();
  void populateConstraintsList();
  void populateConstraintsList(const std::vector<std::string> &constr);
  void configureForPlanning(const robot_state::RobotState& start, const robot_state::RobotState& goal);
  void configureForPlanning();
  void configureWorkspace();
  void updateQueryStateHelper(robot_state::RobotState &state, const std::string &v);
  void fillStateSelectionOptions();


  //Stored plans tab
  boost::shared_ptr<actionlib::SimpleActionClient<apc_msgs::FollowPrimitivePlanAction> > execute_client_;
  template< typename Message > static QByteArray serializeMessage(const Message& msg);
  template< typename Message > static Message deserializeMessage(const QByteArray& string);
  template< typename Message > static Message getMessageFromUserData(const QVariant& data);
  template< typename Message > static void setMessageToUserData(QVariant& data, const Message& msg);
  void getRobotStateFromUserData(const QVariant& data, robot_state::RobotState& robot);
  void getStateFromAction(robot_state::RobotState& robot, const apc_msgs::PrimitiveAction& action);
  void saveGoalToItem(QListWidgetItem* item);
  void saveGoalToItem(const robot_state::RobotState& state, QListWidgetItem* item);
  void loadGoalFromItem(QListWidgetItem* item);
  void loadGoalFromItem(QTreeWidgetItem* item);
  void loadGoalFromData(const QVariant& data);
  // displaying waypoints
  void loadWaypointsToDisplay(QList<QListWidgetItem*> items);
  void loadWaypointsToDisplay(QList<QTreeWidgetItem*> items);
  void loadWaypointsToDisplay(std::vector<QVariant>& data);
  // plan options
  void loadOptionsToView(QList<QListWidgetItem*> items, bool enable = true);
  void loadOptionsToView(QList<QTreeWidgetItem*> items, bool enable = true);
  void loadOptionsToView(std::vector<QVariant>& data, bool enable=true);
  void setTristateCheckBox(QCheckBox* checkbox, bool b, bool init);
  void saveOptionsFromView(QList<QListWidgetItem*> items);
  void saveOptionsFromView(std::vector<QVariant>& data);
  // plan database
  void computeSavePlansButtonClicked();
  void computeLoadPlansButtonClicked();
  // primitive planning
  void computePickAndPlaceButtonClicked();
  void computePlanGoalsButtonClicked();
  void computeLinearInterpPlan(const robot_state::RobotState& start, apc_msgs::PrimitiveAction& goal);
  void copyTrajectoryToDisplay(const moveit_msgs::RobotState& start_state, const apc_msgs::PrimitivePlan& plan);
  void appendToTrajectory(trajectory_msgs::JointTrajectory& first, const trajectory_msgs::JointTrajectory& second);
  // execution
  void initExecutePlans();
  void computeExecutePlansButtonClicked();
  void executeActiveCallback();
  void executeFeedbackCallback(const apc_msgs::FollowPrimitivePlanFeedbackConstPtr &feedback);
  void executeDoneCallback(const actionlib::SimpleClientGoalState &state, const apc_msgs::FollowPrimitivePlanResultConstPtr &result);
  void updateExecuteActive();
  void updateExecuteFeedback(const apc_msgs::FollowPrimitivePlanFeedback &feedback);
  void updateExecuteDone(const actionlib::SimpleClientGoalState& state, const apc_msgs::FollowPrimitivePlanResult& result);
  // objects
  std::string computeObjectScenePath();
  void computeLoadObjectsButtonClicked();
  void computeSaveObjectsButtonClicked();
  void computePopulateObjects();
  void computeAttachObjectToState(robot_state::RobotState& state, const std::string& object_name, const std::string& link_name);
  void computeAttachObjectToState(robot_state::RobotState& state, const std::string& object_name, const std::string& link_name, const EigenSTL::vector_Affine3d& poses);
  void computeAttachObjectToState(robot_state::RobotState& state,
                                  const std::string& object_name,
                                  const std::string& link_name,
                                  const std::vector<geometry_msgs::Pose>& poses);
  void computeDetachObjectFromState(robot_state::RobotState& state, const std::string& object_name);
  void computeAttachObjectToPlan(apc_msgs::PrimitivePlan& plan, const robot_state::RobotState& state);
  void computeAttachObjectToAction(const robot_state::RobotState& state, apc_msgs::PrimitiveAction& action);
  void computeDetachObjectFromPlan(apc_msgs::PrimitivePlan& plan);
  void computeDetachObjectFromAction(apc_msgs::PrimitiveAction& action);
  // trajopt
  void loadStateFromAction(robot_state::RobotState& state, const apc_msgs::PrimitiveAction& action);
  void saveStateToAction(const robot_state::RobotState& state, apc_msgs::PrimitiveAction& action);
  bool computeTrajectoryFromActiveGoals();
  bool computeTrajectoryOptimizationSequence(apc_msgs::PrimitivePlan& plan);
  void computeTrajectoryOptimizationPlan();

  //Scene objects tab
  void addObject(const collision_detection::WorldPtr &world, const std::string &id,
                 const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);
  void updateCollisionObjectPose(bool update_marker_position);
  void createSceneInteractiveMarker();
  void renameCollisionObject(QListWidgetItem *item);
  void attachDetachCollisionObject(QListWidgetItem *item);
  void populateCollisionObjectsList();
  void computeImportFromText(const std::string &path);
  void computeExportAsText(const std::string &path);

  //Stored scenes tab
  void computeSaveSceneButtonClicked();
  void computeSaveQueryButtonClicked(const std::string &scene, const std::string &query_name);
  void computeLoadSceneButtonClicked();
  void computeLoadQueryButtonClicked();
  void populatePlanningSceneTreeView();
  void computeDeleteSceneButtonClicked();
  void computeDeleteQueryButtonClicked();
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem *s);
  void checkPlanningSceneTreeEnabledButtons();


  //States tab
  void saveRobotStateButtonClicked(const robot_state::RobotState &state);
  void populateRobotStatesList();

  //Pick and place
  void processDetectedObjects();  
  void updateDetectedObjectsList(const std::vector<std::string> &object_ids,
                                 const std::vector<std::string> &objects);
  void publishTables();  
  void updateSupportSurfacesList();
  ros::Publisher object_recognition_trigger_publisher_;
  std::map<std::string, std::string> pick_object_name_;
  std::string place_object_name_;
  std::vector<geometry_msgs::PoseStamped> place_poses_;
  void pickObject();
  void placeObject();
  void triggerObjectDetection();  
  void updateTables();  
  std::string support_surface_name_;
  // For coloring
  std::string selected_object_name_;
  std::string selected_support_surface_name_;  
  
  boost::scoped_ptr<actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> > object_recognition_client_;  
  template<typename T>
  void waitForAction(const T &action, const ros::NodeHandle &node_handle, const ros::Duration &wait_for_server, const std::string &name);
  void listenDetectedObjects(const object_recognition_msgs::RecognizedObjectArrayPtr &msg);  
  ros::Subscriber object_recognition_subscriber_;  
  
  ros::Subscriber plan_subscriber_;
  ros::Subscriber execute_subscriber_;
  ros::Subscriber update_start_state_subscriber_;
  ros::Subscriber update_goal_state_subscriber_;
  //General
  void changePlanningGroupHelper();
  void changePlanningGroupHelper(const std::string& group);
  void importResource(const std::string &path, std::string name = "", Eigen::Affine3d pose = Eigen::Affine3d::Identity());
  void loadStoredStates(const std::string& pattern);

  void remotePlanCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteExecuteCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateStartStateCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateGoalStateCallback(const std_msgs::EmptyConstPtr& msg);

  /* Selects or unselects a item in a list by the item name */
  void setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list);
  
  ros::NodeHandle nh_;
  ros::Publisher planning_scene_publisher_;
  ros::Publisher planning_scene_world_publisher_;

  collision_detection::CollisionWorld::ObjectConstPtr scaled_object_;



  std::vector< std::pair<std::string, bool> > known_collision_objects_;
  long unsigned int known_collision_objects_version_;
  bool first_time_;
  ros::ServiceClient clear_octomap_service_client_;

  ros::ServiceClient motion_plan_client_;

  // Convenience typdef for attached objects.
  typedef std::vector<std::pair<std::string, bool> > AttachedObjects;

  // List of attached objects.
  AttachedObjects attached_objects_;

  // List of file paths to the loaded objects.
  typedef std::map<std::string, std::string> ObjectPathMap;
  ObjectPathMap object_paths_;
};

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_impl.hpp>

}

#endif
