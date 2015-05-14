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
#include <QTableWidgetItem>
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
#include <apc_msgs/WorldState.h>
#include <apc_msgs/RunDPM.h>
#include <apc_msgs/RunICP.h>
#include <map>
#include <string>
#define RAPIDJSON_ASSERT(x) if (!(x)) throw std::logic_error(RAPIDJSON_STRINGIFY(x))
#include <rapidjson/document.h>
#include <robot_calibration/robot.h>
#include <apc/exception.h>

#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define ROS_DEBUG_FUNCTION                                              \
  ROS_DEBUG("%s at %s line %d", __FUNCTION__, __FILENAME__, __LINE__)   \


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
class ItemSymmetryStorage;
}


namespace moveit_rviz_plugin
{
class MotionPlanningDisplay;

const std::string OBJECT_RECOGNITION_ACTION = "/recognize_objects";

  // Convenience typedef for map from frame and object keys to poses.
  typedef std::map<std::string, Eigen::Affine3d, std::less<std::string>,
                   Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > >
  KeyPoseMap;

  // Convenience typedef for bin-object work order.
  typedef std::pair<std::string, std::string>
  WorkOrderItem;

  // Convenience typedef for bin-object work order.
  typedef std::vector<WorkOrderItem>
  WorkOrder;

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
  boost::shared_ptr<moveit_warehouse::ItemSymmetryStorage> item_symmetry_storage_;

  typedef std::map<std::string, boost::shared_ptr<moveit_warehouse::PrimitivePlanStorage> > PlanDatabaseMap;
  PlanDatabaseMap plan_databases_;

  boost::shared_ptr<rviz::InteractiveMarker> scene_marker_;

  typedef std::map<std::string, moveit_msgs::RobotState> RobotStateMap;
  typedef std::pair<std::string, moveit_msgs::RobotState> RobotStatePair;
  RobotStateMap robot_states_;

  // APC customizations.
private:
  boost::shared_ptr<actionlib::SimpleActionClient<apc_msgs::FollowPrimitivePlanAction> > execute_client_;

  std::map<std::string, int> _bin_item_counts;

  boost::shared_ptr<rviz::InteractiveMarker> _item_marker;

  boost::shared_ptr<robot_calibration::Robotd> _kiva_pod;

  std::vector<ros::ServiceClient> _compute_dense_motion_clients;

  // ros::ServiceClient _compute_dense_motion_client;

  ros::ServiceClient _run_vision_client;

  ros::ServiceClient _publish_shelf_client;

  tf::TransformListener _tf_listener;

  bool show_kiva_pod_;
  bool show_objects_;

private Q_SLOTS:
  // APC tab.

  // Teleoperation widget slots.
  void planButtonClicked();
  void previewButtonClicked();
  void executeButtonClicked();
  void stopButtonClicked();
  void initExecuteProgressLabel();
  void updateExecuteActive();
  void updateExecuteFeedback(const apc_msgs::FollowPrimitivePlanFeedback &feedback);
  void updateExecuteDone(const actionlib::SimpleClientGoalState& state,
                         const apc_msgs::FollowPrimitivePlanResult& result);

  // Teleoperation widget helper slots.
  void displayCurrentButtonToggled(bool checked);
  void displayStartButtonToggled(bool checked);
  void displayGoalButtonToggled(bool checked);
  void displayEefButtonToggled(bool checked);
  void displayJointsButtonToggled(bool checked);
  void displayKivaPodButtonToggled(bool checked);
  void displayObjectsButtonToggled(bool checked);
  void setCurrentToStartButtonClicked();
  void setCurrentToGoalButtonClicked();
  void setStartToGoalButtonClicked();
  void setGoalToStartButtonClicked();
  void padlockButtonToggled(bool);
  void startRadioButtonClicked();
  void goalRadioButtonClicked();
  void updateGroupComboBox();
  void groupComboBoxActivated(int);
  void updateGroupComboBoxFromAction(const apc_msgs::PrimitiveAction& action);
  void updateFrameComboBox();
  void frameComboBoxActivated(const QString& text);
  void updateFrameComboBoxFromAction(const apc_msgs::PrimitiveAction& action);
  void updateObjectComboBox();
  void objectComboBoxActivated(int);
  void objectComboBoxCurrentIndexChanged(const QString& text);
  void updateObjectComboBoxFromAction(const apc_msgs::PrimitiveAction& action);
  void updateOptionsCheckBoxesFromAction(const apc_msgs::PrimitiveAction& action);
  void updateLockedStateFromAction(const apc_msgs::PrimitiveAction& action);
  void graspCheckBoxToggled(bool grasp);


  // Pick and place widget slots.
  void runAPCButtonClicked();
  void randomizeBinsButtonClicked();
  void randomizeOrderButtonClicked();
  void reloadJsonButtonClicked();
  void nextJsonButtonClicked();
  void previousJsonButtonClicked();
  void tripleIntegralButtonClicked();

  // Pick and place widget helper slots.
  void binContentsItemClicked(QTableWidgetItem* item);
  void binContentsItemDoubleClicked(QTableWidgetItem* item);
  void processInteractiveMarkerFeedbackForItem(visualization_msgs::InteractiveMarkerFeedback& feedback);

  // Vision widget slots.
  void runDpmButtonClicked();
  void runIcpButtonClicked();
  void runVisionButtonClicked();
  void saveSnapshotButtonClicked();
  void publishShelfButtonClicked();

  // Calibration widget slots.

  // Active actions widget slots.
  void insertActionButtonClicked();
  void deleteActionButtonClicked();
  void replaceActionButtonClicked();
  void renameAllActionsButtonClicked();
  void activeActionsSelectionChanged();
  void activeActionsListClicked(const QModelIndex& index);
  void activeActionsItemClicked(QListWidgetItem* item);
  void activeActionsItemDoubleClicked(QListWidgetItem* item);

  // Stored plans widget slots.
  void activeToStoredButtonClicked();
  void storedToActiveButtonClicked();
  void storedPlansTreeClicked(const QModelIndex& index);
  void storedPlansItemClicked(QTreeWidgetItem* item, int col);
  void storedPlansItemDoubleClicked(QTreeWidgetItem* item, int col);
  // void storedPlansDatabaseComboBoxChanged();
  void savePlansButtonClicked();
  void loadPlansButtonClicked();
  void deletePlanButtonClicked();
  void storedPlansDatabaseNameChanged(const QString& text);

  // Stored Objects widget slots.
  void loadObjectsButtonClicked();
  void saveObjectsButtonClicked();
  void objectClicked(QListWidgetItem* item);
  void objectSelectionChanged();

  // Symmetry slots.
  void loadItemsForSymmetryList();
  void loadItemsForSymmetryList(const std::vector<std::string>& item_ids);
  void saveSymmetriesButtonClicked();
  void loadSymmetriesButtonClicked();
  void addSymmetryButtonClicked();
  void deleteSymmetryButtonClicked();
  void itemForSymmetryClicked(QListWidgetItem* clicked_item);
  void symmetryClicked(QListWidgetItem* clicked_item);

Q_SIGNALS:

  void itemsLoadedHook();

private:

  // Widget slots.
  void connectTeleopSlots();
  void connectTeleopHelperSlots();
  void connectPickAndPlaceSlots();
  void connectPickAndPlaceHelperSlots();
  void connectVisionSlots();
  void connectVisionHelperSlots();
  void connectCalibrationSlots();
  void connectCalibrationHelperSlots();
  void connectActiveActionsSlots();
  void connectStoredPlansSlots();
  void connectSymmetrySlots();



  // Teleoperation widget.

  KeyPoseMap computeWorldKeyPoseMap();
  std::string computeNearestFrameKey(const std::string& frame_id,
                                     const std::string& link_id,
                                     const robot_state::RobotState& robot,
                                     const KeyPoseMap& world);
  bool setStateFromPoint(robot_state::RobotState& robot,
                         const std::vector<std::string>& joint_names,
                         const trajectory_msgs::JointTrajectoryPoint& point);
  bool setStateFromIK(robot_state::RobotState& robot,
                      const std::string& link_id,
                      const std::string& group_id,
                      const Eigen::Affine3d& T_frame_world,
                      const geometry_msgs::Pose& pose_link_frame);
  bool setAttachedObjectFromAction(robot_state::RobotState& robot_state,
                                   const KeyPoseMap& world_state,
                                   const apc_msgs::PrimitiveAction& action,
                                   const int index);
  bool setStateFromAction(robot_state::RobotState& robot_state,
                          const KeyPoseMap& world_state,
                          const apc_msgs::PrimitiveAction& action,
                          const int index,
                          bool use_joint_angles_if_no_ik_solver = false);
  std::string computeNearestObjectKey(const std::string& object_id,
                                      const std::string& link_id,
                                      const robot_state::RobotState& robot,
                                      const KeyPoseMap& world);
  KeyPoseMap computeNearestFrameAndObjectKeys(const robot_state::RobotState& start,
                                              const KeyPoseMap& world,
                                              apc_msgs::PrimitivePlan& plan);
  KeyPoseMap computeNearestFrameAndObjectKeysPartial(const robot_state::RobotState& start,
                                                     const KeyPoseMap& world,
                                                     apc_msgs::PrimitivePlan& plan);
  void setStateToActionJointTrajectory(const robot_state::RobotState& state,
                                       apc_msgs::PrimitiveAction& action,
                                       int index);
  KeyPoseMap computeActionJointTrajectoryPoints(const robot_state::RobotState& start,
                                                const KeyPoseMap& world,
                                                apc_msgs::PrimitivePlan& plan);
  void setWorldKeyPoseToWorldStateMessage(const KeyPoseMap& keypose,
                                          apc_msgs::WorldState& state);
  void computeFullyConnectedPlan(const robot_state::RobotState& start,
                                 apc_msgs::PrimitivePlan& plan);
  bool doesActionMoveAnItem(const apc_msgs::PrimitiveAction& action);
  KeyPoseMap computeDenseMotionPlan(const robot_state::RobotState& start,
                                    const KeyPoseMap& world,
                                    apc_msgs::PrimitivePlan& plan,
                                    int client_index);
  void computeSmoothedPath(apc_msgs::PrimitivePlan& plan);
  apc_msgs::PrimitivePlan getPrimitivePlanFromActiveActions();
  void loadPrimitivePlanToActiveActions(const apc_msgs::PrimitivePlan& plan);
  void computePlan(apc_msgs::PrimitivePlan& plan,
                   const robot_state::RobotState start_state,
                   const KeyPoseMap& world_state,
                   int client_index = 0);
  void computePlanButtonClicked();

  void loadPlanToPreview(const moveit_msgs::RobotState& start_state,
                         const apc_msgs::PrimitivePlan& plan);
  void appendToTrajectory(trajectory_msgs::JointTrajectory& first,
                          const trajectory_msgs::JointTrajectory& second);
  void computeExecute(const apc_msgs::PrimitivePlan& plan);
  void computeExecuteButtonClicked();
  void executeActiveCallback();
  void executeFeedbackCallback(const apc_msgs::FollowPrimitivePlanFeedbackConstPtr &feedback);
  void executeDoneCallback(const actionlib::SimpleClientGoalState &state,
                           const apc_msgs::FollowPrimitivePlanResultConstPtr &result);
  void updateWorkOrderTableWidget(rapidjson::Document& doc);

  // Teleoperation widget helper.
  void updateBinContentsTableWidget(rapidjson::Document& doc);
  bool showQueryStartInteractiveMarkers();
  bool showQueryGoalInteractiveMarkers();

  // Pick and place widget.
  bool testForItemKey(const std::string& key);
  bool testForBinKey(const std::string& key);
  std::string findNearestBinToItemKey(const std::string& item_key,
                                      const KeyPoseMap& world);
  std::vector<std::string> findItemKeysInBin(const std::string& bin_id,
                                             const KeyPoseMap& world);
  std::vector<std::string> findItemKeysInBinMatchingItemID(const std::string& bin_id,
                                                           const std::string& item_id,
                                                           const KeyPoseMap& world);
  std::vector<srdf::Model::EndEffector> getEndEffectors();
  bool matchRegex(const std::string& in,
                  const std::string& expr);
  bool matchEef(const std::string& in,
                const std::string& expr);
  std::vector<apc_msgs::PrimitivePlan> findMatchingPlansAny(const std::string& database = "",
                                                            const std::string& plan_expr = ".*",
                                                            const std::string& group_expr = ".*",
                                                            const std::string& frame_expr = ".*",
                                                            const std::string& object_expr = ".*",
                                                            const std::string& eef_expr = ".*",
                                                            bool grasp = false);
  void setStateToPlanJointTrajectoryEnd(robot_state::RobotState& robot_state,
                                        const apc_msgs::PrimitivePlan& plan);
  void retrieveItemGrasps(std::vector<apc_msgs::PrimitivePlan>& item_grasps,
                          const std::string& item_id);
  void computePickAndPlaceForItem(apc_msgs::PrimitivePlan& item_plan,
                                  const std::string& bin,
                                  const std::string& item_id,
                                  const robot_state::RobotState& start,
                                  const KeyPoseMap& world);
  KeyPoseMap computeExpectedWorldState(const apc_msgs::PrimitivePlan& plan,
                                       const robot_state::RobotState& robot_state,
                                       const KeyPoseMap& world_state);
  robot_state::RobotState computeExpectedRobotState(const apc_msgs::PrimitivePlan& plan,
                                                    const robot_state::RobotState& robot_state,
                                                    const KeyPoseMap& world_state);
  void computeRunAPC(apc_msgs::PrimitivePlan& work_plan,
                     const WorkOrder& work_order,
                     const robot_state::RobotState& start,
                     const KeyPoseMap& world,
                     bool use_vision = false,
                     bool execute = false);

  WorkOrder computeWorkOrder(bool single_step);

  void computeRunAPCButtonClicked();

  // Pick and place again

  void retrieveScoreWithItemPoses(std::vector<apc_msgs::PrimitivePlan>& score_with_item_poses);
  void computeReachableScoreWithItemPoses(std::vector<apc_msgs::PrimitivePlan>& valid_scores,
                                          const std::vector<apc_msgs::PrimitivePlan>& valid_grasps,
                                          const robot_state::RobotState& start_state,
                                          const KeyPoseMap& world_state);

  // Get a list of starting poses from the database.
  void retrieveStartingPoses(std::vector<apc_msgs::PrimitivePlan>& starts);
  // Compute a list of reachable (no collisions) starting poses from
  // the current robot state.
  void computeReachableStartingPoses(std::vector<apc_msgs::PrimitivePlan>& valid_starts,
                                     const robot_state::RobotState& robot_state,
                                     const KeyPoseMap& world_state);

  void retrieveBinPoses(std::vector<apc_msgs::PrimitivePlan>& bin_poses,
                        const std::string& bin_id);
  void computeReachableBinPoses(std::vector<apc_msgs::PrimitivePlan>& valid_bins,
                                const std::string& bin_id,
                                const robot_state::RobotState& start_state,
                                const KeyPoseMap& world_state);

  void loadPlanToActiveActions(const apc_msgs::PrimitivePlan& plan);


  // Pick and place widget helper.
  void computeLoadBinContentsToScene();
  std::vector<std::string> computeLoadBinContentsToScene(const std::vector<std::pair<std::string, std::string> >& bin_contents);
  void loadKivaPodToScene();
  void loadOrderBinToScene();
  void loadWorkTableToScene();
  void loadObjectToScene(const std::string& object_key,
                         const std::string& object_model_path,
                         const Eigen::Affine3d& T_object_world);
  std::string computeItemModelPath(const std::string& item);
  std::string computeItemSceneKey(const std::string& item,
                                  const int number);
  void addItemToScene(const std::string& item_model_path,
                      const std::string& item_key,
                      const std::string& item_bin);
  void removeItemFromScene(const std::string& item_key);
  void createInteractiveMarkerForItem(const std::string& item_key);
  void updateInteractiveMarkerForItem(float wall_dt);

  // Vision widget.
  std::vector<std::string> computeObjectIdsInBinFromJson(const std::string& bin_id);
  void KinectRGBSubscriberCallback();
  void KinectPointCloudSubscriberCallback();
  void computeRunDpmButtonClicked();
  void computeRunDpm(const sensor_msgs::Image& image,
                     const std::vector<std::string>& target_object_ids,
                     const std::vector<std::string>& bin_object_ids,
                     const std::string& bin_id);
  void computeRunDpm(const sensor_msgs::Image& image,
                     const std::vector<std::string>& target_object_ids,
                     const std::vector<std::string>& bin_object_ids);
  void computeRunDpm(apc_msgs::RunDPM& run_dpm_srv);
  void computeIcp(apc_msgs::RunICP& run_icp_srv);
  void computeRunVisionButtonClicked();
  void computeRunVision();

  // Vision widget helper.
  void computePublishShelfButtonClicked();

  // Calibration widget.

  // Calibration widget helper.

  // Active actions widget.
  template< typename Message > static QByteArray serializeMessage(const Message& msg);
  template< typename Message > static Message deserializeMessage(const QByteArray& string);
  template< typename Message > static Message getMessageFromUserData(const QVariant& data);
  template< typename Message > static void setMessageToUserData(QVariant& data, const Message& msg);

  robot_state::RobotStateConstPtr getQueryStartState();
  robot_state::RobotStateConstPtr getQueryGoalState();
  robot_state::RobotStateConstPtr getQueryStartState(bool locked);
  robot_state::RobotStateConstPtr getQueryGoalState(bool locked);
  bool computeStartAndGoalEefLockedState(const apc_msgs::PrimitiveAction& action);
  void saveLockedStateToAction(apc_msgs::PrimitiveAction& action);
  void saveStartAndGoalToAction(apc_msgs::PrimitiveAction& action);
  void appendStateToAction(apc_msgs::PrimitiveAction& action, const robot_state::RobotState& state);
  std::string computeEefLink(const std::string& group);
  Eigen::Affine3d computeNearestFrameKeyPose(const std::string& frame_id,
                                             const std::string& link_id,
                                             const robot_state::RobotState& robot,
                                             const KeyPoseMap& world);
  void saveFrameToAction(apc_msgs::PrimitiveAction& action);
  void saveFrameToAction(apc_msgs::PrimitiveAction& action, const std::string& frame);
  void saveFormatToAction(apc_msgs::PrimitiveAction& action);
  void saveFormatToAction(apc_msgs::PrimitiveAction& action, const std::string& format);
  void saveFormatToPlan(apc_msgs::PrimitivePlan& plan);
  void saveFormatToPlan(apc_msgs::PrimitivePlan& plan, const std::string& format);
  void saveObjectToAction(apc_msgs::PrimitiveAction& action);
  void saveObjectToAction(apc_msgs::PrimitiveAction& action, const std::string& object_id,
                          const robot_state::RobotState& start_state, const robot_state::RobotState& goal_state);
  void saveOptionsToAction(apc_msgs::PrimitiveAction& action);
  void saveOptionsToAction(apc_msgs::PrimitiveAction& action, const std::map<std::string, bool>& options);
  void saveActionToData(const std::vector<apc_msgs::PrimitiveAction>& actions, std::vector<QVariant>& data);
  void saveActionToData(const apc_msgs::PrimitiveAction& action, QVariant& data);
  void loadActionFromData(apc_msgs::PrimitiveAction& action, const QVariant& data);
  void loadActionFromData(std::vector<apc_msgs::PrimitiveAction>& actions, const std::vector<QVariant>& data);
  void snapStateToPoint(robot_state::RobotState& state, const std::vector<std::string>& joint_names,
                        const trajectory_msgs::JointTrajectoryPoint& point);
  void snapStateToFrame(robot_state::RobotState& state, const std::string& frame, const std::string& link,
                        const geometry_msgs::Pose& pose_frame_link, const std::string& group);
  void snapStateToFrame(robot_state::RobotState& state, const Eigen::Affine3d& T_frame_world, const std::string& link,
                        const Eigen::Affine3d& T_frame_link, const std::string& group);
  void attachObjectToState(robot_state::RobotState& state, const std::string& object_id, const std::string& link_id,
                           const geometry_msgs::Pose& pose_object_link);
  void attachObjectToState(robot_state::RobotState& state, const std::string& object_id, const std::string& link_id,
                           const Eigen::Affine3d& T_object_link);
  void loadStartAndGoalFromAction(const apc_msgs::PrimitiveAction& action);
  void loadStartAndGoalFromAction(robot_state::RobotState& start, robot_state::RobotState& goal,
                                  const apc_msgs::PrimitiveAction& action);
  void loadWaypointsToDisplay(QList<QListWidgetItem*> items);
  void loadWaypointsToDisplay(QList<QTreeWidgetItem*> items);
  void loadWaypointsToDisplay(std::vector<apc_msgs::PrimitiveAction>& actions);
  void computeAttachNearestObjectToStateMatchingId(const std::string& object_id,
                                                   const std::string& group_id,
                                                   const KeyPoseMap& world_state,
                                                   robot_state::RobotState& robot_state);

  // Stored plans widget.
  void computeSavePlansButtonClicked();
  void computeLoadPlansButtonClicked();

  // MoveIt interface.
private Q_SLOTS:

  //Context tab
  void databaseConnectButtonClicked();
  void publishSceneButtonClicked();
  void planningAlgorithmIndexChanged(int index);
  void resetDbButtonClicked();
  void approximateIKChanged(int state);

  //Planning tab
  // void planButtonClicked();
  // void executeButtonClicked();
  void planAndExecuteButtonClicked();
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void allowExternalProgramCommunication(bool enable);
  void pathConstraintsIndexChanged(int index);
  void useStartStateButtonClicked();
  void useGoalStateButtonClicked();
  void onClearOctomapClicked();

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
  // void computePlanButtonClicked();
  // void computeExecuteButtonClicked();
  void computePlanAndExecuteButtonClicked();
  void computePlanAndExecuteButtonClickedDisplayHelper();
  void populateConstraintsList();
  void populateConstraintsList(const std::vector<std::string> &constr);
  void configureForPlanning(const robot_state::RobotState& start, const robot_state::RobotState& goal);
  void configureForPlanning();
  void configureWorkspace();
  void updateQueryStateHelper(robot_state::RobotState &state, const std::string &v);
  void fillStateSelectionOptions();

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
