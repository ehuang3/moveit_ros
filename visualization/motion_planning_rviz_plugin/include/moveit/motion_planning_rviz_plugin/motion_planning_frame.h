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
#include <moveit/motion_planning_rviz_plugin/exception.h>

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

  ros::ServiceClient _compute_dense_motion_client;

  ros::ServiceClient _run_vision_client;

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

  // Teleoperation widget.

  /**
   * @brief Extracts the frames and object poses from the current
   * planning scene. Each pose is associated with the unique
   * identifying key of its frame or object.
   *
   * @return  The current frame/object poses.
   */
  KeyPoseMap computeWorldKeyPoseMap();

  /**
   * @brief Certain frame and object IDs are ambiguous, such as "bin"
   * or "oreo_mega_stuf" in the presence of duplicates. This function
   * computes the unique frame key of the frame nearest to the input
   * link given the robot and world states.
   *
   * @param frame_id  The frame ID. May be ambiguous, like "bin".
   * @param link_id  The robot link we evaluate nearest to.
   * @param robot  The robot state.
   * @param world  The world state.
   *
   * @return  The frame key nearest to the link.
   */
  std::string computeNearestFrameKey(const std::string& frame_id,
                                     const std::string& link_id,
                                     const robot_state::RobotState& robot,
                                     const KeyPoseMap& world);

  /**
   * @brief Sets the input joint angles to the robot state. The ith
   * joint position in 'point' will be set to the joint with the ith
   * name in 'joint_names'.
   *
   * @param robot  The robot state to modify.
   * @param joint_names  The joints to set.
   * @param point  The joint angle values to set.
   *
   * @return  True on success.
   */
  bool setStateFromPoint(robot_state::RobotState& robot,
                         const std::vector<std::string>& joint_names,
                         const trajectory_msgs::JointTrajectoryPoint& point);

  /**
   * @brief Sets the robot state using IK so that the given link is
   * located at the given world to link transform.
   *
   * @param robot  The robot state to set.
   * @param link_id  The link to snap.
   * @param group_id  The group to perform IK with.
   * @param T_frame_world  The transform from world to frame.
   * @param pose_link_frame  The transform from frame to link.
   *
   * @return  True on success.
   */
  bool setStateFromIK(robot_state::RobotState& robot,
                      const std::string& link_id,
                      const std::string& group_id,
                      const Eigen::Affine3d& T_frame_world,
                      const geometry_msgs::Pose& pose_link_frame);

  /**
   * @brief Sets the robot state using IK so that the given link is
   * located at the given world to link transform. If no IK solver
   * exists for the group the joints can be optionally set using the
   * joint angles in the action's joint angle trajectory.
   *
   * @param robot_state  A reference to the robot state to set.
   * @param world_state  The world state.
   * @param action  The action to set states with.
   * @param pose_index  The index into the eef pose trajectory of the pose to IK to.
   * @param use_joint_angles_if_no_ik_solver  The transform from frame to link.
   *
   * @return  True on success.
   */
  bool setStateFromIkUsingAction(robot_state::RobotState& robot_state,
                                 const KeyPoseMap& world_state,
                                 const apc_msgs::PrimitiveAction& action,
                                 const int pose_index,
                                 bool use_joint_angles_if_no_ik_solver = false);

  bool setAttachedObjectFromAction(robot_state::RobotState& robot_state,
                                   const KeyPoseMap& world_state,
                                   const apc_msgs::PrimitiveAction& action,
                                   const int index);

  bool setStateFromAction(robot_state::RobotState& robot_state,
                          const KeyPoseMap& world_state,
                          const apc_msgs::PrimitiveAction& action,
                          const int index,
                          bool use_joint_angles_if_no_ik_solver = false);

  /**
   * @brief Certain frame and object IDs are ambiguous, such as "bin"
   * or "oreo_mega_stuf" in the presence of duplicates. This function
   * computes the unique object key nearest to the input link given
   * the robot and world states.
   *
   * @param object_id  The object ID.
   * @param link_id  The robot link we evaluate nearest to.
   * @param robot  The robot state.
   * @param world  The world state.
   *
   * @return The object key nearest to the link.
   */
  std::string computeNearestObjectKey(const std::string& object_id,
                                      const std::string& link_id,
                                      const robot_state::RobotState& robot,
                                      const KeyPoseMap& world);

  /**
   * @brief Certain frame and object IDs are ambiguous, such as "bin"
   * or "oreo_mega_stuf" in the presence of duplicates. This function
   * computes the nearest frame and object keys (which are unique)
   * given a starting state and a plan to follow. The computed keys
   * are stored in the input plan. The input plan must not have any
   * keys filled out already.
   *
   * @param start  The starting state of the robot.
   * @param world  The starting frame/object poses.
   * @param plan  The plan we will compute frame and object keys for.
   *
   * @return  The frame/object poses at the end of plan execution.
   */
  KeyPoseMap computeNearestFrameAndObjectKeys(const robot_state::RobotState& start,
                                              const KeyPoseMap& world,
                                              apc_msgs::PrimitivePlan& plan);

  /**
   * @brief Certain frame and object IDs are ambiguous, such as "bin"
   * or "oreo_mega_stuf" in the presence of duplicates. This function
   * computes the nearest frame and object keys (which are unique)
   * given a starting state and a plan to follow. The computed keys
   * are stored in the input plan. The input plan is allowed to have
   * keys partially filled out already.
   *
   * @param start  The starting state of the robot.
   * @param world  The starting frame/object poses.
   * @param plan  The plan we will compute frame and object keys for.
   *
   * @return  The frame/object poses at the end of plan execution.
   */
  KeyPoseMap computeNearestFrameAndObjectKeysPartial(const robot_state::RobotState& start,
                                                     const KeyPoseMap& world,
                                                     apc_msgs::PrimitivePlan& plan);

  /**
   * @brief This function sets the input robot state to the 'index' point
   * action's joint trajectory. Only the joint angles of joints named
   * in the action joint trajectory are appended.
   *
   * @param state  The input robot state.
   * @param action  The action to modify.
   * @param index  The index of the joint trajectory to modify.
   */
  void setStateToActionJointTrajectory(const robot_state::RobotState& state,
                                       apc_msgs::PrimitiveAction& action,
                                       int index);

  /**
   * @brief Over the course of a plan, objects may move when actions
   * are taken. Given an input plan with frame and object keys, this
   * function tracks object motion over each action and computes the
   * correct joint angle trajectory points relative to those object
   * displacements. The trajetory points are written into the input
   * plan.
   *
   * @param start  The starting state of the robot.
   * @param world  The starting frame/object poses.
   * @param plan  The plan we will compute joint trajectory points
   *              for. Note that frame and object keys must exist.
   *
   * @return  The frame/object poses at the end of plan execution.
   */
  KeyPoseMap computeActionJointTrajectoryPoints(const robot_state::RobotState& start,
                                                const KeyPoseMap& world,
                                                apc_msgs::PrimitivePlan& plan);


  /**
   * @brief Copies frame/object keys and poses to world state
   * message. Note that this function will attempt to extract
   * frame/object IDs from keys and will throw an exception on failure
   * to do so.
   *
   * @param keypose  The keys and poses to copy.
   * @param state  The output world state.
   */
  void setWorldKeyPoseToWorldStateMessage(const KeyPoseMap& keypose,
                                          apc_msgs::WorldState& state);


  /**
   * @brief Inserts additional actions into the plan if needed to
   * connect previous states to next states.
   *
   * @param plan  Plan to process.
   */
  void computeFullyConnectedPlan(const robot_state::RobotState& start,
                                 apc_msgs::PrimitivePlan& plan);


  /**
   * @brief Checks whether the action moves an object.
   *
   * @param action  The action to check.
   *
   * @return  True if the action moves an object.
   */
  bool doesActionMoveAnItem(const apc_msgs::PrimitiveAction& action);

  /**
   * @brief Compute a dense motion plan given the input sparse
   * plan. The dense motion plan is computed using the input sparse
   * joint angle trajectories. If needed, additional actions are
   * inserted to connect previous action goal states to the next
   * action start states. This function makes ROS services calls to
   * the external trajopt planner.
   *
   * @param start  The starting state of the robot.
   * @param world  The starting frame/object poses.
   * @param plan  The input plan.
   *
   * @return  The frame/object poses at the end of plan execution.
   */
  KeyPoseMap computeDenseMotionPlan(const robot_state::RobotState& start,
                                    const KeyPoseMap& world,
                                    apc_msgs::PrimitivePlan& plan);

  /**
   * @brief Computes a smooth path for all joint trajectories in the
   * plan while respecting velocity and acceleration limits.
   *
   * @param plan  The plan to smooth.
   */
  void computeSmoothedPath(apc_msgs::PrimitivePlan& plan);

  /**
   * @brief Returns the current active actions (in the GUI) as a
   * primitive plan.
   *
   * @return The aggregated actions.
   */
  apc_msgs::PrimitivePlan getPrimitivePlanFromActiveActions();

  /**
   * @brief Load the input plan into the active actions list (in the
   * GUI). This function is only safe to call in the Qt main thread.
   *
   * @param plan  The plan to load.
   */
  void loadPrimitivePlanToActiveActions(const apc_msgs::PrimitivePlan& plan);

  /**
   * @brief This function takes the currently active actions and
   * computes a dense trajectory plan for them. The plan is reloaded
   * into the active actions when it is finished generating.
   */
  void computePlanButtonClicked();

  void loadPlanToPreview(const moveit_msgs::RobotState& start_state,
                         const apc_msgs::PrimitivePlan& plan);
  void appendToTrajectory(trajectory_msgs::JointTrajectory& first,
                          const trajectory_msgs::JointTrajectory& second);
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

  /**
   * @brief Test if the input is an object key.
   *
   * @param key  The key to test.
   *
   * @return  True if the input is an object key.
   */
  bool testForItemKey(const std::string& key);

  /**
   * @brief Test if the input is an bin key.
   *
   * @param key  The key to test.
   *
   * @return  True if the input is an bin key.
   */
  bool testForBinKey(const std::string& key);

  /**
   * @brief Find the nearest bin to the input item.
   *
   * @param item_key  The target item.
   * @param world  The world state.
   *
   * @return  The frame ID of the nearest bin.
   */
  std::string findNearestBinToItemKey(const std::string& item_key,
                                      const KeyPoseMap& world);

  /**
   * @brief Returns the keys if items nearest to the input bin.
   *
   * @param bin_id  The target bin.
   * @param world  The world state.
   *
   * @return  The items in the bin.
   */
  std::vector<std::string> findItemKeysInBin(const std::string& bin_id,
                                             const KeyPoseMap& world);

  /**
   * @brief Returns the keys of items in the bin that match the input item ID.
   *
   * @param bin_id  The name of the bin to search in, e.g. "bin_A".
   * @param item_id  The ID of the item to find.
   * @param world  The world state.
   *
   * @return  A vector of matched item keys in the bin.
   */
  std::vector<std::string> findItemKeysInBinMatchingItemID(const std::string& bin_id,
                                                           const std::string& item_id,
                                                           const KeyPoseMap& world);

  /**
   * @brief Returns a list of the robot model's end-effectors.
   *
   * @return  A list of end-effectors.
   */
  std::vector<srdf::Model::EndEffector> getEndEffectors();

  /**
   * @brief Returns whether the input string 'in' matches the regex of 'expr'.
   *
   * @param in  The string to match to the regex.
   * @param expr  The regex to match the string with.
   *
   * @return  True if match.
   */
  bool matchRegex(const std::string& in,
                  const std::string& expr);

  /**
   * @brief Returns whether the input string matches the regex.
   *
   * @param in  String to match.
   * @param expr  Expression to match with.
   *
   * @return  True if match.
   */
  bool matchEef(const std::string& in,
                const std::string& expr);

  /**
   * @brief Find all plans in the database for which at least one of
   * their actions matches regex expressions for each setting.
   *
   * @param database  The name of the database to load. TODO
   * @param group_expr  The group regex to match.
   * @param frame_expr  The frame regex to match.
   * @param object_expr  The object regex to match.
   * @param eef_expr  The end-effector regex to match.
   * @param grasp  The grasp setting to match
   *
   * @return  All the plans that satisfy the above conditions.
   */
  std::vector<apc_msgs::PrimitivePlan> findMatchingPlansAny(const std::string& database,
                                                            const std::string& group_expr,
                                                            const std::string& frame_expr,
                                                            const std::string& object_expr,
                                                            const std::string& eef_expr,
                                                            bool grasp);

  /**
   * @brief Compute a plan for pick and place of the input item.
   *
   * @param bin  The bin ID the item is located in.
   * @param item_id  The item ID of the item to pick.
   * @param start  The robot's start state.
   * @param world  The world's start state.
   *
   * @return  A dense joint trajectory plan for the item.
   */
  apc_msgs::PrimitivePlan computePickAndPlaceForItem(const std::string& bin,
                                                     const std::string& item_id,
                                                     const robot_state::RobotState& start,
                                                     const KeyPoseMap& world);

  KeyPoseMap computeExpectedWorldState(const apc_msgs::PrimitivePlan& plan,
                                       const robot_state::RobotState& robot_state,
                                       const KeyPoseMap& world_state);

  robot_state::RobotState computeExpectedRobotState(const apc_msgs::PrimitivePlan& plan,
                                                    const robot_state::RobotState& robot_state,
                                                    const KeyPoseMap& world_state);

  void computeRunAPC(const WorkOrder& work_order,
                     const robot_state::RobotState& start,
                     const KeyPoseMap& world,
                     apc_msgs::PrimitivePlan& plan,
                     bool use_vision = false,
                     bool execute = false);

  WorkOrder computeWorkOrder(bool single_step);

  void computeRunAPCButtonClicked();


  // Pick and place widget helper.
  void computeLoadBinContentsToScene();
  std::vector<std::string> computeLoadBinContentsToScene(const std::vector<std::pair<std::string, std::string> >& bin_contents);
  void loadKivaPodToScene();
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
  std::string computeNearestBin(std::string group, const robot_state::RobotState& state);
  std::string computeNearestObject(const std::string& object, const std::string& group, const robot_state::RobotState& state);
  Eigen::Affine3d computeFrame(const std::string& frame);
  Eigen::Affine3d computeNearestFrame(const std::string& frame, const std::string& group, const robot_state::RobotState& state);
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
