# set(CMAKE_CXX_FLAGS "-Wshadow -Wreturn-type -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -rdynamic")

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit/motion_planning_rviz_plugin/motion_planning_display.h
  include/moveit/motion_planning_rviz_plugin/motion_planning_frame.h
  include/moveit/motion_planning_rviz_plugin/motion_planning_frame_impl.hpp

  include/apc/eigen.h
  include/apc/planning.h
  include/apc/exception.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

qt4_wrap_ui(UIC_FILES
  src/ui/motion_planning_rviz_plugin_frame.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(backward-cpp)

# Plugin Source
set(SOURCE_FILES 
  src/motion_planning_frame.cpp
  src/motion_planning_frame_context.cpp
  src/motion_planning_frame_planning.cpp
  src/motion_planning_frame_objects.cpp
  src/motion_planning_frame_scenes.cpp
  src/motion_planning_frame_states.cpp
  src/motion_planning_display.cpp
  src/motion_planning_frame_manipulation.cpp

  # Pick and place widget.
  src/motion_planning_widget_pick_and_place.cpp
  src/motion_planning_widget_pick_and_place_slots.cpp
  src/motion_planning_widget_pick_and_place_helper.cpp
  src/motion_planning_widget_pick_and_place_helper_slots.cpp

  # Teleoperation widget.
  src/motion_planning_widget_teleop.cpp
  src/motion_planning_widget_teleop_slots.cpp
  src/motion_planning_widget_teleop_helper.cpp
  src/motion_planning_widget_teleop_helper_slots.cpp

  # Vision widget.
  src/motion_planning_widget_vision.cpp
  src/motion_planning_widget_vision_slots.cpp
  src/motion_planning_widget_vision_helper.cpp
  src/motion_planning_widget_vision_helper_slots.cpp

  # Calibration widget.
  src/motion_planning_widget_calibration.cpp
  src/motion_planning_widget_calibration_slots.cpp
  src/motion_planning_widget_calibration_helper.cpp
  src/motion_planning_widget_calibration_helper_slots.cpp

  # Active actions widget.
  src/motion_planning_widget_active_actions.cpp
  src/motion_planning_widget_active_actions_slots.cpp

  # Stored plans widget.
  src/motion_planning_widget_stored_plans.cpp
  src/motion_planning_widget_stored_plans_slots.cpp

  # APC helpers
  src/apc_eigen.cpp
  src/apc_exception.cpp
  src/apc_planning.cpp

  # signal
  backward-cpp/backward.cpp
)

set(MOVEIT_LIB_NAME moveit_motion_planning_rviz_plugin)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES} ${backward_ENABLE})
target_link_libraries(${MOVEIT_LIB_NAME}_core
  moveit_rviz_plugin_render_tools
  moveit_planning_scene_rviz_plugin_core
  ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} bfd)

add_backward(${MOVEIT_LIB_NAME}_core)

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
