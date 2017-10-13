# CMakeLists.txt
#
# Copyright (c) 2013 Wunderkammer Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(steered_wheel_base_controller)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  angles
  control_toolbox
  controller_interface
  geometry_msgs
  hardware_interface
  kdl_parser
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  tf
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/steered_wheel_base_controller.cpp 
  src/joint_base.cpp
  src/joint_types/pos_joint.cpp
  src/joint_types/vel_joint.cpp
  src/joint_types/pid_joint.cpp
  src/wheel.cpp
  src/util.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
	plugins
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
