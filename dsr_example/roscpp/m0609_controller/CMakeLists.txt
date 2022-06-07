cmake_minimum_required(VERSION 3.0.2)
project(m0609_controller)

add_definitions(-D__XENO__=true)

#set(ENV{XENOMAI_ROOT_DIR} /usr/xenomai)

# Set policy for LINK_INTERFACE
#cmake_policy(SET CMP0022 NEW)
add_compile_options(-std=c++14)

# Needed Modules
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(${PROJECT_SOURCE_DIR}/cmake-modules/CMakeFunctions.cmake)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
  sensor_msgs
  dsr_msgs
)
find_package(
  Xenomai
  REQUIRED
  )

find_package(Eigen3 REQUIRED NO_MODULES)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  roscpp
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
  sensor_msgs
  dsr_msgs
  LIBRARIES ${PROJECT_NAME}
)


include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

#https://stackoverflow.com/questions/8774593/cmake-link-to-external-library
# Your-external "mylib", add GLOBAL if the imported library is located in directories above the current.
add_library( drfl SHARED IMPORTED )
# You can define two import-locations: one for debug and one for release.
set_target_properties( drfl PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/library/libDRFL.a )

add_library( poconet SHARED IMPORTED )
set_target_properties( poconet PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/library/libPocoNet.so.62 )

add_library( pocofoundation SHARED IMPORTED )
set_target_properties( pocofoundation PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/library/libPocoFoundation.so.62 )

add_library(${PROJECT_NAME}
  src/PD_Gravity.cpp
  include/DRFC.h
  include/DRFL.h
  include/DRFLEx.h
  include/DRFS.h
  include/Trajectory.cpp
  include/LieOperator.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#target_link_libraries(doosan_api_example drfl poconet pocofoundation ${Xenomai_NATIVE_DL_FLAGS})
#target_include_directories(doosan_api_example PUBLIC ${Xenomai_NATIVE_INCLUDE_DIRS})
#set_target_properties(doosan_api_example PROPERTIES COMPILE_FLAGS ${Xenomai_NATIVE_DEFINITIONS})
