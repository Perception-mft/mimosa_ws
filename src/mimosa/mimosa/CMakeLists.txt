cmake_minimum_required(VERSION 3.14)
project(mimosa)

# PCL's FindFLANN module uses FLANN_ROOT. Honor it on CMake 3.27+ and avoid
# the CMP0144 developer warning while retaining compatibility with older CMake.
# PCLConfig.cmake resets its policy version, so also provide the nested scope's
# default explicitly.
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mimosa_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(config_utilities REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM 4.2.0 REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)
find_package(TBB REQUIRED)

file(GLOB_RECURSE MIMOSA_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/imu/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/radar/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry/*.cpp)

add_library(${PROJECT_NAME}_lib ${MIMOSA_SOURCES})
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC USE_OPENMP=1)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_lib
  cv_bridge geometry_msgs mimosa_msgs nav_msgs pcl_conversions rclcpp
  sensor_msgs std_msgs tf2_ros visualization_msgs)
target_link_libraries(${PROJECT_NAME}_lib
  config_utilities::config_utilities ${PCL_LIBRARIES} gtsam gtsam_unstable
  gtsam_points::gtsam_points ${OpenCV_LIBS} spdlog::spdlog OpenMP::OpenMP_CXX
  TBB::tbb)

add_executable(mimosa_node src/mimosa_node.cpp)
target_link_libraries(mimosa_node ${PROJECT_NAME}_lib)
ament_target_dependencies(mimosa_node rclcpp)

install(TARGETS ${PROJECT_NAME}_lib mimosa_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config rviz plotjuggler data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")
install(PROGRAMS
  scripts/dataset_evaluation.py scripts/topic_to_file.py
  scripts/tum_path_publisher.py scripts/factor_graph_state_report.py
  scripts/batch_process.sh scripts/run_mimosa_bag.sh
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge geometry_msgs mimosa_msgs nav_msgs pcl_conversions rclcpp
  sensor_msgs std_msgs tf2_ros visualization_msgs)
ament_package()

if(FALSE) # Legacy ROS 1/catkin build retained below for migration history.

add_compile_options(-std=c++17)
add_definitions(-DUSE_OPENMP=1)

# ##############################################################################
# Dependencies ##
# ##############################################################################

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             geometry_msgs
             ${PROJECT_NAME}_msgs
             nav_msgs
             pcl_conversions
             pcl_ros
             rosbag
             roscpp
             sensor_msgs
             std_msgs
             tf2_ros
             visualization_msgs
)

find_package(config_utilities REQUIRED)
find_package(OpenCV REQUIRED)
find_package(
  GTSAM
  4.2.0
  REQUIRED
)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)

# Print dependency info
message(STATUS "PCL Version: ${PCL_VERSION}, Path: ${PCL_DIR}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}, Path: ${OpenCV_DIR}")
message(STATUS "GTSAM Version: ${GTSAM_VERSION}, Path: ${GTSAM_DIR}")
message(STATUS "gtsam_points Path: ${gtsam_points_DIR}")

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  ${PROJECT_NAME}_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Collect all source files from sensor folders
set(SRC_CPP_FOLDERS
    imu
    graph
    lidar
    radar
    odometry
)
set(SOURCES)
foreach(FOLDER ${SRC_CPP_FOLDERS})
  file(
    GLOB_RECURSE
    SRC_CPP_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${FOLDER}/*.cpp
  )
  list(
    APPEND
    SOURCES
    ${SRC_CPP_FILES}
  )
endforeach()

# Common libraries used by all targets
set(COMMON_LIBS
    ${catkin_LIBRARIES}
    config_utilities::config_utilities
    ${PCL_LIBRARIES}
    gtsam
    gtsam_unstable
    gtsam_points::gtsam_points
    ${OpenCV_LIBS}
    spdlog::spdlog
)

# Strict compiler warnings for executables
set(STRICT_WARNINGS
    $<$<CXX_COMPILER_ID:MSVC>:/W4
    /WX>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall
    -Wextra
    -Wpedantic
    -Werror>
)

# Library
add_library(${PROJECT_NAME}_lib ${SOURCES})
add_dependencies(
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_options(
  ${PROJECT_NAME}_lib
  PRIVATE -pthread
          -fexceptions
          ${OpenMP_CXX_FLAGS}
)
target_link_libraries(${PROJECT_NAME}_lib ${COMMON_LIBS})
install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Executables
set(EXECUTABLES node rosbag)
foreach(EXEC ${EXECUTABLES})
  add_executable(${PROJECT_NAME}_${EXEC} src/${PROJECT_NAME}_${EXEC}.cpp)
  add_dependencies(
    ${PROJECT_NAME}_${EXEC}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(
    ${PROJECT_NAME}_${EXEC}
    ${PROJECT_NAME}_lib
    ${COMMON_LIBS}
  )
  target_compile_options(
    ${PROJECT_NAME}_${EXEC}
    PRIVATE ${STRICT_WARNINGS}
            -pthread
            -fexceptions
            ${OpenMP_CXX_FLAGS}
  )
  install(TARGETS ${PROJECT_NAME}_${EXEC}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h*"
)

set(EXTRA_INSTALL_DIRS config launch)
foreach(directory IN LISTS ARGV)
  install(DIRECTORY ${directory}
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

set(SCRIPTS batch_process.sh evaluate.sh)
foreach(script IN LISTS SCRIPTS)
  install(PROGRAMS scripts/${script}
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()
endif()
