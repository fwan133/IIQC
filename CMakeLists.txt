cmake_minimum_required(VERSION 3.0.2)
project(IIQC)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        gazebo_msgs
        image_transport
        cv_bridge
        )

find_package(cv_bridge REQUIRED)
if(cv_bridge_FOUND)
    MESSAGE(STATUS "Found CV_BRIDGE with include dir: " ${cv_bridge_INCLUDE_DIRS})
    MESSAGE(STATUS "Found CV_BRIDGE with libs dir: " ${cv_bridge_LIBRARIES})
endif ()


LIST(APPEND CMAKE_MODULE_PATH /usr/local/include/g2o/cmake_modules)
find_package(G2O REQUIRED)
MESSAGE(STATUS ${G2O_FOUND})
if (G2O_FOUND)
    set(G2O_LIBS
            ${G2O_CORE_LIBRARY}
            ${G2O_STUFF_LIBRARY}
            ${G2O_TYPES_SBA}
            ${G2O_TYPES_SLAM3D}
            ${G2O_TYPES_SIM3}
            ${G2O_SOLVER_CSPARSE}
            ${G2O_SOLVER_CSPARSE_EXTENSION}
            )
    MESSAGE(STATUS "Found G2O with include directory:" ${G2O_INCLUDE_DIR})
    MESSAGE(STATUS "Found G2O with libs:" ${G2O_LIBS})
endif ()


find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    MESSAGE(STATUS "Found Eigen with include dir: " ${EIGEN_INCLUDE_DIR})
endif ()


find_package(octomap REQUIRED)
if (octomap_FOUND)
    MESSAGE(STATUS "Found octomap with include dir: " ${octomap_INCLUDE_DIRS})
    MESSAGE(STATUS "Found octomap with libs: " ${octomap_LIBS})
endif ()

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    MESSAGE(STATUS "Found OpenCV include dir: " ${OpenCV_INCLUDE_DIRS})
    MESSAGE(STATUS "Found OpenCV libs: " ${OpenCV_LIBS})
ENDIF()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(Ceres REQUIRED)
if (Ceres_FOUND)
    MESSAGE(STATUS "Found Ceres with include dir: " ${Ceres_INCLUDE_DIRS})
    MESSAGE(STATUS "Found Ceres with lib dir: " ${Ceres_LIBRARIES})
endif ()

FIND_PACKAGE( PCL REQUIRED)
if(PCL_FOUND)
    MESSAGE(STATUS "The PCL is found in: " ${PCL_INCLUDE_DIRS})
    MESSAGE(STATUS "The PCL is found with lib dir: " ${PCL_LIBRARIES})
endif ()
# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )


find_package(Pangolin REQUIRED)
if (Pangolin_FOUND)
    SET(Pangolin_INCLUDE_DIRS /usr/local/include/pangolin)
    MESSAGE(STATUS "The Pangolin is found with include dir: " ${Pangolin_INCLUDE_DIRS})
    MESSAGE(STATUS "The Pangolin is found with lib dir: " ${Pangolin_LIBRARIES})
endif ()


add_subdirectory(./ThirdParty/GeographicLib/)
#add_subdirectory(./ThirdParty/SfM)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES data_quality_check
        CATKIN_DEPENDS
        roscpp
        rosbag
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        std_msgs
        nav_msgs
        gazebo_msgs
        mavros_msgs
        tf
        cv_bridge
        image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ./ThirdParty/GeographicLib/include/
        usr/local/lib/opencv4/3rdparty Libraries
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIR}
        ${octomap_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

## Declare a C++ library


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(HelloWorld src/HelloWorld.cpp)

add_executable(Test_image_blur_detection test/image_blur_detection.cpp)
target_link_libraries(Test_image_blur_detection
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(Test_cast_ray
        test/Test_castray.cpp
        src/IQM_octree.cpp)
target_link_libraries(Test_cast_ray
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${octomap_LIBRARIES}
        )

add_executable(Test_IQM_img test/Test_IQM_image.cpp)
target_link_libraries(Test_IQM_img
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(Test_IQMOcTree test/Test_IQM_octree.cpp src/IQM_octree.cpp)
target_link_libraries(Test_IQMOcTree
        ${octomap_LIBRARIES}
        )

add_executable(Test_IQMOcMap test/Test_IQM_map.cpp src/IQM_octree.cpp)
target_link_libraries(Test_IQMOcMap
        ${octomap_LIBRARIES}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(Test_mission_manager test/test_mission_manager.cpp)
target_link_libraries(Test_mission_manager
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        libGeographiccc
        )

add_executable(Test_SfM test/Test_SfM.cpp)
target_link_libraries(Test_SfM
        ${OpenCV_LIBS}
        )

add_executable(Test_orb test/test_orb_match.cpp)
target_link_libraries(Test_orb
        ${OpenCV_LIBS}
        )

add_executable(Test_waypoint_mission test/Test_waypoint_mission.cpp)
target_link_libraries(Test_waypoint_mission
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(Test_coordinate_converter test/Test_coordinate_converter.cpp
        ThirdParty/GeographicLib/src/LocalCartesian.cpp
        ThirdParty/GeographicLib/src/Geocentric.cpp
        ThirdParty/GeographicLib/src/Math.cpp)
target_link_libraries(Test_waypoint_mission
        ${catkin_LIBRARIES}
        )

add_executable(Test_Eigen test/Test_Eigen.cpp)

add_executable(Test_gimbal test/Test_gimbal.cpp)
target_link_libraries(Test_gimbal
        ${catkin_LIBRARIES}
        )

add_executable(Test_frame_collection test/Test_frame_collection.cpp)
target_link_libraries(Test_frame_collection
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(Test_opencv_sfm test/Test_opencv_sfm.cpp)
target_link_libraries(Test_opencv_sfm
        ${OpenCV_LIBS}
        ${CERES_LIBS}
        )

add_executable(Test_point_cloud_alignment test/Test_point_cloud_alignment.cpp)
target_link_libraries(Test_point_cloud_alignment
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        )

add_executable(Test_sequential_sfm test/Test_sequential_sfm.cpp)
target_link_libraries(Test_sequential_sfm
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${G2O_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        pthread)

add_executable(Test_cpp test/Test_cpp.cpp)
target_link_libraries(Test_cpp)

add_executable(Test_pangolin test/Test_pangolin.cpp)
target_link_libraries(Test_pangolin
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS})

add_executable(Test_cv_bridge test/Test_cv_bridge.cpp)
target_link_libraries(Test_cv_bridge
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

add_executable(Test_reconstructor test/Test_reconstructor.cpp)
target_link_libraries(Test_reconstructor
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${G2O_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        pthread)

add_executable(Test_incremental_sfm test/Test_incremental_sfm.cpp)
target_link_libraries(Test_incremental_sfm
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${G2O_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        pthread)

add_executable(Test_cv_project test/Test_cv_project.cpp)
target_link_libraries(Test_cv_project
        ${OpenCV_LIBS}
    )

add_executable(Test_camera_pose test/Test_camera_pose.cpp)
target_link_libraries(Test_camera_pose
        ${Pangolin_LIBRARIES}
        )

add_executable(Test_icp_solver test/Test_ICP_Solver.cpp)
target_link_libraries(Test_icp_solver
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(Test_solve_SIM3 test/Test_solve_SIM3.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_IIQC.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
