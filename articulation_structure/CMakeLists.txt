cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(graph_learner  src/structure_learner_st.cpp  src/structure_learner_base.cpp  src/hogman_wrapper.cpp src/structs.cpp src/utils.cpp )

rosbuild_add_executable(structure_learner_srv  src/structure_learner_srv.cpp src/ArticulatedObject.cpp src/hogman_wrapper.cpp src/structs.cpp src/utils.cpp )

#rosbuild_add_executable(graph_learner src/graph_learner.cpp src/structure_learner_base.cpp src/utils.cpp src/structs.cpp src/predict_pose.cpp src/hogman_wrapper.cpp)

#rosbuild_add_executable(structure_learner_mrf src/structure_learner_mrf.cpp)

rosbuild_add_openmp_flags(structure_learner_srv)
