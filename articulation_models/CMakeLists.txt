cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${Eigen_DEFINITIONS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(
	${PROJECT_NAME} 
	src/utils.cpp
	src/models/generic_model.cpp
	src/models/prismatic_model.cpp
	src/models/rigid_model.cpp
	src/models/pca_gp_model.cpp
	src/models/factory.cpp
	src/models/rotational_model.cpp
)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(model_learner_msg
	src/model_learner_msg.cpp)

rosbuild_add_executable(model_learner_srv
	src/model_learner_srv.cpp)

rosbuild_add_executable(model_learner_prior
	src/model_learner_prior.cpp)

target_link_libraries(model_learner_msg ${PROJECT_NAME})
target_link_libraries(model_learner_srv ${PROJECT_NAME})
target_link_libraries(model_learner_prior ${PROJECT_NAME})
