cmake_minimum_required (VERSION 2.6)
project(rovio_lib)
SET(CMAKE_CXX_FLAGS "-O0 -Wno-long-long -g -ggdb -std=c++11 -fPIC")
add_definitions(-D_GLIBCXX_USE_C99 -llog)

##################### Find, include, and-llog compile library #####################

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
#Require Boost
FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
include_directories ( ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
src/Camera.cpp
src/FeatureCoordinates.cpp
src/FeatureDistance.cpp
include/rovio/RovioManger.hpp
include/rovio/Camera.hpp
include/rovio/Duration.hpp
include/rovio/exceptions.hpp
include/rovio/FeatureCoordinates.hpp
include/rovio/FeatureDistance.hpp
include/rovio/FeatureManager.hpp
include/rovio/FeatureStatistics.hpp
include/rovio/featureTracker.hpp
include/rovio/FilterStates.hpp
include/rovio/ImagePyramid.hpp
include/rovio/ImgUpdate.hpp
include/rovio/ImuPrediction.hpp
include/rovio/MultiCamera.hpp
include/rovio/MultilevelPatch.hpp
include/rovio/MultilevelPatchAlignment.hpp
include/rovio/Patch.hpp
include/rovio/PoseUpdate.hpp
include/rovio/RobocentricFeatureElement.hpp
include/rovio/RovioFilter.hpp
include/rovio/RovioManger.hpp
include/rovio/RovioScene.hpp
include/rovio/Scene.hpp
include/rovio/Time.hpp
include/rovio/ZeroVelocityUpdate.hpp
)
target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    libyaml-cpp.a
)

add_executable(rovio_src 
src/rovio_run.cpp
src/Duration.cpp
src/Time.cpp
)
target_link_libraries(rovio_src
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    libyaml-cpp.a
)



#add_executable(rovio_rosbag_loader src/rovio_rosbag_loader.cpp)
#target_link_libraries(rovio_rosbag_loader ${PROJECT_NAME}
#    ${PROJECT_NAME}
#    ${OpenCV_LIBS}
#Q    ${Boost_LIBRARIES}
#    /home/jimmy/project/rovio/libs/libyaml-cpp.a
#)

#add_executable(feature_tracker_node src/feature_tracker_node.cpp)
#target_link_libraries(feature_tracker_node ${PROJECT_NAME})
