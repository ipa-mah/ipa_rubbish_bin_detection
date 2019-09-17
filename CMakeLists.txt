cmake_minimum_required(VERSION 2.8.3)
project(ipa_rubbish_bin_detection)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++14"  )
# set catkin packages that are needed when running the code
set(catkin_RUN_PACKAGES
    actionlib
    cmake_modules
    cv_bridge
    dynamic_reconfigure
    eigen_conversions
    message_generation
    geometry_msgs
    image_transport
    roscpp
    rospy
    rosconsole
    roslaunch
    roslib
    std_msgs
    sensor_msgs
    tf
    tf_conversions
    visualization_msgs
    )
set(catkin_BUILD_PACKAGES
    ${catkin_RUN_PACKAGES}
    cmake_modules
    )
find_package(catkin REQUIRED COMPONENTS ${catkin_BUILD_PACKAGES})
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

SET(CMAKE_BUILD_TYPE Release)


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV 3 REQUIRED)
MESSAGE("OpenCV version : ${OpenCV_VERSION}")
MESSAGE("OpenCV DIR : ${OpenCV_DIR}")

generate_dynamic_reconfigure_options(
    cfg/ipa_rubbish_bin_detection.cfg
    )
## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY
    msgs
    FILES
    CylinderParams.msg
    )
generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    )
catkin_package(
    INCLUDE_DIRS common/include
    CATKIN_DEPENDS ${catkin_RUN_PACKAGES}
    DEPENDS
    LIBRARIES
    )

include_directories(
    common/include
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}

    )
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
#Common cpp files
file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/common/src/*.cpp" )
add_executable(${PROJECT_NAME}_node ros/wastebasket_detection.cpp "${SOURCES}")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )








