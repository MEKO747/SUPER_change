cmake_minimum_required(VERSION 3.5)
project(sentry_vision)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-Werror=return-type)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-DUSE_ROS1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        image_transport
        message_generation
        cv_bridge
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        image_transport
        cv_bridge
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sentry_vision
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs image_transport cv_bridge
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp)
add_library(sentry_vision STATIC ${srcs})
target_include_directories(sentry_vision PUBLIC include)
target_link_libraries(sentry_vision ${catkin_LIBRARIES})

add_executable(vision_node
        src/vision_node.cpp
)
target_link_libraries(vision_node
        sentry_vision
        ${catkin_LIBRARIES}
)
