cmake_minimum_required(VERSION 3.5)
project(sentry_control)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-Werror=return-type)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-DUSE_ROS1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        message_generation
        can_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sentry_control
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs can_msgs
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp)
add_library(sentry_control STATIC ${srcs})
target_include_directories(sentry_control PUBLIC include)
target_link_libraries(sentry_control ${catkin_LIBRARIES})

add_executable(control_node
        src/control_node.cpp
)
target_link_libraries(control_node
        sentry_control
        ${catkin_LIBRARIES}
)
