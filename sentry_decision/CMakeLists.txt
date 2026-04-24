cmake_minimum_required(VERSION 3.5)
project(sentry_decision)

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
        super_planner
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sentry_decision
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs super_planner
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${super_planner_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp)
add_library(sentry_decision STATIC ${srcs})
target_include_directories(sentry_decision PUBLIC include)
target_link_libraries(sentry_decision PUBLIC ${THIRD_PARTY})

add_executable(decision_node
        src/decision_node.cpp
)
target_link_libraries(decision_node
        sentry_decision
        ${THIRD_PARTY}
)
