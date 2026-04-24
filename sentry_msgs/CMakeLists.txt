cmake_minimum_required(VERSION 3.5)
project(sentry_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

add_message_files(
    FILES
    TargetInfo.msg
    GimbalCommand.msg
    GimbalState.msg
    SentryStatus.msg
    FireCommand.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)
