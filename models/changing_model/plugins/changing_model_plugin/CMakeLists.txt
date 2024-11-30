cmake_minimum_required(VERSION 3.0)
project(texture_change_plugin)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})

# プラグインのソースファイル名を指定
add_library(texture_change_plugin SHARED texture-change-plugin.cpp)
target_link_libraries(texture_change_plugin ${GAZEBO_LIBRARIES})

