cmake_minimum_required(VERSION 3.14) # 建议最低 3.14，对 FetchContent 支持更好
project(dankong)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
endif()

add_subdirectory(dk)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# 调用函数，扫描用户自己的 src 目录
dk_create_json_library(my_json_lib "${CMAKE_CURRENT_SOURCE_DIR}")

# ------------------------------------------------------------------------------
# 1. 系统级与 ROS 依赖 (保留 find_package)
# ------------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  mavros
  mavros_msgs
  rsos_msgs
  cv_bridge
)
# OpenCV 体积过于庞大，源码编译耗时极长，强烈建议保留 find_package
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------
# 2. 第三方依赖 (改用 FetchContent 自动拉取源码构建)
# ------------------------------------------------------------------------------
include(FetchContent)

# CLI11 (Header-only)
FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        v2.4.1
)

# Eigen3 (Header-only)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)

# TinyXML2
FetchContent_Declare(
  tinyxml2
  GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
  GIT_TAG        9.0.0
)

# AprilTag
FetchContent_Declare(
  apriltag
  GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
  GIT_TAG        v3.3.0
)

# yaml-cpp
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG        0.8.0 # 稳定版本
)

# 一次性生成并引入所有声明的库（需要 CMake 3.14+）
FetchContent_MakeAvailable(cli11 eigen tinyxml2 apriltag yaml-cpp)

# ------------------------------------------------------------------------------
catkin_package()

# 全局的 include 目录 (移除了 Eigen 和 tinyxml2，因为 target 会自动传播它们)
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

# 基础的源文件（所有 target 共享）
set(BASE_STATE_SRC
  src/states/ground_state.cpp
  src/states/hover_state.cpp
  src/states/init_state.cpp
  src/states/land_state.cpp
  src/states/state_utils.cpp
  src/states/takeoff_state.cpp
  src/states/waypoint_state.cpp
)

set(FEATURES_SRC
  src/features/algo/event_listener.cpp
  src/features/control/event_listener.cpp
)

set(BASE_MAVLINK_SRC src/mavlink/mavros.cpp)
set(BASE_NODE_SRC src/dankong_node.cpp)

# ==============================================================================
# 文件/目录拷贝工具函数
# ==============================================================================
function(target_copy_post_build TARGET_NAME)
    set(ALL_ARGS ${ARGN})
    list(LENGTH ALL_ARGS ARGS_LEN)

    if(ARGS_LEN LESS 2)
        message(FATAL_ERROR "用法: target_copy_post_build(<target_name> <src1> [<src2> ...] <dest_relative_path>)")
    endif()

    math(EXPR LAST_IDX "${ARGS_LEN} - 1")
    list(GET ALL_ARGS ${LAST_IDX} DEST_REL)
    list(REMOVE_AT ALL_ARGS ${LAST_IDX})

    if(DEST_REL STREQUAL "" OR DEST_REL STREQUAL ".")
        set(DEST_BASE "$<TARGET_FILE_DIR:${TARGET_NAME}>")
    else()
        set(DEST_BASE "$<TARGET_FILE_DIR:${TARGET_NAME}>/${DEST_REL}")
    endif()

    list(LENGTH ALL_ARGS SRC_COUNT)

    foreach(ITEM IN LISTS ALL_ARGS)
        set(SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}")
        
        if(IS_DIRECTORY "${SRC_PATH}")
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${SRC_PATH}"
                "${DEST_BASE}"
            )
        else()
            if(SRC_COUNT GREATER 1 OR DEST_REL MATCHES "/$")
                get_filename_component(ITEM_NAME "${ITEM}" NAME)
                set(FINAL_DEST_FILE "${DEST_BASE}/${ITEM_NAME}")
                set(FINAL_DEST_DIR "${DEST_BASE}")
            else()
                set(FINAL_DEST_FILE "${DEST_BASE}")
                get_filename_component(DEST_REL_DIR "${DEST_REL}" DIRECTORY)
                if(DEST_REL_DIR STREQUAL "")
                    set(FINAL_DEST_DIR "$<TARGET_FILE_DIR:${TARGET_NAME}>")
                else()
                    set(FINAL_DEST_DIR "$<TARGET_FILE_DIR:${TARGET_NAME}>/${DEST_REL_DIR}")
                endif()
            endif()

            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory "${FINAL_DEST_DIR}"
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SRC_PATH}"
                "${FINAL_DEST_FILE}"
            )
        endif()
    endforeach()
endfunction()

# ==============================================================================
# 定义生成目标节点的函数
# ==============================================================================
function(create_dankong_target TARGET_TYPE)
    cmake_parse_arguments(
        PARSED_ARGS
        "" 
        "" 
        "EXTRA_SOURCES;EXTRA_INCLUDES;EXTRA_LIBS;EXTRA_DEFS"
        ${ARGN}
    )

    set(TARGET_NAME ${PROJECT_NAME}_${TARGET_TYPE}_node)

    add_executable(${TARGET_NAME} 
        ${BASE_STATE_SRC} 
        ${BASE_MAVLINK_SRC} 
        ${BASE_NODE_SRC}
        ${FEATURES_SRC}
        ${PARSED_ARGS_EXTRA_SOURCES}
    )

    target_include_directories(${TARGET_NAME} BEFORE PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/targets/${TARGET_TYPE}
        include
        ${PARSED_ARGS_EXTRA_INCLUDES}
    )

    target_link_libraries(${TARGET_NAME} PRIVATE
        dk              
        my_json_lib     
        Geographic
        atomic  
        Eigen3::Eigen    # FetchContent 自动生成的 Alias 目标
        apriltag         # FetchContent 生成的 apriltag 目标
        tinyxml2         # FetchContent 生成的 tinyxml2 目标 (代替 ${TINYXML2_LIBRARY})
        yaml-cpp         # FetchContent 生成的 yaml-cpp 目标
        CLI11::CLI11
        ${catkin_LIBRARIES}
        ${PARSED_ARGS_EXTRA_LIBS}
        ${OpenCV_LIBS}
    )

    string(TOUPPER "BUILD_${TARGET_TYPE}" DEFAULT_DEF)
    target_compile_definitions(${TARGET_NAME} PUBLIC 
        ${DEFAULT_DEF}
        ${PARSED_ARGS_EXTRA_DEFS}
    )

    add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS})

    target_copy_post_build(${TARGET_NAME} config config)
    target_copy_post_build(${TARGET_NAME} dk/frontend/dist dist)

    message(STATUS "Created target: ${TARGET_NAME} with includes for ${TARGET_TYPE}")
endfunction()

# ==============================================================================
# 使用函数生成不同的 Targets
# ==============================================================================
create_dankong_target(drone)
# create_dankong_target(dog)

# ==============================================================================
# 测试部分
# ==============================================================================
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  # dog测试
  # add_rostest(test/test_dog.test)
  
  # drone测试
  add_rostest(test/test_follow.test)
  add_rostest(test/test_posvel_drone.test)
  add_rostest(test/test_waypoint_drone.test)
endif()

add_executable(test_tracker_sim 
    test/tracker/test_tracker.cpp 
    src/states/state_utils.cpp   
)
target_include_directories(test_tracker_sim PRIVATE 
    include 
)
target_link_libraries(test_tracker_sim PRIVATE
    dk
    Eigen3::Eigen
    yaml-cpp
    ${catkin_LIBRARIES}
    pthread
    my_json_lib
)