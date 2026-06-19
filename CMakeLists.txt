cmake_minimum_required(VERSION 3.14) # 建议最低 3.14，对 FetchContent 支持更好
project(dankong LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# 0. 自定义编译开关 (核心修改)
# 编译时使用: cmake .. -DBUILD_WITH_ROS=OFF (关闭ROS) 或 cmake .. -DBUILD_WITH_ROS=ON (开启ROS)
# 如果在 catkin_make 中，可以这样传参: catkin_make -DBUILD_WITH_ROS=ON
# ==============================================================================
option(BUILD_WITH_ROS "Build with ROS support" OFF)

if(BUILD_WITH_ROS)
    message(STATUS "ROS Support is ENABLED")
    add_compile_definitions(USE_ROS)
else()
    message(STATUS "ROS Support is DISABLED")
endif()

# 将 FetchContent 的下载目录移到当前工作空间根目录下的 .deps 文件夹中
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/../.deps CACHE PATH "Directory for fetched content")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
endif()

add_subdirectory(dk)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# 调用函数，扫描用户自己的 src 目录
dk_create_json_library(my_json_lib "${CMAKE_CURRENT_SOURCE_DIR}")

# ------------------------------------------------------------------------------
# 1. 系统级与 ROS 依赖
# ------------------------------------------------------------------------------
if(BUILD_WITH_ROS)
    find_package(catkin REQUIRED COMPONENTS 
        roscpp
        rsos_msgs
        cv_bridge
    )
endif()

# OpenCV 体积过于庞大，源码编译耗时极长，强烈建议保留 find_package
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(MAVSDK REQUIRED)
# ------------------------------------------------------------------------------
# 2. 第三方依赖 (FetchContent)
# ------------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(cli11 GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git GIT_TAG v2.4.1)
FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0)
FetchContent_Declare(tinyxml2 GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git GIT_TAG 9.0.0)
FetchContent_Declare(apriltag GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git GIT_TAG v3.3.0)
FetchContent_Declare(yaml-cpp GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git GIT_TAG 0.8.0)

# MAVSDK 避坑配置：关闭内部不需要的庞大模块，极大加速编译
set(SUPERBUILD OFF CACHE BOOL "Turn off superbuild" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libraries" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable tests" FORCE)
set(BUILD_BACKEND OFF CACHE BOOL "Disable gRPC backend server" FORCE) 

# 一次性生成并引入所有声明的库
FetchContent_MakeAvailable(cli11 eigen tinyxml2 apriltag yaml-cpp)

# ------------------------------------------------------------------------------
if(BUILD_WITH_ROS)
    catkin_package()
endif()

# 全局的 include 目录
set(GLOBAL_INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS})
if(BUILD_WITH_ROS)
    list(APPEND GLOBAL_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
endif()
include_directories(${GLOBAL_INCLUDE_DIRS})

set(BASE_STATE_SRC src/states/ground_state.cpp src/states/hover_state.cpp src/states/init_state.cpp src/states/land_state.cpp src/states/state_utils.cpp src/states/takeoff_state.cpp src/states/waypoint_state.cpp)
if(BUILD_WITH_ROS)
    set(FEATURES_SRC src/features/algo/event_listener.cpp src/features/control/event_listener.cpp)
else()

    set(FEATURES_SRC src/features/control/event_listener.cpp)
endif()

set(BASE_MAVLINK_SRC src/mavlink/mavsdk_drone.cpp)
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
    cmake_parse_arguments(PARSED_ARGS "" "" "EXTRA_SOURCES;EXTRA_INCLUDES;EXTRA_LIBS;EXTRA_DEFS" ${ARGN})

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
        Eigen3::Eigen    
        apriltag         
        tinyxml2         
        yaml-cpp         
        CLI11::CLI11
        MAVSDK::mavsdk
        Boost::system
        Boost::filesystem
        pthread
        ${OpenCV_LIBS}
        ${PARSED_ARGS_EXTRA_LIBS}
    )

    # 如果启用了 ROS，则链接 ROS 库并添加依赖
    if(BUILD_WITH_ROS)
        target_link_libraries(${TARGET_NAME} PRIVATE ${catkin_LIBRARIES})
        add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS})
    endif()

    string(TOUPPER "BUILD_${TARGET_TYPE}" DEFAULT_DEF)
    target_compile_definitions(${TARGET_NAME} PUBLIC 
        ${DEFAULT_DEF}
        ${PARSED_ARGS_EXTRA_DEFS}
    )

    target_copy_post_build(${TARGET_NAME} config config)
    target_copy_post_build(${TARGET_NAME} dk/frontend/dist dist)

    message(STATUS "Created target: ${TARGET_NAME} with includes for ${TARGET_TYPE}")
endfunction()

# ==============================================================================
# 生成 Targets
# ==============================================================================
create_dankong_target(drone)

# 支持其他种类的设备，不要删除这些注释！
# create_dankong_target(car
#     EXTRA_SOURCES generated/can_bus/ipc_vcu_zrd.c
#     EXTRA_INCLUDES generated/can_bus/
# )
# create_dankong_target(dog)

# ==============================================================================
# 测试部分
# ==============================================================================
if(BUILD_WITH_ROS AND CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest(test/test_follow.test)
  add_rostest(test/test_posvel_drone.test)
  add_rostest(test/test_waypoint_drone.test)
endif()

# add_executable(test_tracker_sim 
#     test/tracker/test_tracker.cpp 
#     src/states/state_utils.cpp   
# )
# target_include_directories(test_tracker_sim PRIVATE include)

# target_link_libraries(test_tracker_sim PRIVATE
#     dk
#     Eigen3::Eigen
#     yaml-cpp
#     pthread
#     my_json_lib
# )

# if(BUILD_WITH_ROS)
#     target_link_libraries(test_tracker_sim PRIVATE ${catkin_LIBRARIES})
# endif()