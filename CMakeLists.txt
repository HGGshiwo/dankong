cmake_minimum_required(VERSION 3.12)
project(dankong)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
endif()

add_subdirectory(dk)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# 调用函数，扫描用户自己的 src 目录
dk_create_json_library(my_json_lib "${CMAKE_CURRENT_SOURCE_DIR}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  mavros
  mavros_msgs
  rsos_msgs
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

find_library(TINYXML2_LIBRARY NAMES tinyxml2)
find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
if(NOT TINYXML2_LIBRARY OR NOT TINYXML2_INCLUDE_DIR)
    message(FATAL_ERROR "tinyxml2 not found")
endif()

include(FetchContent)
FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        v2.4.1 # 推荐去 GitHub 查阅并使用最新稳定版本
)
FetchContent_MakeAvailable(cli11)

catkin_package()

# 全局的 include 目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIR}
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

# 将 config/a 文件复制到可执行文件旁，并改名为 config
# target_copy_post_build(dankong_dog_node "config/a" "config")
# 如果 config/a 是一个目录，它的内容会被拷贝到可执行文件旁的 config 文件夹下
# target_copy_post_build(dankong_dog_node "config/a" "config")
# 拷贝 a.txt 到可执行文件下的 config 目录中（最终路径: /.../config/a.txt）
# target_copy_post_build(dankong_dog_node "config/a.txt" "config/")
# 将 a.txt 和 b.txt 一起复制到可执行文件旁的 config 目录中
# target_copy_post_build(dankong_dog_node "config/a.txt" "config/b.txt" "config")

function(target_copy_post_build TARGET_NAME)
    # 获取所有的额外参数
    set(ALL_ARGS ${ARGN})
    list(LENGTH ALL_ARGS ARGS_LEN)

    if(ARGS_LEN LESS 2)
        message(FATAL_ERROR "用法: target_copy_post_build(<target_name> <src1> [<src2> ...] <dest_relative_path>)")
    endif()

    # 提取最后一个参数作为相对目标路径
    math(EXPR LAST_IDX "${ARGS_LEN} - 1")
    list(GET ALL_ARGS ${LAST_IDX} DEST_REL)
    list(REMOVE_AT ALL_ARGS ${LAST_IDX})

    # 自动拼接前缀：可执行文件所在目录 $<TARGET_FILE_DIR:${TARGET_NAME}>
    if(DEST_REL STREQUAL "" OR DEST_REL STREQUAL ".")
        set(DEST_BASE "$<TARGET_FILE_DIR:${TARGET_NAME}>")
    else()
        set(DEST_BASE "$<TARGET_FILE_DIR:${TARGET_NAME}>/${DEST_REL}")
    endif()

    # 获取要复制的源文件/目录数量，用于做智能判断
    list(LENGTH ALL_ARGS SRC_COUNT)

    foreach(ITEM IN LISTS ALL_ARGS)
        set(SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}")
        
        if(IS_DIRECTORY "${SRC_PATH}")
            # 如果是目录，直接将源目录内的所有内容复制到目标路径下
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${SRC_PATH}"
                "${DEST_BASE}"
            )
        else()
            # 如果是文件，我们要像 Linux 的 cp 命令一样智能：
            # 1. 如果用户传了多个文件，或者路径以 / 结尾，则保留原文件名放入该目录中。
            # 2. 如果只有一个文件且没有以 / 结尾，就直接将文件重命名为用户指定的名字（比如 config/a -> config）。
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
# 语法: create_dankong_target(
#           TARGET_TYPE         # 必填: 类型名，例如 drone 或 dog
#           [EXTRA_SOURCES ...] # 可选: 额外的 .cpp 文件
#           [EXTRA_INCLUDES ...]# 可选: 额外的 include 路径
#           [EXTRA_LIBS ...]    # 可选: 额外的第三方库
#           [EXTRA_DEFS ...]    # 可选: 额外的编译宏定义
#       )
# ==============================================================================
function(create_dankong_target TARGET_TYPE)
    # 解析传入的可选参数
    cmake_parse_arguments(
        PARSED_ARGS
        "" # options
        "" # one_value_keywords
        "EXTRA_SOURCES;EXTRA_INCLUDES;EXTRA_LIBS;EXTRA_DEFS" # multi_value_keywords
        ${ARGN}
    )

    # 定义当前 Target 的名称 (例如: dankong_drone_node)
    set(TARGET_NAME ${PROJECT_NAME}_${TARGET_TYPE}_node)

    # 1. 创建可执行文件 (包含基础源码 + 额外传入的源码)
    add_executable(${TARGET_NAME} 
        ${BASE_STATE_SRC} 
        ${BASE_MAVLINK_SRC} 
        ${BASE_NODE_SRC}
        ${FEATURES_SRC}
        ${PARSED_ARGS_EXTRA_SOURCES}
    )

    # 2. 设置 Include 目录 (特定 Target 的 include 优先级最高)
    # 使用 BEFORE 关键字确保 target 专属的 include 排在全局 include 前面
    target_include_directories(${TARGET_NAME} BEFORE PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/targets/${TARGET_TYPE}
        include
        ${PARSED_ARGS_EXTRA_INCLUDES}
    )

    # 3. 链接库 (基础依赖 + 额外传入的库)
    target_link_libraries(${TARGET_NAME} PRIVATE
        dk              # 链接框架
        my_json_lib     # 链接刚才生成的 JSON 宏库
        Geographic
        atomic  
        Eigen3::Eigen
        apriltag::apriltag
        CLI11::CLI11
        ${TINYXML2_LIBRARY}
        ${catkin_LIBRARIES}
        ${PARSED_ARGS_EXTRA_LIBS}
        ${OpenCV_LIBS}
    )

    # 4. 编译宏定义 (自动生成类似 BUILD_DRONE 的宏，并支持追加宏)
    string(TOUPPER "BUILD_${TARGET_TYPE}" DEFAULT_DEF)
    target_compile_definitions(${TARGET_NAME} PUBLIC 
        ${DEFAULT_DEF}
        ${PARSED_ARGS_EXTRA_DEFS}
    )

    # 5. 添加依赖关系
    add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS})

    # 6. Post-build 文件/文件夹拷贝逻辑
    target_copy_post_build(${TARGET_NAME} config config)
    target_copy_post_build(${TARGET_NAME} dk/frontend/dist dist)

    # target_copy_post_build(${TARGET_NAME} config/${TARGET_TYPE} config)

    message(STATUS "Created target: ${TARGET_NAME} with includes for ${TARGET_TYPE}")
endfunction()
# ==============================================================================


# ==============================================================================
# 使用函数生成不同的 Targets
# ==============================================================================

# 1. 生成 Drone (只有基础配置)
create_dankong_target(drone)

# 2. 生成 Dog (基础配置)
# create_dankong_target(dog)


# ==============================================================================
# 测试部分
# ==============================================================================
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  # dog测试
  # add_rostest(test/test_dog.test)
  # add_rostest(test/test_posvel_dog.test)
  # add_rostest(test/test_waypoint_dog.test )

  # drone测试
  add_rostest(test/test_follow.test)
  # add_rostest(test/test_pland.test)
  # add_rostest(test/test_planner.test)
  add_rostest(test/test_posvel_drone.test)
  add_rostest(test/test_waypoint_drone.test)
endif()


add_executable(test_tracker_sim 
    test/tracker/test_tracker.cpp 
    src/states/state_utils.cpp   # 因为你的 Tracker 用了 angle_ctrl_
)
target_include_directories(test_tracker_sim PRIVATE 
    include 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(test_tracker_sim PRIVATE
    dk
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    pthread
    my_json_lib
)