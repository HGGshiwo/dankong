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
)
find_package(Eigen3 REQUIRED)

find_library(TINYXML2_LIBRARY NAMES tinyxml2)
find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
if(NOT TINYXML2_LIBRARY OR NOT TINYXML2_INCLUDE_DIR)
    message(FATAL_ERROR "tinyxml2 not found")
endif()

catkin_package()

# 全局的 include 目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
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
        ${TINYXML2_INCLUDE_DIR}
        ${PARSED_ARGS_EXTRA_INCLUDES}
    )

    # 3. 链接库 (基础依赖 + 额外传入的库)
    target_link_libraries(${TARGET_NAME} PRIVATE
        dk              # 链接框架
        my_json_lib     # 链接刚才生成的 JSON 宏库
        Geographic
        atomic  
        Eigen3::Eigen
        ${TINYXML2_LIBRARY}
        ${catkin_LIBRARIES}
        ${PARSED_ARGS_EXTRA_LIBS}
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
    set(ITEMS_TO_COPY config dk/frontend/dist)
    foreach(ITEM IN LISTS ITEMS_TO_COPY)
        if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}")
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>/${ITEM}"
            )
        else()
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>/${ITEM}"
            )
        endif()
    endforeach()

    message(STATUS "Created target: ${TARGET_NAME} with includes for ${TARGET_TYPE}")
endfunction()
# ==============================================================================


# ==============================================================================
# 使用函数生成不同的 Targets
# ==============================================================================

# 1. 生成 Drone (只有基础配置)
create_dankong_target(drone)

# 2. 生成 Dog (基础配置)
create_dankong_target(dog)


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