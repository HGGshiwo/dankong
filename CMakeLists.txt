cmake_minimum_required(VERSION 3.14) # 建议最低 3.14，对 FetchContent 支持更好
project(dankong LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# ==============================================================================
# 0. 自定义编译开关 (支持 非ROS, ROS1, ROS2)
# 编译时使用: 
#   非ROS: cmake .. -DROS_SUPPORT=NONE
#   ROS 1: catkin_make -DROS_SUPPORT=ROS1 (或 cmake .. -DROS_SUPPORT=ROS1)
#   ROS 2: colcon build --cmake-args -DROS_SUPPORT=ROS2
# ==============================================================================
set(ROS_SUPPORT "AUTO" CACHE STRING "Set ROS support level: AUTO, NONE, ROS1, ROS2")

# 如果设置为 AUTO，则通过环境变量自动检测
if(ROS_SUPPORT STREQUAL "AUTO")
    if(DEFINED ENV{ROS_VERSION})
        if("$ENV{ROS_VERSION}" STREQUAL "1")
            set(ROS_SUPPORT "ROS1")
        elseif("$ENV{ROS_VERSION}" STREQUAL "2")
            set(ROS_SUPPORT "ROS2")
        else()
            set(ROS_SUPPORT "NONE")
        endif()
    else()
        set(ROS_SUPPORT "NONE")
    endif()
endif()

if(ROS_SUPPORT STREQUAL "ROS1")
    message(STATUS "ROS Support is ENABLED [ROS 1]")
    add_compile_definitions(USE_ROS USE_ROS1)
elseif(ROS_SUPPORT STREQUAL "ROS2")
    message(STATUS "ROS Support is ENABLED [ROS 2]")
    add_compile_definitions(USE_ROS USE_ROS2)
else()
    message(STATUS "ROS Support is DISABLED [Non-ROS]")
    set(ROS_SUPPORT "NONE")
endif()

# 将 FetchContent 的下载目录移到当前工作空间根目录下的 .deps 文件夹中
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/../.deps CACHE PATH "Directory for fetched content")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
endif()

# 1. 在进入第三方库之前，主动接管 spdlog 的寻找
if(ROS_SUPPORT STREQUAL "ROS2")
    find_package(spdlog REQUIRED)
    
    set(DUMMY_SPDLOG_DIR "${CMAKE_CURRENT_BINARY_DIR}/dummy_spdlog")
    file(MAKE_DIRECTORY "${DUMMY_SPDLOG_DIR}")
    file(WRITE "${DUMMY_SPDLOG_DIR}/CMakeLists.txt" "project(dummy_spdlog)\n")
    
    # 3. 官方重定向：强制告诉 FetchContent "别去网上拉代码了，读取我这个空工程就行"
    set(FETCHCONTENT_SOURCE_DIR_SPDLOG "${DUMMY_SPDLOG_DIR}" CACHE PATH "Bypass FetchContent for spdlog" FORCE)
endif()

add_subdirectory(dk)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# 调用函数，扫描用户自己的 src 目录
dk_create_json_library(my_json_lib "${CMAKE_CURRENT_SOURCE_DIR}")

# ------------------------------------------------------------------------------
# 1. 系统级与 ROS 依赖
# ------------------------------------------------------------------------------
if(ROS_SUPPORT STREQUAL "ROS1")
    find_package(catkin REQUIRED COMPONENTS 
        roscpp
        rsos_msgs
        cv_bridge
    )
elseif(ROS_SUPPORT STREQUAL "ROS2")
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
endif()

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(MAVSDK REQUIRED)
find_package(OpenSSL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)
# ------------------------------------------------------------------------------
# 2. 第三方依赖 (FetchContent)
# ------------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(cli11 GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git GIT_TAG v2.4.1)
FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0)
FetchContent_Declare(tinyxml2 GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git GIT_TAG 9.0.0)
FetchContent_Declare(apriltag GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git GIT_TAG v3.3.0)
FetchContent_Declare(yaml-cpp GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git GIT_TAG 0.8.0)


set(YAML_CPP_INSTALL OFF CACHE BOOL "Disable yaml-cpp install" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Disable yaml-cpp tests" FORCE)

# 下载并使得库可用
FetchContent_MakeAvailable(cli11 tinyxml2 apriltag yaml-cpp)

# 3. 单独处理 Eigen (因为它是 Header-Only，直接拉取并提取路径，不执行其 CMakeLists)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
    
    # 模拟出一个名为 Eigen3::Eigen 的目标，完美兼容你下方的 target_link_libraries
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# ------------------------------------------------------------------------------
# ROS Package 配置 (在生成 Target 之前)
# ------------------------------------------------------------------------------
if(ROS_SUPPORT STREQUAL "ROS1")
    catkin_package()
endif()

# 全局的 include 目录
set(GLOBAL_INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS})

if(ROS_SUPPORT STREQUAL "ROS1")
    list(APPEND GLOBAL_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
endif()
# ROS 2 通常通过 ament_target_dependencies 管理 include，无需加入全局
include_directories(${GLOBAL_INCLUDE_DIRS})

set(BASE_STATE_SRC src/states/ground_state.cpp src/states/hover_state.cpp src/states/init_state.cpp src/states/land_state.cpp src/states/state_utils.cpp src/states/takeoff_state.cpp src/states/waypoint_state.cpp)

if(ROS_SUPPORT STREQUAL "ROS1")
    if(ROBOT STREQUAL "drone")
        set(FEATURES_SRC 
            src/features/algo/event_listener.cpp 
            src/features/control/event_listener.cpp 
            src/features/mavlink/event_listener.cpp)
    else()
        set(FEATURES_SRC 
            src/features/control/event_listener.cpp 
            src/features/mavlink/event_listener.cpp)
    endif()
else()
    set(FEATURES_SRC 
        src/features/control/event_listener.cpp 
        src/features/mavlink/event_listener.cpp)
endif()

set(BASE_MAVLINK_SRC src/mavlink/mavsdk_drone.cpp)
set(BASE_NODE_SRC src/dankong_node.cpp)

# ==============================================================================
# 文件/目录拷贝工具函数 (保持不变)
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
        generated
        ${PARSED_ARGS_EXTRA_INCLUDES}
    )

    target_link_libraries(${TARGET_NAME}
        dk              
        my_json_lib     
        ${GeographicLib_LIBRARIES}
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
        OpenSSL::SSL      # 链接 SSL 库 (负责 TLS/SSL 协议层)
        OpenSSL::Crypto   # 链接 Crypto 库 (负责底层的加密算法)
        ${OpenCV_LIBS}
        ${PARSED_ARGS_EXTRA_LIBS}
    )

    # 针对不同 ROS 版本的依赖注入
    if(ROS_SUPPORT STREQUAL "ROS1")
        target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
        add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS})
    elseif(ROS_SUPPORT STREQUAL "ROS2")
        ament_target_dependencies(${TARGET_NAME}
            rclcpp
            cv_bridge
            nav_msgs
            geometry_msgs
        )
        # ROS 2 需要显式的 install 规则才能在 colcon workspace 中找到它
        install(TARGETS ${TARGET_NAME}
            DESTINATION lib/${PROJECT_NAME}
        )
        install(DIRECTORY 
            ${CMAKE_CURRENT_SOURCE_DIR}/config
            ${CMAKE_CURRENT_SOURCE_DIR}/dk/frontend/dist
            DESTINATION lib/${PROJECT_NAME}
        )
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
set(ROBOT "drone" CACHE STRING "Select hardware target to build (drone, car, dog, all)")

if(ROBOT STREQUAL "drone")
    create_dankong_target(drone)
elseif(ROBOT STREQUAL "car")
    create_dankong_target(car
        EXTRA_SOURCES generated/can_bus/ipc_vcu_zrd.c
        EXTRA_INCLUDES generated/can_bus/
    )
elseif(ROBOT STREQUAL "dog")
    create_dankong_target(dog)
elseif(ROBOT STREQUAL "go2")
    create_dankong_target(go2)
else()
    message(FATAL_ERROR "Invalid ROBOT: '${ROBOT}'. Options are: drone, car, dog, all.")
endif()

# ==============================================================================
# ROS 2 Package 声明 (必须在所有 install 生成结束后)
# ==============================================================================
if(ROS_SUPPORT STREQUAL "ROS2")
    ament_package()
endif()

# ==============================================================================
# 测试部分
# ==============================================================================
if(ROS_SUPPORT STREQUAL "ROS1" AND CATKIN_ENABLE_TESTING)
    # find_package(rostest REQUIRED)
    # add_rostest(test/test_follow.test)
    # add_rostest(test/test_posvel_drone.test)
    # add_rostest(test/test_waypoint_drone.test)
elseif(ROS_SUPPORT STREQUAL "ROS2" AND BUILD_TESTING)
    # 如果未来需要添加 ROS 2 对应的测试，可以在这里补充 (例如 ament_add_gtest 或 launch_testing)
    # find_package(ament_cmake_gtest REQUIRED)
    # ament_add_gtest(...)
endif()