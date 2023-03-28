cmake_minimum_required(VERSION 3.12)

project(SRM-IC-2023)
set(EXECUTABLE_NAME srm-ic-2023)  # 可执行文件名

set(MVS_PATH /opt/MVS)  # MVS 相机驱动目录
set(DH_CAM_PATH /opt/Galaxy_camera)  # Galaxy 相机驱动目录

set(CMAKE_CXX_STANDARD 20)
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_compile_options(-O0)
elseif (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-Ofast -march=native -flto)
endif ()
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIR})
include_directories(${MVS_PATH}/include)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/aarch64/*.so)
endif ()
include_directories(${DH_CAM_PATH}/inc)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/x86_64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/armv8/*.so)
endif ()
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    include_directories(/usr/include/x86_64-linux-gnu)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    include_directories(/usr/include/aarch64-linux-gnu)
endif ()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)
file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/*/*.c*)
add_executable(${EXECUTABLE_NAME} main.cpp ${SRC} modules/controller/controller-sentry/controller-sentry.cpp)
target_link_libraries(
        ${EXECUTABLE_NAME}
        ${OpenCV_LIBS}
        ${GLOG_LIBRARIES}
        gflags
        ${MVS_LIBS}
        ${DH_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
)
