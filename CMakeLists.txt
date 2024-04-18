cmake_minimum_required(VERSION 3.10)
project(myprogram)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")  # 关闭所有警告信息
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")  # 关闭所有警告信息

include_directories(
    ./src/ziyan_io/include
    ./src/map/include
    ./src/planner/include
    ./thirdparty/eigen3
    ./thirdparty/ompl/src
)

set(MAP_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/map/src)
set(PLANNER_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/planner/src)
set(ZIYAN_IO_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/ziyan_io/src)

file(GLOB Target_SRCS
    ${MAP_SOURCES_DIR}/*.cpp
    ${PLANNER_SOURCES_DIR}/*.cpp
    ${ZIYAN_IO_SOURCES_DIR}/*.cpp
)

option(BUILD_Target "Building Target" ON)

if(BUILD_Target)
    message("Building Target")
    add_executable(myprogram main.cpp ${Target_SRCS})
    target_link_libraries(myprogram
        ${PROJECT_SOURCE_DIR}/thirdparty/ompl/build/zzzz/share/libompl.so
    )
endif()