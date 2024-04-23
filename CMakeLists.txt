cmake_minimum_required(VERSION 3.12)
project(myplanner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")  # 关闭所有警告信息
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")  # 关闭所有警告信息

# Set the output directory for the build executables and libraries
SET(planner_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${planner_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${planner_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${planner_LIBRARY_OUTPUT_DIRECTORY})


set(PLANNER_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/src)
message(${PLANNER_SOURCES_DIR})

find_package(Eigen3 3.1.0 REQUIRED)

set(FIND_OMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
message(${FIND_OMPL_DIR})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${FIND_OMPL_DIR}/ompl/src
)

file(GLOB LIB_SRCS
    ${PLANNER_SOURCES_DIR}/*.cpp
)

add_library(${PROJECT_NAME} SHARED ${LIB_SRCS})
target_link_libraries(${PROJECT_NAME}
    ${FIND_OMPL_DIR}/ompl/build/src/ompl/libompl.so
)

# 添加可执行文件 main.cpp
add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_NAME})