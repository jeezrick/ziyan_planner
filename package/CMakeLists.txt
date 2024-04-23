cmake_minimum_required(VERSION 3.12)
project(myplanner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")  # 关闭所有警告信息
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")  # 关闭所有警告信息

set(FIND_DIR /home/dwang/code/workspace/ziyan_pathplanner/)
set(PLANNER_SOURCES_DIR ${FIND_DIR}/package/src)
message(${FIND_DIR})
message(${PLANNER_SOURCES_DIR})

find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
    ${FIND_DIR}/package
    ${FIND_DIR}/package/pathplanner
    # ${FIND_DIR}/thirdparty/eigen3
    ${EIGEN3_INCLUDE_DIR}
    ${FIND_DIR}/thirdparty/ompl/src
)

file(GLOB Target_SRCS
    ${PLANNER_SOURCES_DIR}/*.cpp
)

option(BUILD_Target "Building Target" ON)

# add_subdirectory(thirdparty/ompl)
# if(BUILD_Target)
#     message("Building Target")
#     add_executable(myprogram main.cpp ${Target_SRCS})
#     target_link_libraries(myprogram
#         ompl
#     )
# endif()

if(BUILD_Target)
    message("Building Target")
    add_executable(${PROJECT_NAME} main.cpp ${Target_SRCS})
    target_link_libraries(${PROJECT_NAME}
        ${FIND_DIR}/thirdparty/ompl/build/src/ompl/libompl.so
    )
endif() 
