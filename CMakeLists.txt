cmake_minimum_required(VERSION 3.5)

# OPTION (USE_OpenMP "Use OpenMP" ON)
# IF(USE_OpenMP)
#   FIND_PACKAGE(OpenMP)
#   IF(OPENMP_FOUND)
#     SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#   ENDIF()
# ENDIF()

# set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS}, ${OPENMP_C_FLAGS})
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fopenmp")#编译器启用openmp
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}, ${OPENMP_CXX_FLAGS})

project(emplanner)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#设置运行时目标文件（exe dll）的输出位置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#设置存档时目标文件（lib a）的输出位置
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 定义子目录src，用以递归的调用src中的MakeLists.txt
add_subdirectory(config)
add_subdirectory(reference_line)
add_subdirectory(EMPlanner)
add_subdirectory(plot)
add_subdirectory(routing)
add_subdirectory(localization)
add_subdirectory(perception)


include_directories(${PROJECT_SOURCE_DIR}#工程目录
                    "usr/include"#系统目录
                    "usr/local/include"#安装文件目录
                    "/usr/include/python3.6m"#python目录
                    "usr/local/include/eigen3/Eigen/"
)

# Create the executable
add_executable(path_plan_test path_plan_test.cc)

link_directories(
    "usr/local/lib"
    "usr/local/lib64"
    "usr/lib"
    ${PROJECT_BINARY_DIR}/lib
)

message(${PROJECT_BINARY_DIR})

target_include_directories(path_plan_test PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
                            "/usr/include/python3.6m"#python目录
                             SYSTEM ${EIGEN3_INCLUDE_DIR}

)

#link_directories( ${subprojects_SOURCE_DIR}/lib )

# Link the static library from subproject1 using it's alias sub::lib1
# Link the header only library from subproject2 using it's alias sub::lib2
# This will cause the include directories for that target to be added to this project

#如何调用第三方库
#1.findpackage()查找 库需要具有cmake文件
#2.直接添加link_include 和 库文件 

# <LibaryName>_FOUND
# <LibaryName>_INCLUDE_DIR or <LibaryName>_INCLUDES <LibaryName>_LIBRARY or <LibaryName>_LIBRARIES
find_package(Eigen3 REQUIRED)
# message(${EIGEN3_INCLUDE_DIRS})
#file(GLOB GLOG_LIBRARIES /usr/lib/x86_64-linux-gnu/libglog.so)

find_package(OsqpEigen)


target_link_libraries(path_plan_test PUBLIC
    reference_line_provider
    plot
    routing
    localization
    perception 
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

#include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

