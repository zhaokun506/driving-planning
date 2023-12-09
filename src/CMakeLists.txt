cmake_minimum_required(VERSION 3.5)


project(emplanner)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#设置运行时目标文件（exe dll）的输出位置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#设置存档时目标文件（lib a）的输出位置
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#头文件包含的系统工程目录
include_directories(${PROJECT_SOURCE_DIR}#工程目录
)

#包含使用的第三方库
include($ENV{PROJECT_ROOT_PATH}/cmake/eigen.cmake)
include($ENV{PROJECT_ROOT_PATH}/cmake/osqp-0.6.cmake)
include($ENV{PROJECT_ROOT_PATH}/cmake/osqp-eigen.cmake)

#python
# set(Python_INLCUDE_DIRS $ENV{TOOL_BASE_PATH}/Python-3.9.5/include/python3.9)
set(Python_INLCUDE_DIRS /usr/include/python3.8)
set(Python_LIBRARIES "/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so")
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")
include_directories(${Python_INLCUDE_DIRS})


# 定义子目录src，用以递归的调用src中的MakeLists.txt，生成library
add_subdirectory(config)
add_subdirectory(reference_line)
add_subdirectory(EMPlanner)
add_subdirectory(plot)
add_subdirectory(routing)
add_subdirectory(localization)
add_subdirectory(perception)

# Create the executable
add_executable(path_plan_test path_plan_test.cc)

target_link_libraries(path_plan_test PUBLIC
    reference_line_provider
    plot
    routing
    localization
    perception
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

# find_package(Eigen3 REQUIRED)
# find_package(OsqpEigen)




