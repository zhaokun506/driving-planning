

# Library target
add_library(matplotlib_cpp INTERFACE)

target_include_directories(matplotlib_cpp INTERFACE
    ${PROJECT_SOURCE_DIR}#工程目录
    ${Python_INLCUDE_DIRS}
)

target_link_libraries(matplotlib_cpp INTERFACE
                      ${Python_LIBRARIES}
)

target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_17
)


add_library(plot plot.cc)
find_package(Eigen3)
target_include_directories(plot PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
                             "/usr/include/python3.6m"#python目录
)


target_link_libraries(plot PUBLIC
    matplotlib_cpp
    Eigen3::Eigen
    routing
    reference_line_provider
    EMPlanner
    localization
    perception
)