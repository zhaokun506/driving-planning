
# set(Numpy_INCLUDE_DIRS "/usr/include/python3.6m/numpy")
# set(Numpy_LIBRARIES "/usr/lib/x86_64-linux-gnu/libboost_numpy.so")

# Library target
add_library(matplotlib_cpp INTERFACE)

target_include_directories(matplotlib_cpp INTERFACE
    ${PROJECT_SOURCE_DIR}#工程目录
    ${Python_INCLUDE_DIRS}
)

target_link_libraries(matplotlib_cpp INTERFACE
                      ${Python_LIBRARIES}
)

target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_17
)
