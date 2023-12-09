add_subdirectory(matplot)

add_library(plot SHARED plot.cc)
set_target_properties(plot  PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(plot PUBLIC
    matplotlib_cpp
    Eigen3::Eigen
    routing
    reference_line_provider
    EMPlanner
    localization
    perception
)