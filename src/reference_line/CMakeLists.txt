aux_source_directory(. REFERENCELINE_SRC)

add_library(reference_line_provider SHARED ${REFERENCELINE_SRC})
set_target_properties(reference_line_provider  PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(reference_line_provider PUBLIC
    # Eigen3::Eigen
    ${eigen}
    ${OsqpEigen}
)