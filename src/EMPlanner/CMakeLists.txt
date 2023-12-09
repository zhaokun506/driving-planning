aux_source_directory(. EMPLANNER_SRC)

add_library(EMPlanner SHARED ${EMPLANNER_SRC} )

message(工程目录是：${PROJECT_SOURCE_DIR})


#target_link_libraries(perception PUBLIC)