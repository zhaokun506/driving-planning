aux_source_directory(. LOCALIZATION_SRC)

add_library(localization SHARED ${LOCALIZATION_SRC})

message(工程目录是：${PROJECT_SOURCE_DIR})

target_link_libraries(localization PUBLIC
    EMPlanner
)