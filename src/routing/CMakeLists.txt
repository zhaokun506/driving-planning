aux_source_directory(. ROUTING_SRC)

add_library(routing SHARED ${ROUTING_SRC})

message(工程目录是：${PROJECT_SOURCE_DIR})

# target_include_directories(routing PUBLIC 
#                             ${PROJECT_SOURCE_DIR}#工程目录
#                             "usr/include"#系统目录
#                             "usr/local/include"#安装文件目录
# )

#target_link_libraries(routing PUBLIC)