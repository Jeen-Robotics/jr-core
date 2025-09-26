add_library(jr_middleware
    src/subscription.cpp
    src/middleware.cpp
    src/in_process_middleware.cpp
    src/node.cpp
)

target_include_directories(jr_middleware PUBLIC include/)

find_package(protobuf CONFIG REQUIRED)
target_link_libraries(jr_middleware PUBLIC protobuf::libprotobuf)

install(TARGETS jr_middleware DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})

if(BUILD_TESTS)
    add_subdirectory(test)
endif()