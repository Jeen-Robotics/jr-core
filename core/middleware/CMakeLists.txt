find_package(protobuf CONFIG REQUIRED)

add_library(jr_middleware
    src/middleware.cpp
)

target_include_directories(jr_middleware 
    PUBLIC 
        include/
)

# Link against Rust middleware backend
target_link_libraries(jr_middleware 
    PUBLIC 
        protobuf::libprotobuf
        middleware_rs_cpp
)

install(TARGETS jr_middleware DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})

if(BUILD_TESTS)
    add_subdirectory(test)
endif()
