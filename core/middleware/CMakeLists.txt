find_package(protobuf CONFIG REQUIRED)

add_library(jr_middleware
    src/middleware.cpp
)

target_include_directories(jr_middleware 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link against Rust middleware backend
target_link_libraries(jr_middleware 
    PUBLIC 
        protobuf::libprotobuf
        middleware_rs_cpp
)

# Install library
install(TARGETS jr_middleware 
    EXPORT jr_middlewareTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY include/middleware
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()
