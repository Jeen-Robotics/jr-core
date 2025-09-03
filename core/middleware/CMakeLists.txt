add_library(jr_middleware
    src/middleware.cpp
)

target_include_directories(jr_middleware PUBLIC include/)

install(TARGETS jr_middleware DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})

if(BUILD_TESTS)
    add_subdirectory(test)
endif()