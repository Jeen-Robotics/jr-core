add_executable(jr_middleware_unit_tests
    unit/backend_unit_test.cpp
)

target_link_libraries(jr_middleware_unit_tests
    PRIVATE
        jr_middleware
        gtest::gtest
)

add_test(NAME jr_middleware_unit_tests COMMAND jr_middleware_unit_tests)

add_executable(jr_middleware_integration_tests
    integration/middleware_integration_test.cpp
)

target_link_libraries(jr_middleware_integration_tests
    PRIVATE
        jr_middleware
        gtest::gtest
)

add_test(NAME jr_middleware_integration_rust COMMAND jr_middleware_integration_tests)
set_tests_properties(jr_middleware_integration_rust PROPERTIES
    ENVIRONMENT "JR_MIDDLEWARE_BACKEND=rust"
)

add_test(NAME jr_middleware_integration_cpp COMMAND jr_middleware_integration_tests)
set_tests_properties(jr_middleware_integration_cpp PROPERTIES
    ENVIRONMENT "JR_MIDDLEWARE_BACKEND=cpp"
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(jr_middleware_linux_tests
        linux/linux_transport_test.cpp
    )

    target_link_libraries(jr_middleware_linux_tests
        PRIVATE
            jr_middleware
            gtest::gtest
    )

    add_test(NAME jr_middleware_linux_tests COMMAND jr_middleware_linux_tests)
endif()
