add_executable(jr_middleware_tests
    middleware_test.cpp
)

target_link_libraries(jr_middleware_tests
    PRIVATE
        jr_middleware
        gtest::gtest
)

add_test(NAME jr_middleware_tests COMMAND jr_middleware_tests)
