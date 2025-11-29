add_executable(slam_test
    main.cpp
)

target_link_libraries(slam_test PRIVATE
    jr_vio
    jr_bag
)

# Propagate WITH_RERUN definition from jr_vio
if(TARGET rerun_sdk)
    target_compile_definitions(slam_test PRIVATE WITH_RERUN)
endif()
