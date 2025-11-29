add_executable(slam_test
    main.cpp
)

target_link_libraries(slam_test PRIVATE
    jr_vio
    jr_bag
)
