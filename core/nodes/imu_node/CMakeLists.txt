find_package(jr_msgs CONFIG REQUIRED)

add_library(jr_imu_node
    src/imu_node.cpp
)

target_include_directories(jr_imu_node PUBLIC include/)

target_link_libraries(jr_imu_node
    PUBLIC
    jr_middleware
    jr_msgs::jr_msgs
)

if(ANDROID)
    target_link_libraries(jr_imu_node PRIVATE
            jr_android
            android
            log
    )
endif()

install(TARGETS jr_imu_node DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})

