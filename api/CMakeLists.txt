# Add source files
set(SOURCES
    src/bag_api.cpp
    src/camera_device_api.cpp
    src/runner_api.cpp
)

find_package(jr_msgs CONFIG REQUIRED)

# Create library
add_library(jr_api ${SOURCES})

# Add include directories
target_include_directories(jr_api PUBLIC include/)

# Link libraries
target_link_libraries(jr_api
    PUBLIC
    jr_imgproc
    PRIVATE
    jr_bag
    jr_middleware
    jr_camera_node
    jr_msgs::jr_msgs
)

if(ANDROID)
    target_link_libraries(jr_api PRIVATE
        jr_android
    )
endif()