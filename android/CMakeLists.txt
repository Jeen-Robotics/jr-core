add_library(jr_android
    src/camera_device.cpp
    src/imu_device.cpp
)

# Add include directories
target_include_directories(jr_android PUBLIC include/)

# Link libraries
target_link_libraries(jr_android
    PUBLIC
    jr_imgproc
    PRIVATE
    android
    log
    mediandk
    camera2ndk
)