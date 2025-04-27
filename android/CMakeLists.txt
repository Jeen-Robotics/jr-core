add_library(jr_android
    src/camera_device.cpp
)

# Add include directories
target_include_directories(jr_android PUBLIC include/)

# Link libraries
target_link_libraries(jr_android
    android
    log
    mediandk
    camera2ndk
    jr_imgproc
) 