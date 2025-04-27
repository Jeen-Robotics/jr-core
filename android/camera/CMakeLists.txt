# Add source files
set(SOURCES
    src/camera_device.cpp
)

# Create library
add_library(android_camera ${SOURCES})

# Add include directories
target_include_directories(android_camera PUBLIC include/)

# Link libraries
target_link_libraries(android_camera
    android
    log
    mediandk
    camera2ndk
    imgproc
) 