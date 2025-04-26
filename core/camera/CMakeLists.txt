# Add source files
set(SOURCES
    src/android_camera_streamer.cpp
    src/android_camera_streamer_api.cpp
)

# Create library
add_library(camera ${SOURCES})

# Add include directories
target_include_directories(camera PUBLIC include/)

# Link libraries
target_link_libraries(camera
    android
    log
    mediandk
    camera2ndk
) 