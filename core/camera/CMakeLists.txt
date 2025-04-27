# Add source files
set(SOURCES
    src/camera_device_api.cpp
)

# Create library
add_library(camera ${SOURCES})

# Add include directories
target_include_directories(camera PUBLIC include/)

# Link libraries
if(ANDROID)
    target_link_libraries(camera
        android_camera
    )
endif()