# Add source files
set(SOURCES
    src/camera_device_api.cpp
)

# Create library
add_library(jr_api ${SOURCES})

# Add include directories
target_include_directories(jr_api PUBLIC include/)

# Link libraries
target_link_libraries(jr_api PUBLIC
    jr_imgproc
)

if(ANDROID)
    target_link_libraries(jr_api PRIVATE
        jr_android
    )
endif()