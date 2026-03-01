add_subdirectory(math)
add_subdirectory(imgproc)
add_subdirectory(middleware)

if(NOT SKIP_BAG_MODULE)
    add_subdirectory(bag)
endif()

# nodes/camera_node requires OpenCV, skip if no OpenCV deps
if(NOT SKIP_BAG_MODULE)
    add_subdirectory(nodes)
endif()

if(NOT SKIP_VIO_MODULE)
    add_subdirectory(vio)
endif()

if(BUILD_RUST_MIDDLEWARE AND NOT ANDROID)
    add_subdirectory(middleware_rs)
endif()
