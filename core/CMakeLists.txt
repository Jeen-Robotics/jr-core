add_subdirectory(math)
add_subdirectory(imgproc)

# Rust middleware backend (not available on Android)
if(NOT ANDROID)
    add_subdirectory(middleware_rs)
    add_subdirectory(middleware)
    add_subdirectory(bag)
endif()

add_subdirectory(nodes)
add_subdirectory(vio)
