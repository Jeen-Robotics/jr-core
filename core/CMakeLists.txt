add_subdirectory(math)
add_subdirectory(imgproc)

# Rust middleware backend (required for middleware)
if(NOT ANDROID)
    add_subdirectory(middleware_rs)
endif()

add_subdirectory(middleware)

# Optional modules that require additional dependencies
find_package(OpenCV QUIET COMPONENTS core)

if(OpenCV_FOUND)
    find_package(mcap CONFIG QUIET)
    if(mcap_FOUND)
        add_subdirectory(bag)
    else()
        message(STATUS "Skipping bag module (mcap not found)")
    endif()
    
    add_subdirectory(nodes)
    add_subdirectory(vio)
else()
    message(STATUS "Skipping bag, nodes, vio modules (OpenCV not found)")
endif()
