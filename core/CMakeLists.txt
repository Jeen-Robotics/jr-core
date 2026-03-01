add_subdirectory(math)
add_subdirectory(imgproc)

# Rust middleware backend (not available on Android)
if(NOT ANDROID)
    add_subdirectory(middleware_rs)
    add_subdirectory(middleware)
else()
    message(WARNING "Middleware is not available on Android (Rust toolchain required). "
                    "Targets depending on jr_middleware will fail to link.")
endif()

# Optional modules that require additional dependencies
find_package(OpenCV QUIET COMPONENTS core)

if(OpenCV_FOUND)
    find_package(mcap CONFIG QUIET)
    if(mcap_FOUND AND NOT ANDROID)
        add_subdirectory(bag)
    elseif(NOT mcap_FOUND)
        message(STATUS "Skipping bag module (mcap not found)")
    endif()
    
    add_subdirectory(nodes)
    add_subdirectory(vio)
else()
    message(STATUS "Skipping bag, nodes, vio modules (OpenCV not found)")
endif()
