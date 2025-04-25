add_library(camera
    src/camera_interface.cpp
)

target_include_directories(camera PUBLIC include/)

install(TARGETS camera DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})