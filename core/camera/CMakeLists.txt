add_library(camera
    src/camera_processor.cpp
    src/camera_interface.cpp
)

target_include_directories(camera PUBLIC include/)

target_link_libraries(camera PRIVATE
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
)

install(TARGETS camera DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})