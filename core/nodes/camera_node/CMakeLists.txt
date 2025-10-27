find_package(jr_msgs CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

add_library(jr_camera_node src/camera_node.cpp)

target_include_directories(jr_camera_node PUBLIC include/)

target_link_libraries(jr_camera_node
    PUBLIC
    jr_middleware
    jr_msgs::jr_msgs
    PRIVATE
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
)

if(ANDROID)
    target_link_libraries(jr_camera_node PRIVATE
            jr_android
            android
            log
            mediandk
            camera2ndk
    )
endif()

install(TARGETS jr_camera_node DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})
