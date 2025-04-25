add_executable(camera_test
    main.cpp
)

target_link_libraries(camera_test PRIVATE
    camera
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
)