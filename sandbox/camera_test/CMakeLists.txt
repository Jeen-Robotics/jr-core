add_executable(camera_test
    main.cpp
)

target_link_libraries(camera_test PRIVATE
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
    jr_middleware
)