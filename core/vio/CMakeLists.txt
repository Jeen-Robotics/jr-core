find_package(jr_msgs CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui features2d calib3d video)

add_library(jr_vio
    src/msg_utils.cpp
    src/camera_calibration.cpp
    src/imu_state.cpp
    src/vio_node.cpp
    src/visualizer_node.cpp
    src/multi_camera_vio_node.cpp
    src/multi_camera_visualizer_node.cpp
)

target_include_directories(jr_vio PUBLIC include/)

target_link_libraries(jr_vio
    PUBLIC
        jr_middleware
        jr_msgs::jr_msgs
        opencv_core
        opencv_imgproc
        opencv_highgui
        opencv_features2d
        opencv_calib3d
        opencv_video
)

install(TARGETS jr_vio DESTINATION lib)

