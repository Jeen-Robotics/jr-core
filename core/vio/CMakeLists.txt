find_package(jr_msgs CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui features2d calib3d video)

set(VIO_SOURCES
    src/msg_utils.cpp
    src/camera_calibration.cpp
    src/imu_state.cpp
    src/vio_node.cpp
    src/visualizer_node.cpp
    src/multi_camera_vio_node.cpp
    src/multi_camera_visualizer_node.cpp
)

set(VIO_LIBS
    jr_middleware
    jr_msgs::jr_msgs
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_features2d
    opencv_calib3d
    opencv_video
)

# Add ReRun visualizer if available (desktop builds only)
if(TARGET rerun_sdk)
    list(APPEND VIO_SOURCES src/multi_camera_rerun_node.cpp)
    list(APPEND VIO_LIBS rerun_sdk)
    add_compile_definitions(WITH_RERUN)
endif()

add_library(jr_vio ${VIO_SOURCES})

target_include_directories(jr_vio PUBLIC include/)

target_link_libraries(jr_vio PUBLIC ${VIO_LIBS})

install(TARGETS jr_vio DESTINATION lib)

