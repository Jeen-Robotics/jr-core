find_package(protobuf CONFIG REQUIRED)
find_package(zstd CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(jr_msgs CONFIG REQUIRED)

# Generate protobuf sources from bag.proto
set(PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/bag.proto")
set(PROTO_SRC "${CMAKE_CURRENT_BINARY_DIR}/bag.pb.cc")
set(PROTO_HDR "${CMAKE_CURRENT_BINARY_DIR}/bag.pb.h")

# Generate protobuf sources from bag.proto
# For Android, we use the generated sources from scripts/install_android_deps.sh
if(NOT ANDROID)
    add_custom_command(
        OUTPUT ${PROTO_SRC} ${PROTO_HDR}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating protobuf sources for bag.proto"
        VERBATIM
    )
endif()

add_library(jr_bag
    src/bag_reader.cpp
    src/bag_writer.cpp
    src/video_encoder.cpp
    src/video_decoder.cpp
    ${PROTO_SRC}
    ${PROTO_HDR}
)

target_include_directories(jr_bag 
    PUBLIC 
        include/
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}  # For generated protobuf headers
)

target_link_libraries(jr_bag 
    PRIVATE 
        jr_middleware 
        jr_msgs::jr_msgs
        protobuf::libprotobuf 
        zstd::libzstd_static
        opencv_core
        opencv_imgproc
        opencv_videoio
)

if(ANDROID)
    target_link_libraries(jr_bag PRIVATE
        android
        log
        mediandk
        camera2ndk
    )
endif()

install(TARGETS jr_bag DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})