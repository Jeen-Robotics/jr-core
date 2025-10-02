find_package(protobuf CONFIG REQUIRED)

# Generate protobuf sources from bag.proto
set(PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/bag.proto")
set(PROTO_SRC "${CMAKE_CURRENT_BINARY_DIR}/bag.pb.cc")
set(PROTO_HDR "${CMAKE_CURRENT_BINARY_DIR}/bag.pb.h")

add_custom_command(
    OUTPUT ${PROTO_SRC} ${PROTO_HDR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating protobuf sources for bag.proto"
    VERBATIM
)

add_library(jr_bag
    src/bag_reader.cpp
    src/bag_writer.cpp
    ${PROTO_SRC}
    ${PROTO_HDR}
)

target_include_directories(jr_bag 
    PUBLIC 
        include/
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}  # For generated protobuf headers
)

target_link_libraries(jr_bag PRIVATE protobuf::libprotobuf jr_middleware)

install(TARGETS jr_bag DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})