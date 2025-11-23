find_package(mcap CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(jr_msgs CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(mcap_test main.cpp)
target_link_libraries(mcap_test mcap::mcap jr_msgs::jr_msgs protobuf::libprotobuf opencv::opencv)
