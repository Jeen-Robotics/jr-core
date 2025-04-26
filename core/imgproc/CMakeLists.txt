add_library(imgproc
    src/imgproc.cpp
)

target_include_directories(imgproc PUBLIC include/)

install(TARGETS imgproc DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})