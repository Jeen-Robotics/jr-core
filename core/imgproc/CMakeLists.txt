add_library(jr_imgproc
    src/imgproc.cpp
)

target_include_directories(jr_imgproc PUBLIC include/)

install(TARGETS jr_imgproc DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})