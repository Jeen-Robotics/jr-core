add_library(jr_math
    src/jr_math.cpp
)

target_include_directories(jr_math PUBLIC include/)

install(TARGETS jr_math DESTINATION lib)
install(FILES ${INTERFACE_INCLUDE_DIRECTORIES})
