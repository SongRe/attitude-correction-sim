cmake_minimum_required(VERSION 3.16)

# creates the vmqmath library
add_library(vmqmath STATIC src/vmath.c src/mmath.c src/qmath.c)
target_include_directories(vmqmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_link_libraries(vmqmath PUBLIC m)  # math.h library

# sets up the test executables for the vmqmath library
add_executable(test_mmath tests/test_mmath.cpp)
target_link_libraries(test_mmath PRIVATE Catch2::Catch2WithMain vmqmath)

add_executable(test_vmath tests/test_vmath.cpp)
target_link_libraries(test_vmath PRIVATE Catch2::Catch2WithMain vmqmath)

add_executable(test_qmath tests/test_qmath.cpp)
target_link_libraries(test_qmath PRIVATE Catch2::Catch2WithMain vmqmath)