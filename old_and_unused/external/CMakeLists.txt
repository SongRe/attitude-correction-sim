cmake_minimum_required(VERSION 3.16)

# creates the Quaternion library
add_library(Quaternion STATIC Quaternion/Quaternion.h Quaternion/Quaternion.c)
target_include_directories(Quaternion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Quaternion ${CMAKE_CURRENT_SOURCE_DIR}/)
target_link_libraries(Quaternion PUBLIC m) # math.h library

add_executable(TestQuaternion Quaternion/TestQuaternion.c)
target_link_libraries(TestQuaternion PUBLIC Quaternion)