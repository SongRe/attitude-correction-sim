cmake_minimum_required(VERSION 3.16)


# creates the vmqmath library
add_library(vmqmath STATIC src/math/vmath.c src/math/mmath.c src/math/qmath.c)
target_include_directories(vmqmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_link_libraries(vmqmath PUBLIC m)  # math.h library

# creates the control interface library
add_library(cntrlinf STATIC src/control_bridge.c src/control_proxy.c)
target_include_directories(cntrlinf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# creates the control algorithms library
# add_library(cntrlalg STATIC src/control_pid.c)
# target_include_directories(cntrlalg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# creates a test implementation (not for production code but can be an example)
add_executable(exper_test src/exper_test.c)
target_include_directories(exper_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(exper_test PUBLIC vmqmath cntrlinf)

# sets up the test executables for the vmqmath library
add_executable(test_mmath tests/vmqmath/test_mmath.cpp)
target_link_libraries(test_mmath PRIVATE Catch2::Catch2WithMain vmqmath)

add_executable(test_vmath tests/vmqmath/test_vmath.cpp)
target_link_libraries(test_vmath PRIVATE Catch2::Catch2WithMain vmqmath)

add_executable(test_qmath tests/vmqmath/test_qmath.cpp)
target_link_libraries(test_qmath PRIVATE Catch2::Catch2WithMain vmqmath)

# sets up an experimenting executable for vmqmath
add_executable(exper tests/vmqmath/exper.cpp)
target_link_libraries(exper PRIVATE vmqmath)