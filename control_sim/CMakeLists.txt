cmake_minimum_required(VERSION 3.16)

# creates the control sim library
add_library(controlsim STATIC src/control_sim.cpp)
target_include_directories(controlsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(controlsim PUBLIC physicssim vmqmath cntrlinf)