cmake_minimum_required(VERSION 3.16)

# creates the command sim library
add_library(commandsim STATIC src/command_sim.cpp)
target_include_directories(commandsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(commandsim PUBLIC physicssim vmqmath cntrlinf)