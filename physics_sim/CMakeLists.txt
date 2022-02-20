cmake_minimum_required(VERSION 3.16)

# creates the physics sim library
add_library(physicssim STATIC src/physics_sim.cpp src/rigidbody.cpp)
target_include_directories(physicssim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(physicssim PUBLIC vmqmath)