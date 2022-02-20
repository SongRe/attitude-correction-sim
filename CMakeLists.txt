cmake_minimum_required(VERSION 3.16)

# set project name
project(ATTITUDE_CONTROL C CXX)

# ensures that the graphical dependencies are loaded
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# add the generated libraries
# add_subdirectory(external)

add_subdirectory(control)
add_subdirectory(physics_sim)
add_subdirectory(sim_viewer)
add_subdirectory(control_sim)



set(CMAKE_C)

# add the include folder
# include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(sim_with_viewer sim_with_viewer.cpp)
target_link_libraries(sim_with_viewer PUBLIC physicssim simviewer)

# copies asset files
configure_file(${CMAKE_SOURCE_DIR}/assets/cubesat_model_tex.png ${CMAKE_BINARY_DIR}/assets/cubesat_model_tex.png COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/assets/origin.vs ${CMAKE_BINARY_DIR}/assets/origin.vs COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/assets/origin.fs ${CMAKE_BINARY_DIR}/assets/origin.fs COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/assets/sat.vs ${CMAKE_BINARY_DIR}/assets/sat.vs COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/assets/sat.fs ${CMAKE_BINARY_DIR}/assets/sat.fs COPYONLY)