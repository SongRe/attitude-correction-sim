cmake_minimum_required(VERSION 3.16)

# creates the required libraries
add_library(simviewer STATIC src/glad.c src/sim_viewer.cpp)
target_include_directories(simviewer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(simviewer PUBLIC physicssim glfw -lm -pthread ${OPENGL_LIBRARIES} ${CMAKE_DL_LIBS})