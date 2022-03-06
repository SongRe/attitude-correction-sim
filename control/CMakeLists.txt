cmake_minimum_required(VERSION 3.16)

# creates the control interface library
add_library(cntrlinf STATIC src/control_bridge.c src/control_proxy.c)
target_include_directories(cntrlinf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cntrlinf PUBLIC vmqmath)

# creates the control algorithms library
add_library(cntrlalg STATIC src/control_utils.c)
target_include_directories(cntrlalg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cntrlalg PUBLIC vmqmath)

# creates a test implementation (not for production code but can be an example)
add_executable(exper_test src/exper_test.c)
target_include_directories(exper_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(exper_test PUBLIC vmqmath cntrlinf cntrlalg)