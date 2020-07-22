
add_library(plot gnuplot_i.h gnuplot_i.cpp Graph_Logger.h Graph_Logger.cpp)
target_include_directories(plot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(plot LINK_PUBLIC odrive_cpp)