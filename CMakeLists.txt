cmake_minimum_required(VERSION 2.8)

project (neptune)

find_package (OpenCV REQUIRED)

include_directories (include ${OpenCV_INCLUDE_DIRS})

set(SOURCES src/main.cpp)

add_executable (exe ${SOURCES})

target_link_libraries (exe ${OpenCV_LIBS} -lopencv_highgui)

