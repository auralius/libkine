cmake_minimum_required(VERSION 2.8)

project(LIBKINE)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(FLTK REQUIRED)
find_package(OpenGL REQUIRED)

add_library(kine Link.cpp Link.h Robot.cpp Robot.h Graphic.cpp Graphic.h JointSliders.cpp JointSliders.h PracticalSocket.cpp PracticalSocket.h)
target_include_directories (kine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (VTK_LIBRARIES)
    target_link_libraries(kine ${VTK_LIBRARIES})
else ()
    target_link_libraries(kine vtkHybrid vtkWidgets)
endif ()

add_executable(demo1 main1.cpp)
add_executable(demo2 main2.cpp)

install(TARGETS demo1 RUNTIME DESTINATION bin)
install(TARGETS demo2 RUNTIME DESTINATION bin)

target_link_libraries(demo1 LINK_PUBLIC kine pthread armadillo ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})
target_link_libraries(demo2 LINK_PUBLIC kine pthread armadillo ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})

target_compile_options(kine PUBLIC -std=c++11 -fstack-protector-all)