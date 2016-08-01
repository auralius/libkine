cmake_minimum_required(VERSION 2.8)

project(libkine)

add_definitions(-std=c++11) 

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(FLTK REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(libkine main.cpp Link.cpp Link.h Robot.cpp Robot.h Graphic.cpp Graphic.h JointSliders.cpp JointSliders.h PracticalSocket.cpp PracticalSocket.h)

if (VTK_LIBRARIES)
    target_link_libraries(libkine ${VTK_LIBRARIES})
else ()
    target_link_libraries(libkine vtkHybrid vtkWidgets)
endif ()

install(TARGETS libkine RUNTIME DESTINATION bin)

target_link_libraries(libkine pthread armadillo ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})