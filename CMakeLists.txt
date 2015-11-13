cmake_minimum_required(VERSION 2.8)

project(libkine)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(libkine main.cpp Link.cpp Link.h Robot.cpp Robot.h Graphic.cpp Graphic.h)

if (VTK_LIBRARIES)
    target_link_libraries(libkine ${VTK_LIBRARIES})
else ()
    target_link_libraries(libkine vtkHybrid vtkWidgets)
endif ()

install(TARGETS libkine RUNTIME DESTINATION bin)

target_link_libraries(libkine mgl-wnd mgl pthread armadillo VLCore VLGLUT VLGraphics VLMolecule glut)