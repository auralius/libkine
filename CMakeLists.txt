cmake_minimum_required(VERSION 2.8)

project(libkine)

add_executable(libkine main.cpp Link.cpp Link.h Robot.cpp Robot.h Graphic.cpp Graphic.h)

install(TARGETS libkine RUNTIME DESTINATION bin)

link_directories(/usr/lib)

include_directories(/usr/include /usr/include/qt4 /usr/include/qt4/Qt)

#target_link_libraries(libkine SoQt4 QtOpenGL QtGui QtCore  Xmu Xi Coin GL Xext SM ICE X11 dl pthread armadillo)

target_link_libraries(libkine mgl-wnd mgl GL pthread armadillo)