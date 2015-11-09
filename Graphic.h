//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_GRAPHIC_H
#define LIBKINE_GRAPHIC_H

#include <mgl2/qt.h>
#include <unistd.h>

#include "Robot.h"

class Graphic {
public:
    Graphic(Robot *robot);   
    
    void SetK(double K);
    
private:
    int DrawAxes(mglQT *gr);
    
    Robot *m_Robot;
    
    double m_K;
};


#endif //LIBKINE_GRAPHIC_H
