//
// Created by manurunga on 01.04.16.
//

#ifndef LIBKINE_JOINTSLIDERS_H
#define LIBKINE_JOINTSLIDERS_H

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Slider.H>
#include <stdio.h>

#include <sstream>
#include <vector>

#include "Robot.h"

using namespace std;

class JointSliders {
public:
    JointSliders(Robot *robot);
    ~JointSliders();

    void End();

    Fl_Window *GetWindow();

private:
    static void *Run(void *data);
    void *RunWorker();

    static void SliderCB(Fl_Widget *w, void *data);
    void SliderCBWorker();

    Fl_Window *m_Win;
    vector <Fl_Slider *> m_Sliders;

    Robot *m_Robot;
};


#endif //LIBKINE_JOINTSLIDERS_H
