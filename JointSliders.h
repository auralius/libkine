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

#include "PracticalSocket.h"
#include "Robot.h"

using namespace std;

class JointSliders {
public:
    JointSliders(vector<Robot *> *robot, int verbose = 0);
    ~JointSliders();

    void End();

    Fl_Window *GetWindow();

private:
    static void *Run(void *data);
    void *RunWorker();

    static void SliderCB(Fl_Widget *w, void *data);
    void SliderCBWorker();
    
    static void *UDPReceiver(void *data);
    void UDPReceiverWorker();

    Fl_Window *m_Win;
    vector <Fl_Slider *> m_Sliders;

    int m_Running;
    vector <Robot *> *m_Robots;
    int m_Verbose;
    
    int m_DataSize;
};


#endif //LIBKINE_JOINTSLIDERS_H
