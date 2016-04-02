//
// Created by manurunga on 01.04.16.
//

#include "JointSliders.h"


JointSliders::JointSliders(Robot *robot) {
    m_Robot = robot;

    // How many joints we have?
    size_t n = robot->GetLinks().size();

    // adapt the size to the number of the links
    m_Win = new Fl_Window(360, 50 * n);

    for (size_t i = 0; i < n; i++) {

        char *c = new char(3);
        sprintf(c, "%lu", i);
        Fl_Slider *slider;
        slider = new Fl_Slider(20, 25 + (40 * (int) i), 300, 20, c);

        slider->callback(SliderCB, (void *) this);

        slider->bounds(robot->GetLinks().at(i)->GetMinJointLimit(),
                       robot->GetLinks().at(i)->GetMaxJointLimit());

        slider->type(1);
        slider->value(0);
        m_Sliders.push_back(slider);
    }

    m_Win->show();

    pthread_t thread;
    pthread_create(&thread, NULL, Run, (void *) this);
}

JointSliders::~JointSliders() {
    // Window has been closed, now clean up!
    // TODO: The odes below cause exit 139
//    for (size_t i = 0; i < m_Sliders.size(); i++) {
//        delete [] m_Sliders.at(i);
//    }
    delete m_Win;
}

void JointSliders::SliderCB(Fl_Widget *w, void *data) {
    ((JointSliders *) data)->SliderCBWorker();
}

void JointSliders::SliderCBWorker() {
    size_t n = m_Robot->GetLinks().size();
    for (size_t i = 0; i < n; i++) {
        m_Robot->ActuateJoint((int) i, m_Sliders.at(i)->value());
    }
    m_Robot->Update();
}

Fl_Window *JointSliders::GetWindow() {
    return m_Win;
}

void *JointSliders::Run(void *data) {
    ((JointSliders *) data)->RunWorker();
}

void *JointSliders::RunWorker() {
    Fl::run(); // Blocks here
}

void JointSliders::End() {
    m_Win->end();
}



