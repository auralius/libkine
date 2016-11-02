//
// Created by manurunga on 01.04.16.
//

#include "JointSliders.h"


JointSliders::JointSliders(vector<Robot *> *robots, int verbose) {
    m_Running = 1;
    m_Robots = robots;
    m_Verbose = verbose;

    // How many joints we have?
    m_TotNumLinks = 0;
    for (size_t i = 0; i < m_Robots->size(); i++) {
        Robot *robot = m_Robots->at(i);
        m_TotNumLinks = m_TotNumLinks + robot->GetLinks().size();
    }

    pthread_t thread1, thread2;
    pthread_create(&thread1, NULL, UDPReceiver, (void *) this);
    pthread_create(&thread2, NULL, Run, (void *) this);    
}

JointSliders::~JointSliders() {
    // Window has been closed, now clean up!
    //if (m_Win)
    //    delete m_Win;
}

void JointSliders::SliderCB(Fl_Widget *w, void *data) {
    ((JointSliders *) data)->SliderCBWorker();
}

void JointSliders::SliderCBWorker() {
    size_t k = 0;
    
    for (size_t h = 0; h < m_Robots->size(); h ++) {
        Robot *robot = m_Robots->at(h);
        size_t n = robot->GetLinks().size();
        
        for (size_t i = 0; i < n; i++) {
            robot->ActuateJoint((int) i, m_Sliders.at(k)->value());
            k = k + 1;
        }
        robot->Update(m_Verbose);
    }
}

void *JointSliders::UDPReceiver(void *data) {
    ((JointSliders *) data)->UDPReceiverWorker();

    return NULL;
}

void JointSliders::UDPReceiverWorker() {
    double *data = new double[m_TotNumLinks];
    
    UDPSocket sockfd(12345);
    
    cout << "\nListening in port 12345...\n";
    cout << flush;
    
    string sourceAddress;             // Address of datagram source
    unsigned short sourcePort;        // Port of datagram source
    
    while (m_Running) {
        int rc = sockfd.recvFrom((char *)data, 8*m_TotNumLinks, sourceAddress, sourcePort);
        
        size_t k = 0;
        for (size_t h = 0; h < m_Robots->size(); h ++) {
            Robot *robot = m_Robots->at(h);
            size_t n = robot->GetLinks().size();
            
            for (size_t i = 0; i < n; i++) {
                robot->ActuateJoint((int) i, data[k]);
                k = k + 1;
            }
        
            robot->Update(m_Verbose);        
        }
    } //while (m_Running)      
    
    delete [] data;
}

Fl_Window *JointSliders::GetWindow() {
    return m_Win;
}

void *JointSliders::Run(void *data) {
    ((JointSliders *) data)->RunWorker();

    return NULL;
}

void *JointSliders::RunWorker() {
    // adapt the size to the number of the links
    m_Win = new Fl_Window(360, 50 * m_TotNumLinks);

    int k = 0;
    for (size_t h = 0; h < m_Robots->size(); h++) {
        Robot *robot = m_Robots->at(h);
        size_t n = robot->GetLinks().size();

        for (size_t i = 0; i < n; i++) {

            char *c = new char[10];
            snprintf(c, 10, "Joint-%lu", i);
            Fl_Slider *slider;
            slider = new Fl_Slider(20, 25 + (40 * k), 300, 20, c);
            k = k + 1;

            slider->callback(SliderCB, (void *) this);

            slider->bounds(robot->GetLinks().at(i)->GetMinJointLimit(),
            robot->GetLinks().at(i)->GetMaxJointLimit());

            slider->type(1);
            slider->value(0);
            m_Sliders.push_back(slider);
        }
    }

    m_Win->show();
    Fl::run(); // Blocks here

    return NULL;
}

void JointSliders::End() {
    m_Running = 0;
    
    for (size_t  i = 0; i < m_Sliders.size(); i ++) {
        Fl_Slider *slider = m_Sliders.at(i);
        delete [] slider->label();
    }

    m_Win->end();
}



