//
// Created by manurunga on 01.04.16.
//

#include "JointSliders.h"


JointSliders::JointSliders(Robot *robot, int verbose) {
    m_Running = 1;
    m_Robot = robot;
    m_Verbose = verbose;

    // How many joints we have?
    size_t n = robot->GetLinks().size();

    // adapt the size to the number of the links
    m_Win = new Fl_Window(360, 50 * n);

    for (size_t i = 0; i < n; i++) {

        char *c = new char(10);
        snprintf(c, 10, "Joint-%lu", i);
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

    pthread_t thread1, thread2;
    pthread_create(&thread1, NULL, UDPReceiver, (void *) this);
    pthread_create(&thread2, NULL, Run, (void *) this);    
}

JointSliders::~JointSliders() {
    // Window has been closed, now clean up!
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
    m_Robot->Update(m_Verbose);
}

void *JointSliders::UDPReceiver(void *data) {
    ((JointSliders *) data)->UDPReceiverWorker();
}

void JointSliders::UDPReceiverWorker() {
    size_t n = m_Robot->GetLinks().size();
    double data[n];
    
    int sockfd;
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &n, sizeof(n)) == -1) 
        cerr << "error: setsockopt\n";
    
    struct sockaddr_in serv, client;
    
    bzero(&serv, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_port = htons(12345);
    serv.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(sockfd, (struct sockaddr* ) &serv, sizeof(serv)) == -1) 
        cerr << "error: bind\n";
        
    socklen_t l = sizeof(client);

    cout << "\nListening in port 12345...\n";
    cout.flush();
    
    while (m_Running) {
        int rc = recvfrom(sockfd, (char *)&data, 8*n, 0, (struct sockaddr *)&client, &l);
        
        if (rc < 0) 
            cerr << "error: recvfrom\n";
        else {
            for (size_t i = 0; i < n; i++) 
                m_Robot->ActuateJoint((int) i, data[i]);
            m_Robot->Update(m_Verbose);
        } // else
    } //while (m_Running)        
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
    m_Running = 0;
    
    for (int i = 0; i < m_Sliders.size(); i ++) {
        Fl_Slider *slider = m_Sliders.at(i);
        delete [] slider->label();
    }

    m_Win->end();
}



