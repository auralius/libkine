/*
 * main1.m
 * Load a robot programmatically.
 */

#include "Robot.h"
#include "Graphic.h"

int running = 1;

void *simulation(void *ptr) {
    Robot *puma = (Robot *)ptr;

    double t = 0;
    double y = 0;
    while (running) {
        puma->ActuateJoint(2, -0.2);
        puma->ActuateJoint(3, y);
        puma->Update(1);
        y = 0.2*sin(t);
        t = t + 0.01;
#ifdef WIN32
        Sleep(1);
#else
        usleep(1000);
#endif
    }

    cout << "bye\n";

    return NULL;
}

int main(int argc, char **argv) {
    double d2 = 218.44 / 1000.0;
    double d3 = -88.9 / 1000.0;
    double a2 = 332.74 / 1000.0;
    double a3 = 0 / 1000.0;
    double d4 = 432.09 / 1000.0;
    double d6 = 53.34 / 1000.0;


    Robot puma;
    puma.SetBaseSTLFileName("../model/puma/base.STL");
    puma.SetBasePosition(-0.10, 0, 0);

    double c[3] = { 1, 1, 0 };

    //      a, alpha, d, theta, type
    puma.AddLink(0, 0, 0, 0, Link::REVOLUTE, "../model/puma/link1.STL", 'r');
    puma.AddLink(0, -M_PI_2, d2, 0, Link::REVOLUTE, "../model/puma/link2.STL", 'g');
    puma.AddLink(a2, 0, d3, 0, Link::REVOLUTE, "../model/puma/link3.STL", 'b');
    puma.AddLink(a3, M_PI_2, d4, 0, Link::REVOLUTE, "../model/puma/link4.STL", 'y');
    puma.AddLink(0, -M_PI_2, 0, 0, Link::REVOLUTE, "../model/puma/link5.STL", 'm');
    puma.AddLink(0, M_PI_2, d6, 0, Link::REVOLUTE);

    puma.SetLogFileName("puma.log");

    pthread_t thread;
    int ret = pthread_create(&thread, NULL, simulation, (void *)&puma);

    Graphic G("Puma");
    G.AddRobot(&puma);
    G.SetGraphicScaling(0.15);
    G.SetOpacity(1);
    G.Run();  // Blocks here until VTK GUI is closed

    running = 0;
    pthread_join(thread, NULL);

	return 0;
}