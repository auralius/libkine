#include "Robot.h"
#include "Graphic.h"

int running = 1;

void *simulation(void *ptr) {
    Robot *puma = (Robot *)ptr;

    double x = 0;
    while (running) {

        puma->SetTheta(1, x);
        puma->Update();
        x = x + .01;
#ifdef WIN32
        Sleep(1);
#else
        usleep(1000)
#endif
    }

    cout << "bye\n";

    return NULL;
}

int main(int, char **argv) {
    double d2 = 218.44 / 1000.0;
    double d3 = -88.9 / 1000.0;
    double a2 = 332.74 / 1000.0;
    double a3 = 0 / 1000.0;
    double d4 = 432.09 / 1000.0;
    double d6 = 53.34 / 1000.0;

    Robot puma;
    puma.SetBaseSTLFileName("E:\\GIT\\libkine\\model\\puma\\base.STL");

    //      a, alpha, d, theta, type
    puma.AddLink(0, 0, 0, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link1.STL");
    puma.AddLink(0, -M_PI_2, d2, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link2.STL");
    puma.AddLink(a2, 0, d3, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link3.STL");
    puma.AddLink(a3, M_PI_2, d4, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link4.STL");
    puma.AddLink(0, -M_PI_2, 0, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link5.STL");
    puma.AddLink(0, M_PI_2, d6, 0, Link::REVOLUTE);

    pthread_t thread;
    int ret = pthread_create(&thread, NULL, simulation, (void *)&puma);

    Graphic G(&puma, "Puma");
    G.SetGraphicScaling(0.15);
    G.SetOpacity(0.5);
    G.Run();  // Blocks here until VTK GUI is closed

    running = 0;
    pthread_join(thread, NULL);
}