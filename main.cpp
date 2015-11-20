#include "Robot.h"
#include "Graphic.h"

int running = 1;

void *simulation(void *ptr) {
    Robot *puma = (Robot *)ptr;

    double x = 0;
    while (running) {
        puma->SetTheta(1, x);
        puma->Update(1);
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
    puma.SetBasePosition(-0.10, 0, 0);

    double c[3] = { 1, 1, 0 };

    //      a, alpha, d, theta, type
    puma.AddLink(0, 0, 0, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link1.STL", 'r');
    puma.AddLink(0, -M_PI_2, d2, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link2.STL", 'g');
    puma.AddLink(a2, 0, d3, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link3.STL", 'b');
    puma.AddLink(a3, M_PI_2, d4, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link4.STL", 'w');
    puma.AddLink(0, -M_PI_2, 0, 0, Link::REVOLUTE, "E:\\GIT\\libkine\\model\\puma\\link5.STL", 'k');
    puma.AddLink(0, M_PI_2, d6, 0, Link::REVOLUTE);

    puma.SetLogFileName("puma.log");

    pthread_t thread;
    int ret = pthread_create(&thread, NULL, simulation, (void *)&puma);

    Graphic G(&puma, "Puma");
    G.SetGraphicScaling(0.15);
    G.SetOpacity(1);
    G.Run();  // Blocks here until VTK GUI is closed

    running = 0;
    pthread_join(thread, NULL);
}