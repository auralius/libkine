#include "Robot.h"
#include "Graphic.h"

int running = 1;

void *simulation(void *ptr) {
    Robot *puma = (Robot *) ptr;

    double x = 0;
    while (running) {

        puma->SetTheta(1, x);
        puma->Update();
        x = x + .01;
        usleep(10000);
    }

    cout << "bye\n";
}

int main(int, char **argv) {
    double d2 = 243.5 / 1000.0;
    double d3 = -93.4 / 1000.0;
    double a2 = 431.8 / 1000.0;
    double a3 = -20.3 / 1000.0;
    double d4 = 433.1 / 1000.0;

    Robot puma;
    puma.SetBaseSTLFileName("/home/manurunga/Documents/GIT/libkine/model/puma/base.STL");
    //, "/home/manurunga/Documents/GIT/libkine/model/puma/link3.STL"
    //, "/home/manurunga/Documents/GIT/libkine/model/puma/link4.STL"
    //      a, alpha, d, theta, type
    puma.AddLink(0, 0, 0, 0, Link::REVOLUTE, "/home/manurunga/Documents/GIT/libkine/model/puma/link1.STL");
    puma.AddLink(0, -M_PI_2, d2, 0, Link::REVOLUTE, "/home/manurunga/Documents/GIT/libkine/model/puma/link2.STL");
    puma.AddLink(a2, 0, d3, 0, Link::REVOLUTE, "/home/manurunga/Documents/GIT/libkine/model/puma/link3.STL");
    puma.AddLink(0, M_PI_2, d4, 0, Link::REVOLUTE, "/home/manurunga/Documents/GIT/libkine/model/puma/link4.STL");
    puma.AddLink(0, -M_PI_2, 0, 0, Link::REVOLUTE, "/home/manurunga/Documents/GIT/libkine/model/puma/link5.STL");
    puma.AddLink(0, M_PI_2, 0, 0, Link::REVOLUTE);

    pthread_t thread;
    int ret = pthread_create(&thread, NULL, simulation, (void *) &puma);

    Graphic G(&puma, "Puma");
    G.SetGraphicScaling(0.15);
    G.SetOpacity(0.5);
    G.Run();  // Blocks here until VTK GUI is closed

    running = 0;
    pthread_join(thread, NULL);

}