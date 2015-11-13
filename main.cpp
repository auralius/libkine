#include "Robot.h"
#include "Graphic.h"

int running = 1;

void *simulation(void *ptr) {
    Robot *puma = (Robot *) ptr;

    double x = 0;
    while (running) {

        puma->SetTheta(2, x);
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
    //      a, alpha, d, theta, type
    puma.AddLink(0, 0, 0, 0, Link::REVOLUTE);
    puma.AddLink(0, -M_PI_2, d2, 0, Link::REVOLUTE);
    puma.AddLink(a2, 0, d3, 0, Link::REVOLUTE);
    puma.AddLink(a3, M_PI_2, d4, 0, Link::REVOLUTE);
    puma.AddLink(0, -M_PI_2, 0, 0, Link::REVOLUTE);
    puma.AddLink(0, M_PI_2, 0, 0, Link::REVOLUTE);

    pthread_t thread;
    int ret = pthread_create(&thread, NULL, simulation, (void *) &puma);

    Graphic G(&puma);
    G.SetGraphicScaling(0.05);
    G.Run();  // Blocks here until VTK GUI is closed

    running = 0;
    pthread_join(thread, NULL);

}