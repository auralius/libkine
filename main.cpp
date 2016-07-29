#include "Robot.h"
#include "Graphic.h"
#include "JointSliders.h"


int main(int argc, char **argv) {

    //Robot puma;
    Robot robot(argv[1]);
    robot.SetLogFileName("MyRobot.log");

    JointSliders js(&robot, 1); // with logging

    Graphic G(&robot, "MyRobot");
    G.SetGraphicScaling(0.15);
    G.SetOpacity(1);
    G.Run();  // Blocks here until VTK GUI is closed

    js.End();
}