/*
 * main2.m
 * Load 2 robots, and display the joint sliders. 
 * The robots are loaded externally from given arguments.
 * The arguments point to the location of the configuration files of the robots.
 */

#include "Robot.h"
#include "Graphic.h"
#include "JointSliders.h"


int main(int argc, char **argv) {
    Robot robot1(argv[1]);
    Robot robot2(argv[2]);
    robot1.SetLogFileName("MyRobot1.log");
    robot2.SetLogFileName("MyRobot2.log");
    
    robot1.SetBasePosition(0.5, 0, 0);
    robot1.Update();
    robot2.SetBasePosition(-0.5, 0, 0);
    robot2.Update();

    vector<Robot *> robots;
    robots.push_back(&robot1);
    robots.push_back(&robot2);
    JointSliders js(&robots); // with logging

    Graphic G("MyRobot");
    G.AddRobot(&robot1);
    G.AddRobot(&robot2);
    G.SetGraphicScaling(0.15);
    G.SetCameraDistance(5);
    G.SetOpacity(1);
    G.Run();  // Blocks here until VTK GUI is closed

    js.End();
}