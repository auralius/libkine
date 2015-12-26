//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_ROBOT_H
#define LIBKINE_ROBOT_H

#ifndef WIN32
#include <unistd.h>
#endif

#ifdef WIN32
#define _USE_MATH_DEFINES
#define NOMINMAX
#include <windows.h>
#include <pthread.h>
#endif

#include <iomanip>
#include <vector>
#include "Link.h"
#include <string>

class Robot {
public:
    // Constructor to load the DH parameters from an external file
    Robot(const char *fn);

	// Constructor create the robot programatically
    Robot();
    
    // Destructor
	~Robot();

    // Add a new link, given the DH parameter of the link
    void AddLink(double a, double alpha, double d, double theta, 
		Link::joint_t type = Link::REVOLUTE, const char *stl_fn = NULL, 
		char c = 'e');

    // After the joint angle (theta) or translation (d) is updated, call this  
    // function to recalculate the transformation matrix of all links
    void Update(int verbose = 0);

    // Actuate the joint, depending on the joint type: REVOLUTE of PRISMATIC
    void ActuateJoint(int n_link, double v);

    // Set an STL file to render for the current link
    void SetBaseSTLFileName(const char *fn);

    // Set logging file name
    void SetLogFileName(const char *fn);

    void SetBasePosition(double x, double y, double z);

    // Get the vector that hold all links together
    vector<Link *> GetLinks();

    // Set the simulation sampling rate
    void SetSimulationRate(double rate);

    // Given a link, find its tip position
    void GetTipPosition(int n_link, mat &p);

    // Given a link, find its tip position
    void GetTipPosition(int n_link, double p[3]);

    // Given a link, find its tip position, measured from global frame
    void GetJointPosition(int n_link, mat &p);

    // Given a link, get its joint position, measured from global frame
    void GetJointPosition(int n_link, double p[3]);

    // Given a link, get its homogeneous transformation matrix at the tip, 
    // measured from global frame
    void GetTipTransformation(int n_link, mat &A);

    // Given a link, get its homogeneous transformation matrix at the joint, 
    // measured from global frame
    void GetJointTransformation(int n_link, mat &A);

    // Get the STL file name, return NULL if it not defined
    const char *GetBaseSTLFileName();

    // Get the end effector position;
    void GetEndEffectorPosition(double p[3]);

    // Get the rotation matrix of the end effector;
    void GetEndEffectorRotation(double R[3][3]);

    /// Get base position of the robot
    void GetBasePosition(double p[3]);

private:
    // Iinitialize variables
    void InitVariables();

    // Calculate the link transformation matrix, as soon as it is added 
    void CalcTransformationMatrix(Link &l, mat &A);

    // Print out interesting data
    void DoVerbosity();

	// Set the joint position
	void SetTheta(int n_link, double theta);

	// Set the joint translation
	void SetD(int n_link, double d);

    // Vector to hold the links
    vector<Link *> m_Links;

    // Base position of the robot
    mat m_BasePos;

    // File name for STL file of the robot's base
    string m_BaseSTLFileName;

    // Simulation time
    double m_SimulationTime;

    // Sampling rate for the simulation
    double m_SimulationRate;

    // Stream for data logging
    ofstream m_LoggingStream;

    // Flag for logging
    int m_DoLogging;

};


#endif //LIBKINE_ROBOT_H
