//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_LINK_H
#define LIBKINE_LINK_H

#include <armadillo>

using namespace std;
using namespace arma;

class Link {
public:
    enum joint_t {
        REVOLUTE = 0, 
        PRISMATIC = 1
    };

    // Define the DH-parameters
    Link(double a, double alpha, double d, double theta, joint_t type = REVOLUTE);

    ~Link();

    // Print out the transformation matrix in global coordinate frame
    void PrintGlobalTransformationMatrix();

    // Perform actuation, either rotational or prismatic
    void ActuateJoint(double v);

    // Apply the homogeneous transformation matrix in global coordinate frame 
    void SetGlobalTransformation(mat &T);

    // Apply the homogeneous transformation matrix in local coordinate frame
    void SetLocalTransformation(mat &T);

    // Set the joint id
    void SetId(int id);

    // Provide value for theta (for rotational joint)
    void SetTheta(double theta);

    // Provide value for d (for translational joint)
    void SetD(double d);

    // Provide an STL file
    void SetSTLFileName(const char *fn);

    // Set the STL color
    void SetColor(char c);

	// Set joint limits
	void SetJointLimits(double min, double max);

    // Return the joint id
    int GetJointId();

    // Return a
    double GetA();

    // Return alpha
    double GetAlpha();

    // Return d
    double GetD();

    // Return theta
    double GetTheta();

    // Get rotation of the tip in global coordinate frame
    void GetRotation(mat &R);

    // Get position of the tip in global coordinate frame
    void GetRotation(double R[3][3]);

    // Get position of the tip in global coordinate frame
    void GetPosition(mat &T);

    // Get position of the tip in global coordinate frame
    void GetPosition(double p[3]);

    // In global coordinate frame, measured at the tip
    void GetTransformation(mat &A);

    // In local coordinate frame, measured at the tip
    void GetLocalTransformation(mat &Tlocal);

    // Return link ID
    int GetId();

    // Get the file name of the STL file
    const char *GetSTLFileName();

    // Get the defined color for the STL model
    char GetColor();

	// Get joint type
	joint_t GetJointType();

	// Get joint minimum range
	double GetMinJointLimit();

	// Get joint maximum range
	double GetMaxJointLimit();



private:

    // Joint id
    int m_Id;

    // Rotation about the xi-1 axis
    double m_A;

    // Translation along the x_(i-1) axis 
    double m_Alpha;

    // Translation along the z_i axis
    double m_D;

    // Joint angle
    double m_Theta;

    // Global homogeneous transformation matrix, in global coordinate frame.
    mat m_T;

    // Local homogeneous transformation matrix, in local body coordinate frame.
    mat m_Tlocal;

    // 3D model of the link
    string m_STLFileName;

    // Link color as rgb
    char m_Color;

    joint_t m_Type;

	// Maximum joint range
	double m_JointMax;

	// Minimum joint range
	double m_JointMin;
};


#endif //LIBKINE_LINK_H
