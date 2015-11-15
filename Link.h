//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_LINK_H
#define LIBKINE_LINK_H

#include <stdio.h>
#include <armadillo>

using namespace std;
using namespace arma;

class Link {
public:
    enum joint_t {
        REVOLUTE, PRISMATIC, WELD
    };

    Link(double a, double alpha, double d, double theta, joint_t type);

    ~Link();

    void PrintGlobalTransformationMatrix();

    void SetGlobalTransformation(mat &T);

    void SetLocalTransformation(mat &T);

    void SetId(int id);

    void SetTheta(double theta);

    void SetD(double d);

    void SetSTLFileName(const char *fn);

    double GetA();

    double GetAlpha();

    double GetD();

    double GetTheta();

    /**
     * In global coordinate frame, measured at the tip
     */
    void GetRotation(mat &R);

    void GetPosition(mat &T);

    void GetTransformation(mat &A);

    /**
    * In local coordinate frame, measured at the tip
    */
    void GetLocalTransformation(mat &Alocal);

    const char *GetSTLFileName();

private:

    int m_id;

    double m_a;
    double m_alpha;
    double m_d;
    double m_theta;
    joint_t m_type;

    /**
     * Global Homogeneous transformation matrix, in global coordinate
     * frame.
     */
    mat m_A;

    /**
     * Local Homogeneous transformation matrix, in local body coordinate
     * frame.
     */
    mat m_Alocal;

    string m_STLFileName;


};


#endif //LIBKINE_LINK_H
