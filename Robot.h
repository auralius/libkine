//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_ROBOT_H
#define LIBKINE_ROBOT_H

#include <unistd.h>
#include <vector>

#include "Link.h"

class Robot {
public:
    Robot ();
    
    ~Robot();

    void AddLink(double a, double alpha, double d, double theta, Link::joint_t type, char * stl_fn = NULL);

    void Update();

    void SetTheta(int n_link, double theta);

    void SetD(int n_link, double d);
    
    void SetBaseSTLFileName(const char *fn);

    vector<Link *> GetLinks();

    void GetTipPosition(int n_link, mat &T);

    void GetTipPosition(int n_link, double p[3]);

    void GetJointPosition(int n_link, mat &T);

    void GetJointPosition(int n_link, double p[3]);

    void GetTipTransformation(int n_link, mat &A);

    void GetJointTransformation(int n_link, mat &A);
    
    const char *GetBaseSTLFileName();

private:
    void CalcTransformationMatrix(Link &l, mat &A);

    vector<Link *> m_Links;
    
    mat m_Temp_mat;
    
    mat m_BasePos;
    
    string m_BaseSTLFileName;
};


#endif //LIBKINE_ROBOT_H
