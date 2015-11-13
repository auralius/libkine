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
    ~Robot();

    void AddLink(double a, double alpha, double d, double theta, Link::joint_t type);

    void Update();

    void SetTheta(int n_link, double theta);

    void SetD(int n_link, double d);

    vector<Link *> GetLinks();

private:
    void CalcTransformationMatrix(Link &l, mat &A);

    vector<Link *> m_Links;
};


#endif //LIBKINE_ROBOT_H
