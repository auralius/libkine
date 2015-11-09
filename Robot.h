//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_ROBOT_H
#define LIBKINE_ROBOT_H

#include <vector>
#include "Link.h"


class Robot {
public:
    ~Robot();
    void AddLink(Link &l);
    void Update();
    
    vector <Link *> GetLinks();

private:
    void CalcTransformationMatrix(Link &l, mat &A);

    vector <Link *> m_Links;
};


#endif //LIBKINE_ROBOT_H
