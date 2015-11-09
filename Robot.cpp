//
// Created by manurunga on 11/8/15.
//

#include "Robot.h"


Robot::~Robot() {
    // Cleaning up here;
    m_Links.clear();
}

void Robot::AddLink(Link &l) {
    m_Links.push_back(&l);
    l.SetId(m_Links.size());
    
    mat A;    
    CalcTransformationMatrix(l, A); 
    l.SetLocalTransformation(A);
    
    if (m_Links.size() > 1) {
        mat A_prev;
        m_Links.at(m_Links.size() - 2)->GetTransformationMatrix(A_prev);
        A = A_prev * A; 
    }
        
    l.SetGlobalTransformation(A);
}

void Robot::Update()
{
    
}

vector <Link *> Robot::GetLinks() {
    return m_Links;
}

void Robot::CalcTransformationMatrix(Link &l, mat &A) {
    mat Rz(4,4);
    mat Tz(4,4);
    mat Tx(4,4);
    mat Rx(4,4);

    double theta = l.GetTheta();
    double a = l.GetA();
    double alpha = l.GetAlpha();
    double d = l.GetD();


    Rz << cos(theta) << -sin(theta)  << 0 << 0 << endr
       << sin(theta) << cos(theta)   << 0 << 0 << endr
       << 0          << 0            << 1 << 0 << endr
       << 0          << 0            << 0 << 1 << endr;

    Tz << 1 << 0 << 0 << 0 << endr
       << 0 << 1 << 0 << 0 << endr
       << 0 << 0 << 1 << d << endr
       << 0 << 0 << 0 << 1 << endr;

    Tx << 1 << 0 << 0 << a << endr
       << 0 << 1 << 0 << 0 << endr
       << 0 << 0 << 1 << 0 << endr
       << 0 << 0 << 0 << 1 << endr;

    Rx << 1 << 0          << 0           << 0 << endr
       << 0 << cos(alpha) << -sin(alpha) << 0 << endr
       << 0 << sin(alpha) << cos(alpha)  << 0 << endr
       << 0 << 0          << 0           << 1 << endr;

    A = Rz * Tz * Tx * Rx;



}


