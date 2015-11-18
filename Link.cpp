//
// Created by manurunga on 11/8/15.
//

#include "Link.h"

Link::Link(double a, double alpha, double d, double theta, Link::joint_t type) {
    m_A = a;
    m_Alpha = alpha;
    m_D = d;
    m_Theta = theta;
    m_Type = type;
}

Link::~Link() {
}

void Link::PrintGlobalTransformationMatrix() {
    char tmp[8];
    sprintf(tmp, "T(%i)=", m_Id);
    m_T.print(tmp);
}

void Link::SetGlobalTransformation(mat &T) {
    m_T = T;
}

void Link::SetLocalTransformation(mat &T) {
    m_Tlocal = T;
}

void Link::SetId(int id) {
    m_Id = id;
}

void Link::SetTheta(double theta) {
    m_Theta = theta;
}

void Link::SetD(double d) {
    m_D = d;
}

void Link::SetSTLFileName(const char *fn) {
    m_STLFileName = fn;
}

int Link::GetJointId() {
    return m_Id;
}

double Link::GetA() {
    return m_A;
}

double Link::GetAlpha() {
    return m_Alpha;
}

double Link::GetD() {
    return m_D;
}

double Link::GetTheta() {
    return m_Theta;
}

void Link::GetRotation(mat &R) {
    R.set_size(4, 4);
    R << m_T.at(0, 0) << m_T.at(0, 1) << m_T.at(0, 2) << endr
        << m_T.at(1, 0) << m_T.at(1, 1) << m_T.at(1, 2) << endr
        << m_T.at(2, 0) << m_T.at(2, 1) << m_T.at(2, 2) << endr;
}

void Link::GetPosition(mat &T) {
    //m_A.print("A=");
    T.set_size(4, 1);
    T << m_T.at(0, 3) << endr
        << m_T.at(1, 3) << endr
        << m_T.at(2, 3) << endr;
}

void Link::GetTransformation(mat &A) {
    A = m_T;
}

void Link::GetLocalTransformation(mat &Alocal) {
    Alocal = m_Tlocal;
}

const char *Link::GetSTLFileName() {
    if (m_STLFileName.size() == 0)
        return NULL;

    return m_STLFileName.c_str();
}
