//
// Created by manurunga on 11/8/15.
//

#include "Link.h"

Link::Link(double a, double alpha, double d, double theta, Link::joint_t type) {
    m_a = a;
    m_alpha = alpha;
    m_d = d;
    m_theta = theta;
    m_type = type;
}

Link::~Link() {

}

void Link::PrintGlobalTransformationMatrix() {
    char tmp[8];
    sprintf(tmp, "T(%i)=", m_id);
    m_A.print(tmp);
}

void Link::SetGlobalTransformation(mat &T) {
    //T.print("T");
    m_A = T;
}

void Link::SetLocalTransformation(mat &T) {
    m_Alocal = T;
}

void Link::SetId(int id) {
    m_id = id;
}

void Link::SetTheta(double theta) {
    m_theta = theta;
}

void Link::SetD(double d) {
    m_d = d;
}


double Link::GetA() {
    return m_a;
}

double Link::GetAlpha() {
    return m_alpha;
}

double Link::GetD() {
    return m_d;
}

double Link::GetTheta() {
    return m_theta;
}

void Link::GetRotationMatrix(mat &R) {
    R.set_size(4, 4);
    R << m_A.at(0, 0) << m_A.at(0, 1) << m_A.at(0, 2) << endr
    << m_A.at(1, 0) << m_A.at(1, 1) << m_A.at(1, 2) << endr
    << m_A.at(2, 0) << m_A.at(2, 1) << m_A.at(2, 2) << endr;
}

void Link::GetTranslationMatrix(mat &T) {
    //m_A.print("A=");
    T.set_size(4, 1);
    T << m_A.at(0, 3) << endr
    << m_A.at(1, 3) << endr
    << m_A.at(2, 3) << endr;
}

void Link::GetTransformationMatrix(mat &A) {
    A = m_A;
}

void Link::GetLocalTransformationMatrix(mat &Alocal) {
    Alocal = m_Alocal;
}


