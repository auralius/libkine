//
// Created by manurunga on 11/8/15.
//

#include "Link.h"

Link::Link(double a, double alpha, double d, double theta, joint_t type) 
{
    m_A = a;
    m_Alpha = alpha;
    m_D = d;
    m_Theta = theta;
    m_Type = type;
    m_Color = 'e';
}

Link::~Link() {
}

void Link::PrintGlobalTransformationMatrix() {
    cout << "T" << m_Id <<" =\n";
    m_T.print();
}

void Link::ActuateJoint(double v) {
    if (m_Type == REVOLUTE)
        SetTheta(v);
    else if (m_Type == PRISMATIC)
        SetD(v);
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

void Link::SetColor(char c) {
     m_Color = c;
}

void Link::SetJointLimits(double min, double max) {
	if (min < max) {
		m_JointMin = min;
		m_JointMax = max;
	}
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
    R.set_size(3, 3);
    R << m_T.at(0, 0) << m_T.at(0, 1) << m_T.at(0, 2) << endr
        << m_T.at(1, 0) << m_T.at(1, 1) << m_T.at(1, 2) << endr
        << m_T.at(2, 0) << m_T.at(2, 1) << m_T.at(2, 2) << endr;
}

void Link::GetRotation(double R[3][3])
{
    mat r;
    GetRotation(r);
    
    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 3; j++) 
            R[i][j] = r.at(i, j);
}

void Link::GetPosition(mat &T) {
    //m_A.print("A=");
    T.set_size(3, 1);
    T << m_T.at(0, 3) << endr
        << m_T.at(1, 3) << endr
        << m_T.at(2, 3) << endr;
}

void Link::GetPosition(double p[3]) {
    mat pos;
    GetPosition(pos);

    for (size_t i = 0; i < 3; i++) 
        p[i] = pos.at(i, 0);       
}

void Link::GetTransformation(mat &A) {
    A = m_T;
}

void Link::GetLocalTransformation(mat &Alocal) {
    Alocal = m_Tlocal;
}

int Link::GetId()
{
    return m_Id;
}

const char *Link::GetSTLFileName() {
    if (m_STLFileName.size() == 0)
        return NULL;

    return m_STLFileName.c_str();
}

char Link::GetColor() {
    return m_Color;
}

Link::joint_t Link::GetJointType() {
	return m_Type;
}

double Link::GetMinJointLimit() {
	return m_JointMin;
}

double Link::GetMaxJointLimit() {
	return m_JointMax;
}
