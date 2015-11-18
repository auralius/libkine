//
// Created by manurunga on 11/8/15.
//

#include "Robot.h"

Robot::Robot() {

}

Robot::~Robot() {
    // Cleaning up here;
    for (unsigned int i = 0; i < m_Links.size(); i++)
        delete m_Links.at(i);

    m_Links.clear();
}

void Robot::AddLink(double a, double alpha, double d, double theta, Link::joint_t type, char * stl_fn) {
    Link *l = new Link(a, alpha, d, theta, type);
    m_Links.push_back(l);

    l->SetId(m_Links.size());

    if (stl_fn)
        l->SetSTLFileName(stl_fn);

    mat A;
    CalcTransformationMatrix(*l, A);
    l->SetLocalTransformation(A);

    if (m_Links.size() > 1) {
        mat A_prev;
        m_Links.at(m_Links.size() - 2)->GetTransformation(A_prev);
        A = A_prev * A;
    }

    l->SetGlobalTransformation(A);

}

void Robot::Update() {
    for (unsigned int i = 0; i < m_Links.size(); i++) {
        Link *l = m_Links.at(i);

        mat A;
        CalcTransformationMatrix(*l, A);

        l->SetLocalTransformation(A);

        // Here we calculate the global transformation matrix
        if (i > 0) {
            mat A_prev;
            m_Links.at(i - 1)->GetTransformation(A_prev);
            A = A_prev * A;
        }
        l->SetGlobalTransformation(A);
    }
}

void Robot::SetTheta(int n_link, double theta) {
    m_Links.at(n_link)->SetTheta(theta);
}

void Robot::SetBaseSTLFileName(const char *fn)
{
    m_BaseSTLFileName = fn;
}

vector<Link *> Robot::GetLinks() {
    return m_Links;
}

void Robot::CalcTransformationMatrix(Link &l, mat &A) {
    double theta = l.GetTheta();
    double a = l.GetA();
    double alpha = l.GetAlpha();
    double d = l.GetD();

    A << cos(theta) << -sin(theta) << 0 << a << endr
        << sin(theta) * cos(alpha) << cos(theta) * cos(alpha) << -sin(alpha) << -sin(alpha) * d << endr
        << sin(theta) * sin(alpha) << cos(theta) * sin(alpha) << cos(alpha) << cos(alpha) * d << endr
        << 0 << 0 << 0 << 1 << endr;
}


void Robot::SetD(int n_link, double d) {
    m_Links.at(n_link)->SetD(d);
}

void Robot::GetTipPosition(int n_link, mat &p) {
    m_Links.at(n_link)->GetPosition(p);
}

void Robot::GetJointPosition(int n_link, mat &p) {
    if (n_link > 0)
        m_Links.at(n_link - 1)->GetPosition(p);
    else
        p << 0 << endr << 0 << endr << 0 << endr;
}

void Robot::GetTipTransformation(int n_link, mat &A) {
    m_Links.at(n_link)->GetTransformation(A);
}

void Robot::GetJointTransformation(int n_link, mat &A) {
    m_Links.at(n_link)->GetTransformation(A);

    mat p;
    GetJointPosition(n_link, p);

    for (int i = 0; i < 3; i++)
        A.at(i, 3) = p.at(i, 0);
}

void Robot::GetTipPosition(int n_link, double p[3]) {
    mat T;
    GetTipPosition(n_link, T);

    for (int i = 0; i < 3; i++)
        p[i] = T.at(i, 0);
}

void Robot::GetJointPosition(int n_link, double p[3]) {
    mat T;
    GetJointPosition(n_link, T);

    for (int i = 0; i < 3; i++)
        p[i] = T.at(i, 0);
}

const char* Robot::GetBaseSTLFileName() {
    if (m_BaseSTLFileName.size() == 0)
        return NULL;

    return m_BaseSTLFileName.c_str();
}
