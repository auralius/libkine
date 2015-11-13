//
// Created by manurunga on 11/8/15.
//

#include "Robot.h"


Robot::~Robot() {
    // Cleaning up here;
    for (int i = 0; i < m_Links.size(); i++)
        delete m_Links.at(i);

    m_Links.clear();
}

void Robot::AddLink(double a, double alpha, double d, double theta, Link::joint_t type) {
    Link *l = new Link(a, alpha, d, theta, type);
    m_Links.push_back(l);

    l->SetId(m_Links.size());

    mat A;
    CalcTransformationMatrix(*l, A);
    l->SetLocalTransformation(A);

    if (m_Links.size() > 1) {
        mat A_prev;
        m_Links.at(m_Links.size() - 2)->GetTransformationMatrix(A_prev);
        A = A_prev * A;
    }

    l->SetGlobalTransformation(A);

}

void Robot::Update() {
    for (int i = 0; i < m_Links.size(); i++) {
        Link *l = m_Links.at(i);

        mat A;
        CalcTransformationMatrix(*l, A);

        l->SetLocalTransformation(A);

        // Here we calculate the global transformation matrix
        if (i > 0) {
            mat A_prev;
            m_Links.at(i - 1)->GetTransformationMatrix(A_prev);
            A = A_prev * A;
        }
        l->SetGlobalTransformation(A);
    }
}

void Robot::SetTheta(int n_link, double theta) {
    m_Links.at(n_link)->SetTheta(theta);
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
