//
// Created by manurunga on 11/8/15.
//

#include "Robot.h"

Robot::Robot() {
    m_DoLogging = 0;
    m_SimulationTime = 0;
    m_SimulationRate = 0.001; // 1khz initial sampling rate
    m_BasePos.zeros(3, 1);

    m_LoggingStream << fixed;
    m_LoggingStream << setprecision(5);
}

Robot::~Robot() {
    // Cleaning up here;
    for (unsigned int i = 0; i < m_Links.size(); i++)
        delete m_Links.at(i);

    m_Links.clear();

    if (m_LoggingStream.is_open())
        m_LoggingStream.close();
}

void Robot::AddLink(double a, double alpha, double d, double theta, 
    Link::joint_t type, char * stl_fn, char c) {
    Link *l = new Link(a, alpha, d, theta, type);
    m_Links.push_back(l);

    l->SetId(m_Links.size()-1);

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

    l->SetColor(c);
}

void Robot::Update(int verbose) {
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

    if (verbose) 
        DoVerbosity();

    m_SimulationTime = m_SimulationTime + m_SimulationRate;
}

void Robot::DoVerbosity()
{
 
    double R[3][3];
    double p[3];
    GetEndEffectorRotation(R);
    GetEndEffectorPosition(p);

    m_LoggingStream << "ee_pos\t" << m_SimulationTime << '\t' << p[0] << '\t' 
        << p[1] << '\t' << p[2] << endl;

    m_LoggingStream << "ee_rot\t" << m_SimulationTime << '\t';
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            m_LoggingStream << R[i][j] << '\t';
        }
    }
    m_LoggingStream << endl;

}

void Robot::SetTheta(int n_link, double theta) {
    m_Links.at(n_link)->SetTheta(theta);
}

void Robot::SetBaseSTLFileName(const char *fn)
{
    m_BaseSTLFileName = fn;
}

void Robot::SetLogFileName(const char *fn) {
    m_LoggingStream.open(fn);
    if (m_LoggingStream.is_open())
        m_DoLogging = 1;
}

void Robot::SetBasePosition(double x, double y, double z)
{
    m_BasePos.set_size(3, 1);
    m_BasePos << x << endr << y << endr << z << endr;
}

vector<Link *> Robot::GetLinks() {
    return m_Links;
}

void Robot::SetSimulationRate(double rate)
{
    m_SimulationRate = rate;
}

void Robot::CalcTransformationMatrix(Link &l, mat &A) {
    double theta = l.GetTheta();
    double a = l.GetA();
    double alpha = l.GetAlpha();
    double d = l.GetD();

    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    A << ct << -st << 0 << a << endr
        << st * ca << ct * ca << -sa << -sa * d << endr
        << st * sa << ct * sa << ca << ca * d << endr
        << 0 << 0 << 0 << 1 << endr;

    if (l.GetId() == 0)
        A.submat(0, 3, 2, 3) = A.submat(0, 3, 2, 3) + m_BasePos;
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
        p = m_BasePos;
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

void Robot::GetEndEffectorPosition(double p[3]) {
    m_Links.at(m_Links.size() - 1)->GetPosition(p);
}

void Robot::GetEndEffectorRotation(double R[3][3]) {
    m_Links.at(m_Links.size() - 1)->GetRotation(R);
}

void Robot::GetBasePosition(double p[3])
{
    for (size_t i = 0; i < 3; i++)
        p[i] = m_BasePos.at(i, 0);
}

const char* Robot::GetBaseSTLFileName() {
    if (m_BaseSTLFileName.size() == 0)
        return NULL;

    return m_BaseSTLFileName.c_str();
}