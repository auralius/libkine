//
// Created by manurunga on 11/8/15.
//

#include "Robot.h"

Robot::Robot() {
    InitVariables();
}

Robot::Robot(const char *fn) {
    InitVariables();
    
    ifstream file(fn);
    if (!file.is_open())
        throw std::invalid_argument("File not found!");

    string line;
    int line_num = 0;
    
    while (!file.eof()) {
        while (1) {
            getline(file, line);
            
            if (file.eof())
                break;

            // Ignore all spaces
			line.erase(
				remove_if(line.begin(), line.end(), static_cast<int(*)(int)>(isspace)),
				line.end());
            
            // Ignore empty and commented lines
            if (!line.empty())
                if (line.at(0) != '#')
                    break;
        }
        
        istringstream ss(line);

        double base_p[3] = { 0, 0, 0 };
        double a = 0;
        double alpha = 0;
        double d = 0;
        double theta = 0;
		Link::joint_t type = Link::REVOLUTE; 
        char color = 'w';
        string stl_fn;
		double min_joint_limit = -M_PI;
		double max_joint_limit = M_PI;

        string token;
        size_t index = 0;
		token = "";

        line_num = line_num + 1;

        // Load the base coordinate and its STL file
        if (line_num == 1) {
            while (getline(ss, token, ',')) {
                if (index == 0)
                    base_p[0] = stod(token);
                else if (index == 1)
                    base_p[1] = stod(token);
				else if (index == 2)
					base_p[2] = stod(token);
				else if (index == 3) {
					stl_fn = token;
				}

                index = index + 1;
            }

            if (strlen(stl_fn.c_str()) > 1)
				SetBaseSTLFileName(stl_fn.c_str());

            SetBasePosition(base_p[0], base_p[1], base_p[2]);
        }

        // Load the DH Parameters and their STL files
        else if (line_num > 1) {
            while (getline(ss, token, ',')) {
                if (index == 0)
                    a = stod(token);
                else if (index == 1)
                    alpha = stod(token);
                else if (index == 2)
                    d = stod(token);
                else if (index == 3)
                    theta = stod(token);
				else if (index == 4)
					type = (Link::joint_t) stoi(token);
				else if (index == 5)
					stl_fn = token;
                else if (index == 6)
                    color = *token.c_str();
				else if (index == 7)
					min_joint_limit= stod(token);
				else if (index == 8)
					max_joint_limit = stod(token);

                index = index + 1;
            }

            if (stl_fn.length() > 1)
                AddLink(a, alpha / 180 * M_PI, d, theta, type, stl_fn.c_str(), color);
            else
                AddLink(a, alpha / 180 * M_PI, d, theta, type);

/*			if (type == Link::REVOLUTE) {
				min_joint_limit = min_joint_limit  * M_PI;
				max_joint_limit = max_joint_limit / 180 * M_PI;
			}*/

			m_Links.at(m_Links.size() - 1)->SetJointLimits(min_joint_limit, max_joint_limit);
        }
    }
    file.close();
}

void Robot::InitVariables() {
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
	Link::joint_t type, const char * stl_fn, char c) {
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

void Robot::ActuateJoint(int n_link, double v) {
	if (m_Links.at(n_link)->GetJointType() == Link::REVOLUTE)
		SetTheta(n_link, v);
	else if (m_Links.at(n_link)->GetJointType() == Link::PRISMATIC)
		SetD(n_link, v);
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
    
    cout << p[0] << '\t' << p[1] << '\t' << p[2] << endl;

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