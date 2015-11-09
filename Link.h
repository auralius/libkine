//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_LINK_H
#define LIBKINE_LINK_H

#include <stdio.h>
#include <armadillo>

using namespace std;
using namespace arma;

class Link {
public:
    enum joint_t { REVOLUTE, PRISMATIC };

    Link(double a, double alpha, double d, double theta, joint_t type );
    ~Link();

    void PrintGlobalTransformationMatrix();
    
    void SetGlobalTransformation(mat &T);
    void SetLocalTransformation(mat &T);
    void SetId(int id);
    
    void SetTheta(double theta);
    void SetD(double d);
    
    double GetA();
    double GetAlpha();
    double GetD();
    double GetTheta();
    
    /**
     * In global coordinate frame
     */
    void GetRotationMatrix(mat &R);
    void GetTranslationMatrix(mat &T);
    void GetTransformationMatrix(mat &A);
   
     /**
     * In local coordinate frame
     */
    void GetLocalTransformationMatrix(mat &Alocal);

private:
        
    int m_id;
    
    double m_a;
    double m_alpha;
    double m_d;
    double m_theta;
    joint_t m_type;

    /**
     * Global Homogenous transformation matrix, in global coordinate 
     * frame.
     */
    mat m_A;
    
    /**
     * Local Homogenous transformation matrix, in local body coordinate
     * frame.
     */
    mat m_Alocal;


};


#endif //LIBKINE_LINK_H
