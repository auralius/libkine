//
// Created by manurunga on 11/8/15.
//

#include "Graphic.h"


Graphic::Graphic(Robot* robot)
{
    m_K = 0.1;
    
    m_Robot = robot;
    
    mglQT gr;    
    DrawAxes(&gr);
    gr.Run();
}

void Graphic::SetK(double K)
{
    m_K = K;
}


int Graphic::DrawAxes(mglQT* gr)
{
    mglPoint pnt;
    pnt = mglPoint(2*mgl_rnd()-1,2*mgl_rnd()-1, 2*mgl_rnd());
    
    vector <Link *> links = m_Robot->GetLinks();
    
    mat x(3,1), y(3,1), z(3,1);
    x << 1 << endr 
      << 0 << endr 
      << 0 << endr;
	
    y << 0 << endr 
      << 1 << endr 
      << 0 << endr; 
	
    z << 0 << endr 
      << 0 << endr 
      << 1 << endr; 
      
    gr->Rotate(90, 0, 0);
    
    for (int i = 0; i < links.size(); i++) {      
	Link *l = links.at(i);
	mat R;
	mat T;
	l->GetRotationMatrix(R);
	l->GetTranslationMatrix(T);
	
	
	mat x_ = R * (x * m_K) + T;
	mat y_ = R * (y * m_K) + T;
	mat z_ = R * (z * m_K) + T;
	
	//T.print("T:");
	
	gr->Line(mglPoint(T.at(0,0), T.at(1,0), T.at(2,0)),mglPoint(x_.at(0,0), x_.at(1,0), x_.at(2,0)), "r");
	gr->Line(mglPoint(T.at(0,0), T.at(1,0), T.at(2,0)),mglPoint(y_.at(0,0), y_.at(1,0), y_.at(2,0)), "g");
	gr->Line(mglPoint(T.at(0,0), T.at(1,0), T.at(2,0)),mglPoint(z_.at(0,0), z_.at(1,0), z_.at(2,0)), "b");
        
        if (i < links.size() - 1) {
            Link *l_next = links.at(i+1);
            mat T_next;                        
            l_next->GetTranslationMatrix(T_next);
            //T_next.print("T_next:");
            gr->Line(mglPoint(T.at(0,0), T.at(1,0), T.at(2,0)),mglPoint(T_next.at(0,0), T_next.at(1,0), T_next.at(2,0)), "q2SS");
        }            	
    }    
    
    gr->Update();
}