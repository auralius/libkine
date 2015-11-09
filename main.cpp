//////
//
// Example 1.4 - Cone And Viewer
//
//


#include <mgl2/qt.h>
#include <unistd.h>

#include "Link.h"
#include "Robot.h"
#include "Graphic.h"

int draw (mglGraph *gr)
{
    mglPoint pnt;
    pnt = mglPoint(2*mgl_rnd()-1,2*mgl_rnd()-1, 2*mgl_rnd());
    gr->Line(mglPoint(0,0,0),mglPoint(1, 1,1));
}


int main(int, char **argv)
{
  //      a,      alpha,   d,       theta, type
  
//   Link l1(0,      -M_PI_2, 0,       0,     Link::REVOLUTE);
//   Link l2(0.4318, 0,       0.14909, 0,     Link::REVOLUTE);
//   Link l3(0.023,  M_PI_2,  0,       0,     Link::REVOLUTE);
//   Link l4(0,      -M_PI_2, 0.43307, 0,     Link::REVOLUTE);
//   Link l5(0,      M_PI_2,  0,       0,     Link::REVOLUTE);
//   Link l6(0,      0,       0.05625, 0,     Link::REVOLUTE);
//   
  
  Link l1(0,      0,       0,       0,     Link::REVOLUTE);
  Link l2(0.5,    0,       0,       0,     Link::REVOLUTE);
  Link l3(0.5,    0,       -0.5,       0,     Link::REVOLUTE);
  

  Robot puma;
  puma.AddLink(l1);
  puma.AddLink(l2);
  puma.AddLink(l3);
//   puma.AddLink(l4);
//   puma.AddLink(l5);
//   puma.AddLink(l6);
  
  Graphic G(&puma);
  
  



   // mglQT gr(draw,"MathGL examples");
   // return gr.Run();

  
  
 
  // return 0;
  
}
