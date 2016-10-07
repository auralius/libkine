//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_GRAPHIC_H
#define LIBKINE_GRAPHIC_H

#ifdef WIN32
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#endif

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkAxesActor.h>
#include <vtkCommand.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkLineSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCaptionActor2D.h>
#include <vtkSTLReader.h>
#include <vtkFeatureEdges.h>
#include <vtkRenderWindowInteractor.h>

#include "Robot.h"

/*---------------------------------------------------------------------*/

class vtkTimerCallback : public vtkCommand {
public:
    // Create the callback function
    static vtkTimerCallback *New();

    // Callback function to call
    virtual void Execute(vtkObject *caller, unsigned long eventId, 
        void * vtkNotUsed(callData));

private:
    // A vector contains transformation matrices of all links, stored in 
    // ascending order
    vector <vtkSmartPointer<vtkTransform> > m_Transforms;

    // convert armadillo matrix to vtk matrix
    void ArmaMatToVTKMat(vtkSmartPointer<vtkMatrix4x4> &to, mat &from);

public:
    // Vector for all robots
    vector<Robot *> *m_RobotList;
    
    //
    // Vector to hold the actors for the 3D axes and the STL files
    //
    vector<vector<vtkSmartPointer<vtkAxesActor> > > *m_AxesActorsList;
    vector<vector<vtkSmartPointer<vtkActor> > > *m_STLActorsList;
    
    // Vector to hold the created lines
    vector<vector<vtkSmartPointer<vtkLineSource> > > *m_LineSourcesList; 
    
    // Vector to hold the boundary edges
    vector<vector<vtkSmartPointer<vtkActor> > > *m_EdgeActorsList; 
};

/*---------------------------------------------------------------------*/

class Graphic {
public:
    // Create the graphic window
    Graphic(const char *window_title);

    ~Graphic();
    
    void AddRobot(Robot *robot);

    // Define the scaling for drawing the 3D coordinate axes
    void SetGraphicScaling(double K);

    // Show/hide the STL models
    void SetSTLVisibility(int flag);

    // Set transparency of the STL models
    void SetOpacity(double opacity);

    // Set camera distance
    void SetCameraDistance(double dist);

    // Run the graphic
    void Run();

private:
    // Create the 3D axes
    void CreateAxes();

    // Create links, which are lines that connect one joint to another joint
    void CreateLinks();

    // Create the 3D models for the given STL files
    void CreateSTLs();

    // Another STL file can be provided for the base of the robot
    void RenderBase();
    
    
    void RenderGridFloor(double grid_dimension, double step);

    void RenderGlobalAxis();

    void Rgb(char c, double color[3]);

    // Pointer to the robot
    //Robot *m_Robot;

    // the scaling factor for the 3D axes
    double m_K;

    // Flag for the visibility of the 3D modes
    int m_STLVisibility;

    // Flag for the opacity of the 3D modes
    double m_Opacity;

    // the window title
    string m_WindowTitle;

    // How far in the intial position of the camera
    double m_CamDist;

    //
    // All variables need to render a vtk windows
    //
    vtkRenderer *m_Ren;
    vtkRenderer *m_RenSTL;
    vtkRenderWindow *m_RenWin;
    vtkRenderWindowInteractor *m_Iren;
    
    // Vector for all robots
    vector<Robot *>m_RobotList;
    
    //
    // Vector to hold the actors for the 3D axes and the STL files
    //
    vector<vector<vtkSmartPointer<vtkAxesActor> > > m_AxesActorsList;
    vector<vector<vtkSmartPointer<vtkActor> > > m_STLActorsList;
    
    // Vector to hold the created lines
    vector<vector<vtkSmartPointer<vtkLineSource> > > m_LineSourcesList; 
    
    // Vector to hold the boundary edges
    vector<vector<vtkSmartPointer<vtkActor> > > m_EdgeActorsList;   
};


#endif //LIBKINE_GRAPHIC_H
