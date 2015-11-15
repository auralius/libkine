//
// Created by manurunga on 11/8/15.
//

#ifndef LIBKINE_GRAPHIC_H
#define LIBKINE_GRAPHIC_H

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

#include "Robot.h"

/*---------------------------------------------------------------------*/

class vtkTimerCallback : public vtkCommand {
public:
    static vtkTimerCallback *New();

    virtual void Execute(vtkObject *caller, unsigned long eventId,
                         void * vtkNotUsed(callData));

private:
    vector <vtkSmartPointer<vtkTransform> > m_Transforms;

    void ArmaMatToVTKMat(vtkSmartPointer<vtkMatrix4x4> &to, mat &from);
    
public:
    vector<vtkSmartPointer<vtkAxesActor> > *m_AxesActors;
    vector<vtkSmartPointer<vtkLineSource> > *m_LineSources;
    vector<vtkSmartPointer<vtkActor> > *m_STLActors;
    Robot *m_Robot;
};

/*---------------------------------------------------------------------*/

class Graphic {
public:
    Graphic(Robot *robot, const char *window_title);

    ~Graphic();

    void SetGraphicScaling(double K);
    
    void SetSTLVisibility(int flag);
    
    void SetOpacity(double opacity);

    void Run();

private:
    void CreateAxes();

    void CreateLinks();

    void CreateSTLs();
    
    void RenderBase();

    Robot *m_Robot;

    double m_K;
    
    int m_STLVisibility;
    
    double m_Opacity;
    
    string m_WindowTitle;

    vtkRenderer *m_Ren;
    vtkRenderWindow *m_RenWin;
    vtkRenderWindowInteractor *m_Iren;

    vector<vtkSmartPointer<vtkAxesActor> > m_AxesActors;
    vector<vtkSmartPointer<vtkLineSource> > m_LineSources;
    vector<vtkSmartPointer<vtkActor> > m_STLActors;
};


#endif //LIBKINE_GRAPHIC_H
