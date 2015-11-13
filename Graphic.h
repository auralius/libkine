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

#include "Robot.h"

/*---------------------------------------------------------------------*/

class vtkTimerCallback : public vtkCommand {
public:
    static vtkTimerCallback *New();

    virtual void Execute(vtkObject *caller, unsigned long eventId,
                         void * vtkNotUsed(callData));

private:

public:
    vector<vtkSmartPointer<vtkAxesActor> > *m_AxesActors;
    vector<vtkSmartPointer<vtkLineSource> > *m_LineSources;
    Robot *m_Robot;
};

/*---------------------------------------------------------------------*/

class Graphic {
public:
    Graphic(Robot *robot);

    ~Graphic();

    void SetGraphicScaling(double K);

    void Run();

private:
    void CreateAxes();

    void CreateLinks();

    Robot *m_Robot;

    double m_K;

    vtkRenderer *m_Ren;
    vtkRenderWindow *m_RenWin;
    vtkRenderWindowInteractor *m_Iren;

    vector<vtkSmartPointer<vtkAxesActor> > m_AxesActors;
    vector<vtkSmartPointer<vtkActor> > m_LineActors;
    vector<vtkSmartPointer<vtkLineSource> > m_LineSources;
};


#endif //LIBKINE_GRAPHIC_H
