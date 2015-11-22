//
// Created by manurunga on 11/8/15.
//

#include "Graphic.h"

/*---------------------------------------------------------------------------*/

vtkTimerCallback *vtkTimerCallback::New() {
    vtkTimerCallback *cb = new vtkTimerCallback;
    return cb;
}

void vtkTimerCallback::Execute(vtkObject *caller, unsigned long eventId, 
                               void *vtkNotUsed(callData)) {
    vector<Link *> links = m_Robot->GetLinks();
    vtkSmartPointer<vtkMatrix4x4> vtk_A_tip = 
        vtkSmartPointer<vtkMatrix4x4>::New();
    vtkSmartPointer<vtkMatrix4x4> vtk_A_joint = 
        vtkSmartPointer<vtkMatrix4x4>::New();
    mat A_tip;
    mat A_joint;

    double p0[3] = { 0, 0, 0 };
    double p1[3] = { 0, 0, 0 };

    for (size_t i = 0; i < links.size(); i++) {

        m_Robot->GetJointTransformation(i, A_joint);
        m_Robot->GetTipTransformation(i, A_tip);

        // Axes
        // Armadillo matrix to VTK matrix
        ArmaMatToVTKMat(vtk_A_tip, A_tip);
        ArmaMatToVTKMat(vtk_A_joint, A_joint);

        vtkSmartPointer<vtkTransform> transform_tip = 
            vtkSmartPointer<vtkTransform>::New();
        transform_tip->PostMultiply();
        transform_tip->SetMatrix(vtk_A_tip);

        m_AxesActors->at(i)->SetUserTransform(transform_tip);

        // Lines
        m_Robot->GetJointPosition(i, p0);
        m_Robot->GetTipPosition(i, p1);

        vtkLineSource *ls = m_LineSources->at(i);
        ls->SetPoint1(p0);
        ls->SetPoint2(p1);
        ls->Update();

        // STL
        if (links.at(i)->GetSTLFileName()) {
            vtkSmartPointer<vtkTransform> transform_joint = 
                vtkSmartPointer<vtkTransform>::New();
            transform_joint->PostMultiply();
            transform_joint->SetMatrix(vtk_A_tip);

            // Index zero is for the base, so we use i+1
            m_STLActors->at(i+1)->SetUserTransform(transform_joint);

            // Move the robot base to the desired base position
            if (i == 0) 
                m_STLActors->at(i)->SetPosition(p0);
        }
    }

    vtkRenderWindowInteractor *iren = 
        vtkRenderWindowInteractor::SafeDownCast(caller);
    iren->GetRenderWindow()->Render();
}


void vtkTimerCallback::ArmaMatToVTKMat(vtkSmartPointer<vtkMatrix4x4> &to, 
                                       mat &from) {
    for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++)
        to->SetElement(r, c, from.at(r, c));
}

/*---------------------------------------------------------------------------*/

Graphic::Graphic(Robot *robot, const char *window_title) {
    m_CamDist = 3;
    m_K = 0.05;
    m_Robot = robot;
    m_STLVisibility = 1;
    m_Opacity = 1;
    m_WindowTitle = window_title;

    m_Ren = vtkRenderer::New();
    m_RenSTL = vtkRenderer::New();

    m_RenWin = vtkRenderWindow::New();
    m_RenWin->SetNumberOfLayers(2);
    m_RenWin->AddRenderer(m_RenSTL);
    m_RenWin->AddRenderer(m_Ren);
    
    m_RenSTL->SetLayer(0);
    m_Ren->SetLayer(1);


    m_Iren = vtkRenderWindowInteractor::New();
    m_Iren->SetRenderWindow(m_RenWin);

    m_Ren->SetBackground(.3, .6, .3);
    m_RenSTL->SetBackground(.3, .6, .3);

    m_RenWin->SetSize(600, 600);
    m_RenWin->SetWindowName(m_WindowTitle.c_str());
}

Graphic::~Graphic() {
    m_Ren->Delete();
    m_RenSTL->Delete();

    m_RenWin->Delete();
    m_Iren->Delete();
}

void Graphic::Run() {
    CreateAxes();
    CreateLinks();
    RenderBase();
    CreateSTLs();

    // Initialize must be called prior to creating timer events.
    m_Iren->Initialize();

    // Sign up to receive TimerEvent
    vtkSmartPointer<vtkTimerCallback> cb = 
        vtkSmartPointer<vtkTimerCallback>::New();

    cb->m_AxesActors = &m_AxesActors;
    cb->m_LineSources = &m_LineSources;
    cb->m_STLActors = &m_STLActors;
    cb->m_Robot = m_Robot;

    m_Iren->AddObserver(vtkCommand::TimerEvent, cb);

    int timer_id = m_Iren->CreateRepeatingTimer(100);

    // Mouse manipulation style
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
        vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    m_Iren->SetInteractorStyle(style);

    // Camera
    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(0, -m_CamDist, 0);
    camera->SetFocalPoint(0, 0, 0);
    camera->SetViewUp(0, 0, 1);
    m_Ren->SetActiveCamera(camera);
    m_RenSTL->SetActiveCamera(camera);

    m_Iren->Start();
}

void Graphic::SetGraphicScaling(double K) {
    m_K = K;
}

void Graphic::SetSTLVisibility(int flag) {
    m_STLVisibility = flag;
}

void Graphic::SetOpacity(double opacity) {
    m_Opacity = opacity;
}



void Graphic::SetCameraDistance(double dist) {
    m_CamDist = dist;
}

void Graphic::CreateAxes() {
    ostringstream o;

    for (size_t i = 0; i < m_Robot->GetLinks().size(); i++) {
        vtkSmartPointer<vtkAxesActor> axes = 
            vtkSmartPointer<vtkAxesActor>::New();
        m_AxesActors.push_back(axes);
        //axes->AxisLabelsOff();        
        axes->SetTotalLength(m_K, m_K, m_K);

        vtkSmartPointer<vtkTextProperty> tprop = 
            vtkSmartPointer<vtkTextProperty>::New();
        tprop->SetFontFamilyToTimes();
        axes->GetXAxisCaptionActor2D()->SetCaptionTextProperty(tprop);
        axes->GetYAxisCaptionActor2D()->SetCaptionTextProperty(tprop);
        axes->GetZAxisCaptionActor2D()->SetCaptionTextProperty(tprop);

        axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);
        axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);
        axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);

        o.str("");
        o << "x" << i;
        axes->SetXAxisLabelText(o.str().c_str());

        o.str("");  
        o << "y" << i;
        axes->SetYAxisLabelText(o.str().c_str());

        o.str("");
        o << "z" << i;
        axes->SetZAxisLabelText(o.str().c_str());

        m_Ren->AddActor(axes);
    }
}

void Graphic::CreateLinks() {
    for (size_t i = 0; i < m_Robot->GetLinks().size(); i++) {
        vtkSmartPointer<vtkLineSource> ls = 
            vtkSmartPointer<vtkLineSource>::New();
        m_LineSources.push_back(ls);

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper = 
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(ls->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);
        actor->GetProperty()->SetColor(1.0, 0.0, 1.0);

        m_Ren->AddActor(actor);
    }
}

void Graphic::CreateSTLs() {
    for (size_t i = 0; i < m_Robot->GetLinks().size(); i++) {
        Link *l = m_Robot->GetLinks().at(i);
        const char *fn = l->GetSTLFileName();

        if (fn) {
            vtkSmartPointer<vtkSTLReader> reader = 
                vtkSmartPointer<vtkSTLReader>::New();
            reader->SetFileName(fn);
            reader->Update();

            // Visualize
            vtkSmartPointer<vtkPolyDataMapper> mapper = 
                vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(reader->GetOutputPort());

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->SetVisibility(m_STLVisibility);
            actor->GetProperty()->SetOpacity(m_Opacity);

            char c = l->GetColor();
            actor->GetProperty()->SetColor(Rgb(c));

            m_RenSTL->AddActor(actor);

            m_STLActors.push_back(actor);
        }
        else
            m_STLActors.push_back(NULL);
    }
}

void Graphic::RenderBase()
{
    const char *fn = m_Robot->GetBaseSTLFileName();
    if (fn) {
        vtkSmartPointer<vtkSTLReader> reader = 
            vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(fn);
        reader->Update();

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper = 
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(reader->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = 
            vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->SetVisibility(m_STLVisibility);
        actor->GetProperty()->SetOpacity(m_Opacity);

        double p[3];
        m_Robot->GetBasePosition(p);
        actor->SetPosition(p);
        actor->GetProperty()->SetColor(Rgb('w')); // White color for the base

        m_RenSTL->AddActor(actor);

        m_STLActors.push_back(actor);
    }
    else
        m_STLActors.push_back(NULL);
}

double * Graphic::Rgb(char c)
{
    double color[3];
    if (c == 'r') {
        color[0] = 1;
        color[1] = 0;
        color[2] = 0;
    }
    else if (c == 'g') {
        color[0] = 0;
        color[1] = 1;
        color[2] = 0;
    }
    else if (c == 'b') {
        color[0] = 0;
        color[1] = 0;
        color[2] = 1;
    }
    else if (c == 'y') {
        color[0] = 1;
        color[1] = 1;
        color[2] = 0;
    }
    else if (c == 'k') {
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
    }
    else if (c == 'c') {
        color[0] = 0;
        color[1] = 1;
        color[2] = 1;
    }
    else if (c == 'm') {
        color[0] = 1;
        color[1] = 0;
        color[2] = 1;
    }
    else if (c == 'w') {
        color[0] = 1;
        color[1] = 1;
        color[2] = 1;
    }
    else {
        color[0] = 1;
        color[1] = 1;
        color[2] = 1;
    }

    return color;

}
