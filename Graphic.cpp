//
// Created by manurunga on 11/8/15.
//

#include "Graphic.h"

/*--------------------------------------------------------------------------------------------------------------------*/

vtkTimerCallback *vtkTimerCallback::New() {
    vtkTimerCallback *cb = new vtkTimerCallback;
    return cb;
}

void vtkTimerCallback::Execute(vtkObject *caller, unsigned long eventId,
                               void *vtkNotUsed(callData)) {
    vector<Link *> links = m_Robot->GetLinks();
    vtkSmartPointer<vtkMatrix4x4> vtk_A = vtkSmartPointer<vtkMatrix4x4>::New();
    mat A;
    mat A_prev;
    Link *l;
    Link *l_prev;
    double p0[3] = {0, 0, 0};
    double p1[3] = {0, 0, 0};

    for (int i = 0; i < links.size(); i++) {
        l = links.at(i);
        l->GetTransformationMatrix(A);

        if (i > 0) {
            l_prev = links.at(i - 1);
            l_prev->GetTransformationMatrix(A_prev);
        }

        // Axes
        // Armadillo matrix to VTK matrix
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                vtk_A->SetElement(r, c, A.at(r, c));

        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        transform->PostMultiply();
        transform->SetMatrix(vtk_A);

        m_AxesActors->at(i)->SetUserTransform(transform);

        // Lines
        if (i > 0)
            for (int r = 0; r < 3; r++)
                p0[r] = A_prev.at(r, 3);

        for (int r = 0; r < 3; r++)
            p1[r] = A.at(r, 3);

        vtkLineSource *ls = m_LineSources->at(i);
        ls->SetPoint1(p0);
        ls->SetPoint2(p1);
        ls->Update();
    }

    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
    iren->GetRenderWindow()->Render();
}

/*--------------------------------------------------------------------------------------------------------------------*/

Graphic::Graphic(Robot *robot) {
    m_K = 0.05;
    m_Robot = robot;

    m_Ren = vtkRenderer::New();
    m_RenWin = vtkRenderWindow::New();
    m_RenWin->AddRenderer(m_Ren);
    m_Iren = vtkRenderWindowInteractor::New();
    m_Iren->SetRenderWindow(m_RenWin);

    m_Ren->SetBackground(0, 0, 0);
    m_RenWin->SetSize(600, 600);

    CreateAxes();
    CreateLinks();

    // Initialize must be called prior to creating timer events.
    m_Iren->Initialize();

    // Sign up to receive TimerEvent
    vtkSmartPointer<vtkTimerCallback> cb = vtkSmartPointer<vtkTimerCallback>::New();

    cb->m_AxesActors = &m_AxesActors;
    cb->m_LineSources = &m_LineSources;
    cb->m_Robot = m_Robot;

    m_Iren->AddObserver(vtkCommand::TimerEvent, cb);

    int timer_id = m_Iren->CreateRepeatingTimer(100);

    // Mouse manipulation style
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    m_Iren->SetInteractorStyle(style);

    // Camera
    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(0, -2, 0);
    camera->SetFocalPoint(0, 0, 0);
    camera->SetViewUp(0, 0, 1);
    m_Ren->SetActiveCamera(camera);

    m_RenWin->Render();
}

Graphic::~Graphic() {
    m_Ren->Delete();
    m_RenWin->Delete();
    m_Iren->Delete();
}

void Graphic::Run() {
    m_Iren->Start();
}

void Graphic::SetGraphicScaling(double K) {
    m_K = K;
}

void Graphic::CreateAxes() {
    for (int i = 0; i < m_Robot->GetLinks().size(); i++) {
        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        m_AxesActors.push_back(axes);
        //axes->AxisLabelsOff();        
        axes->SetTotalLength(m_K, m_K, m_K);

        vtkSmartPointer<vtkTextProperty> tprop = vtkSmartPointer<vtkTextProperty>::New();
        tprop->SetFontFamilyToTimes();
        axes->GetXAxisCaptionActor2D()->SetCaptionTextProperty(tprop);
        axes->GetYAxisCaptionActor2D()->SetCaptionTextProperty(tprop);
        axes->GetZAxisCaptionActor2D()->SetCaptionTextProperty(tprop);

        axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);
        axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);
        axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);

        char c[8];
        sprintf(c, "x%i", i + 1);
        axes->SetXAxisLabelText(c);
        sprintf(c, "y%i", i + 1);
        axes->SetYAxisLabelText(c);
        sprintf(c, "z%i", i + 1);
        axes->SetZAxisLabelText(c);

        m_Ren->AddActor(axes);
    }
}

void Graphic::CreateLinks() {
    for (int i = 0; i < m_Robot->GetLinks().size(); i++) {
        vtkSmartPointer<vtkLineSource> ls = vtkSmartPointer<vtkLineSource>::New();
        m_LineSources.push_back(ls);

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(ls->GetOutputPort());
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);
        actor->GetProperty()->SetColor(1.0, 0.0, 1.0);

        m_Ren->AddActor(actor);
    }
}