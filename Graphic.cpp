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
    
    for (size_t h = 0;  h < m_RobotList->size(); h++) {
        Robot *robot = m_RobotList->at(h);
        vector<vtkSmartPointer<vtkLineSource> > line_sources = m_LineSourcesList->at(h);
        vector<vtkSmartPointer<vtkAxesActor> > axes_actors = m_AxesActorsList->at(h); 
        vector<vtkSmartPointer<vtkActor> > stl_actors = m_STLActorsList->at(h);
        vector<vtkSmartPointer<vtkActor> > edge_actors = m_EdgeActorsList->at(h);        
            
        vector<Link *> links = robot->GetLinks();
        vtkSmartPointer<vtkMatrix4x4> vtk_A_tip = 
            vtkSmartPointer<vtkMatrix4x4>::New();
        vtkSmartPointer<vtkMatrix4x4> vtk_A_joint = 
            vtkSmartPointer<vtkMatrix4x4>::New();
        mat A_tip;
        mat A_joint;

        double p0[3] = { 0, 0, 0 };
        double p1[3] = { 0, 0, 0 };

        for (size_t i = 0; i < links.size(); i++) {

            robot->GetJointTransformation(i, A_joint);
            robot->GetTipTransformation(i, A_tip);

            // Axes
            // Armadillo matrix to VTK matrix
            ArmaMatToVTKMat(vtk_A_tip, A_tip);
            ArmaMatToVTKMat(vtk_A_joint, A_joint);

            vtkSmartPointer<vtkTransform> transform_tip = 
                vtkSmartPointer<vtkTransform>::New();
            transform_tip->PostMultiply();
            transform_tip->SetMatrix(vtk_A_tip);

            axes_actors.at(i)->SetUserTransform(transform_tip);

            // Lines
            robot->GetJointPosition(i, p0);
            robot->GetTipPosition(i, p1);

            vtkLineSource *ls = line_sources.at(i);
            ls->SetPoint1(p0);
            ls->SetPoint2(p1);
            ls->Update();

            // STL
            if (links.at(i)->GetSTLFileName())  {
                vtkSmartPointer<vtkTransform> transform_joint = 
                    vtkSmartPointer<vtkTransform>::New();
                transform_joint->PostMultiply();
                transform_joint->SetMatrix(vtk_A_tip);

                // Index zero is for the base, so we use i+1
                stl_actors.at(i)->SetUserTransform(transform_joint);
                edge_actors.at(i)->SetUserTransform(transform_joint);                              
            }
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

Graphic::Graphic(const char *window_title) {
    m_CamDist = 3;
    m_K = 0.05;
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

void Graphic::AddRobot(Robot *robot)
{
    m_RobotList.push_back(robot);
}

void Graphic::Run() {
    CreateAxes();
    CreateLinks();
    CreateSTLs();
    RenderBase();
    RenderGridFloor(5, 0.5);
    RenderGlobalAxis();

    // Initialize must be called prior to creating timer events.
    m_Iren->Initialize();

    // Sign up to receive TimerEvent
    vtkSmartPointer<vtkTimerCallback> cb = 
        vtkSmartPointer<vtkTimerCallback>::New();

    cb->m_AxesActorsList = &m_AxesActorsList;
    cb->m_EdgeActorsList = &m_EdgeActorsList;
    cb->m_LineSourcesList = &m_LineSourcesList;
    cb->m_STLActorsList = &m_STLActorsList;
    cb->m_RobotList = &m_RobotList;

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

    for (size_t h = 0;  h < m_RobotList.size(); h++) {
        Robot *robot = m_RobotList.at(h);
        
        vector<vtkSmartPointer<vtkAxesActor> > axes_actors;
        
        for (size_t i = 0; i < robot->GetLinks().size(); i++) {
            vtkSmartPointer<vtkAxesActor> axes = 
                vtkSmartPointer<vtkAxesActor>::New();
            axes_actors.push_back(axes);
            
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
        
        m_AxesActorsList.push_back(axes_actors);
    }
}

void Graphic::CreateLinks() {
    for (size_t h = 0;  h < m_RobotList.size(); h++) {
        Robot *robot = m_RobotList.at(h);
        
        vector<vtkSmartPointer<vtkLineSource> > line_sources;
        
        for (size_t i = 0; i < robot->GetLinks().size(); i++) {
            vtkSmartPointer<vtkLineSource> ls = 
                vtkSmartPointer<vtkLineSource>::New();
            line_sources.push_back(ls);

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
        
        m_LineSourcesList.push_back(line_sources);
    }
}

void Graphic::CreateSTLs() {
    for (size_t h = 0;  h < m_RobotList.size(); h++) {
        Robot *robot = m_RobotList.at(h);
        
        vector<vtkSmartPointer<vtkActor> > stl_actors;
        vector<vtkSmartPointer<vtkActor> > edge_actors;
        
        for (size_t i = 0; i < robot->GetLinks().size(); i++) {
            Link *l = robot->GetLinks().at(i);
            const char *fn = l->GetSTLFileName();

            if (fn) {
                vtkSmartPointer<vtkSTLReader> reader = 
                    vtkSmartPointer<vtkSTLReader>::New();
                reader->SetFileName(fn);
                reader->Update();
                
                // Edges
                vtkSmartPointer<vtkFeatureEdges> feature_edges =
                    vtkSmartPointer<vtkFeatureEdges>::New();
                feature_edges->SetInputConnection(reader->GetOutputPort());
                feature_edges->Update();
                
                // Visualize the edges
                vtkSmartPointer<vtkPolyDataMapper> edge_mapper = 
                    vtkSmartPointer<vtkPolyDataMapper>::New();
                edge_mapper->SetInputConnection(feature_edges->GetOutputPort());
                vtkSmartPointer<vtkActor> edge_actor =  vtkSmartPointer<vtkActor>::New();

                edge_mapper->SetInputConnection(feature_edges->GetOutputPort());
                edge_mapper->ScalarVisibilityOff();  

                edge_actor->SetMapper(edge_mapper);
                edge_actor->GetProperty()->SetLineWidth(4);
                edge_actor->GetProperty()->SetColor(0.0, 0.0, 0.0); // <- lines use SetColor
                //edge_actor->GetProperty()->EdgeVisibilityOn();

                // Visualize the STLs
                vtkSmartPointer<vtkPolyDataMapper> stl_mapper = 
                    vtkSmartPointer<vtkPolyDataMapper>::New();
                stl_mapper->SetInputConnection(reader->GetOutputPort());

                vtkSmartPointer<vtkActor> stl_actor = vtkSmartPointer<vtkActor>::New();
                stl_actor->SetMapper(stl_mapper);
                stl_actor->SetVisibility(m_STLVisibility);
                stl_actor->GetProperty()->SetOpacity(m_Opacity);

                char c = l->GetColor();
                double color[3];
                Rgb(c, color);
                stl_actor->GetProperty()->SetColor(color);

                //
                m_RenSTL->AddActor(stl_actor);
                m_RenSTL->AddActor(edge_actor);

                stl_actors.push_back(stl_actor);
                edge_actors.push_back(edge_actor);
            }
            else {
                stl_actors.push_back(NULL);
                edge_actors.push_back(NULL);
            }
        }
        
        m_STLActorsList.push_back(stl_actors);
        m_EdgeActorsList.push_back(edge_actors);
    }
}

void Graphic::RenderBase()
{
    for (size_t h = 0;  h < m_RobotList.size(); h++) {
        Robot *robot = m_RobotList.at(h);
        
        const char *fn = robot->GetBaseSTLFileName();
        if (fn) {
            vtkSmartPointer<vtkSTLReader> reader = 
                vtkSmartPointer<vtkSTLReader>::New();
            reader->SetFileName(fn);
            reader->Update();
            
            // Edges
            vtkSmartPointer<vtkFeatureEdges> feature_edges =
                vtkSmartPointer<vtkFeatureEdges>::New();
            feature_edges->SetInputConnection(reader->GetOutputPort());
            feature_edges->Update();
            
            // Visualize the edges
            vtkSmartPointer<vtkPolyDataMapper> edge_mapper = 
                vtkSmartPointer<vtkPolyDataMapper>::New();

            vtkSmartPointer<vtkActor> edge_actor =
                vtkSmartPointer<vtkActor>::New();

            edge_mapper->SetInputConnection(feature_edges->GetOutputPort());
            edge_mapper->ScalarVisibilityOff();     

            edge_actor->SetMapper(edge_mapper);
            edge_actor->GetProperty()->SetLineWidth(4);
            edge_actor->GetProperty()->SetColor(0.0, 0.0, 0.0); // <- lines use SetColor

            // Visualize th STLs
            vtkSmartPointer<vtkPolyDataMapper> stl_mapper = 
                vtkSmartPointer<vtkPolyDataMapper>::New();
            stl_mapper->SetInputConnection(reader->GetOutputPort());

            vtkSmartPointer<vtkActor> stl_actor = 
                vtkSmartPointer<vtkActor>::New();
            stl_actor->SetMapper(stl_mapper);
            stl_actor->SetVisibility(m_STLVisibility);
            stl_actor->GetProperty()->SetOpacity(m_Opacity);

            double p[3];
            robot->GetBasePosition(p);
            stl_actor->SetPosition(p);
            edge_actor->SetPosition(p);
            
            double color[3];
            Rgb('w', color);
            stl_actor->GetProperty()->SetColor(color); // White color for the base

            //
            m_RenSTL->AddActor(stl_actor);
            m_RenSTL->AddActor(edge_actor);

            m_STLActorsList.at(h).push_back(stl_actor);
            m_EdgeActorsList.at(h).push_back(edge_actor);
            
        }
        else {
            m_STLActorsList.at(h).push_back(NULL);
            m_EdgeActorsList.at(h).push_back(NULL);
        }
    }
}

void Graphic::RenderGridFloor(double grid_dimension, double step)
{
    const double HALF = grid_dimension/2.0;
    
    for (double i = 0.0; i <= grid_dimension; i = i + step) {
        vtkSmartPointer<vtkLineSource> ls = 
            vtkSmartPointer<vtkLineSource>::New();
            
        ls->SetPoint1(i - HALF, -HALF, 0);
        ls->SetPoint2(i - HALF, HALF, 0);
        ls->Update();

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper = 
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(ls->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);
        actor->GetProperty()->SetColor(1.0, 1.0, 1.0); // White

        m_RenSTL->AddActor(actor);
    }
    
     for (double i = 0; i <= grid_dimension; i = i + step) {
        vtkSmartPointer<vtkLineSource> ls = 
            vtkSmartPointer<vtkLineSource>::New();
            
        ls->SetPoint1(-HALF, i - HALF, 0);
        ls->SetPoint2(HALF, i - HALF, 0);
        ls->Update();

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper = 
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(ls->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);
        actor->GetProperty()->SetColor(1.0, 1.0, 1.0); // White

        m_RenSTL->AddActor(actor);
     }
    
}

void Graphic::RenderGlobalAxis()
{   
    vtkSmartPointer<vtkAxesActor> axis =
        vtkSmartPointer<vtkAxesActor>::New();
        
    vtkSmartPointer<vtkTextProperty> tprop = 
        vtkSmartPointer<vtkTextProperty>::New();
    tprop->SetFontFamilyToTimes();

    axis->GetXAxisCaptionActor2D()->SetCaptionTextProperty(tprop);
    axis->GetYAxisCaptionActor2D()->SetCaptionTextProperty(tprop);
    axis->GetZAxisCaptionActor2D()->SetCaptionTextProperty(tprop);

    axis->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);
    axis->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);
    axis->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleMode(m_K);

    axis->SetXAxisLabelText("X");
    axis->SetYAxisLabelText("Y");
    axis->SetZAxisLabelText("Z");

    axis->SetTotalLength(m_K, m_K, m_K);

    axis->GetXAxisShaftProperty()->SetLineWidth(2);
    axis->GetYAxisShaftProperty()->SetLineWidth(2);
    axis->GetZAxisShaftProperty()->SetLineWidth(2);

    axis->SetPosition(0, 0, 0);

    m_Ren->AddActor(axis);
}

void Graphic::Rgb(char c, double color[3])
{
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
}
