/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef VIEWER_H
#define VIEWER_H

#include <Eigen/Dense>

#include <iCubGaze.h>
#include <RankableCandidate.h>
#include <thrift/GraspData.h>

#include <vector>
#include <cmath>
#include <limits>
#include <unordered_map>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkPolyDataMapper.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkProperty.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkOBJReader.h>
#include <vtkTexture.h>
#include <vtkPNGReader.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>


namespace viewer {

class Viewer;

/******************************************************************************/
class vtkUpdateHandler : public vtkCommand {
    Viewer* viewer;
    bool shutdown = false;

public:
    /**************************************************************************/
    static vtkUpdateHandler* New();

    /**************************************************************************/
    void SetViewer(Viewer* viewer);

    /**************************************************************************/
    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), void * vtkNotUsed(callData));

    /**************************************************************************/
    void ShutDown();
};

/******************************************************************************/
class VtkMeshOBJ {
    vtkSmartPointer<vtkPolyData>       vtk_polydata;
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkOBJReader>      vtk_reader;
    vtkSmartPointer<vtkTexture>        vtk_texture;
    vtkSmartPointer<vtkPNGReader>      vtk_texture_reader;
    vtkSmartPointer<vtkActor>          vtk_actor;

public:
    /**************************************************************************/
    VtkMeshOBJ(const std::string& name, const std::string& path) {
        const std::string root = path + "/" + name + "/";
        const std::string mesh_path = root + "textured.obj";
        const std::string texture_path = root + "texture_map.png";

        vtk_reader = vtkSmartPointer<vtkOBJReader>::New();
        vtk_reader->SetFileName(mesh_path.c_str());

        vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_reader->GetOutputPort());

        vtk_actor = vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);

        vtk_texture_reader = vtkSmartPointer<vtkPNGReader>::New();

        vtk_texture = vtkSmartPointer<vtkTexture>::New();
        vtk_texture_reader->SetFileName(texture_path.c_str());
        vtk_texture_reader->Update();
        vtk_texture->SetInputConnection(vtk_texture_reader->GetOutputPort());

        vtk_actor->SetTexture(vtk_texture);
        vtk_actor->GetProperty()->SetOpacity(1.0);
    }

    /**************************************************************************/
    void addToRenderer(vtkRenderer& renderer) {
        renderer.AddActor(vtk_actor);
    }

    /**************************************************************************/
    void hide() {
        vtk_actor->SetVisibility(false);
    }

    /**************************************************************************/
    void show() {
        vtk_actor->SetVisibility(true);
    }

    /**************************************************************************/
    void setPose(const Eigen::Transform<double, 3, Eigen::Affine>& pose) {

        Eigen::AngleAxisd angle_axis(pose.rotation());

        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        transform->Translate(pose.translation().data());
        transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                              angle_axis.axis()(0), angle_axis.axis()(1), angle_axis.axis()(2));

        vtk_actor->SetUserTransform(transform);
    }
};

/******************************************************************************/
class Viewer {
    vtkSmartPointer<vtkRenderer>                     vtk_renderer{nullptr};
    vtkSmartPointer<vtkRenderWindow>                 vtk_renderWindow{nullptr};
    vtkSmartPointer<vtkRenderWindowInteractor>       vtk_renderWindowInteractor{nullptr};
    vtkSmartPointer<vtkUpdateHandler>                vtk_updateCallback{nullptr};
    vtkSmartPointer<vtkAxesActor>                    vtk_axes{nullptr};
    vtkSmartPointer<vtkInteractorStyleSwitch>        vtk_style{nullptr};
    vtkSmartPointer<vtkCamera>                       vtk_camera{nullptr};
    std::vector<vtkSmartPointer<vtkArrowSource>>     vtk_arrows;
    std::vector<vtkSmartPointer<vtkPolyDataMapper>>  vtk_arrows_mappers;
    std::vector<vtkSmartPointer<vtkTransform>>       vtk_arrows_transforms;
    std::vector<vtkSmartPointer<vtkActor>>           vtk_arrows_actors;
    std::unordered_map<std::string, std::unique_ptr<VtkMeshOBJ>>  vtk_meshes;

    std::unordered_map<std::string, Eigen::VectorXd> object_properties;
    Eigen::VectorXd                                  object_sizes;
    Eigen::Transform<double, 3, Eigen::Affine>       object_pose;

    yarp::os::BufferedPort<GraspData>                port_grasp_data;
    std::unique_ptr<iCubGaze>                        gaze;

public:
    /**************************************************************************/
    Viewer() = delete;

    /**************************************************************************/
    Viewer(const int x, const int y, const int w, const int h) {
        vtk_renderer = vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetPosition(x, y);
        vtk_renderWindow->SetSize(w, h);
        vtk_renderWindow->SetWindowName("Grasp candidates viewer");
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);
        vtk_renderer->SetBackground(std::vector<double>({.7, .7, .7}).data());

        vtk_axes = vtkSmartPointer<vtkAxesActor>::New();
        vtk_axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->SetTotalLength(std::vector<double>({.1, .1, .1}).data());
        vtk_renderer->AddActor(vtk_axes);

        vtk_style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

        if (!port_grasp_data.open("/roft-samples-grasp-viewer/grasp-data:i"))
            throw(std::runtime_error("Viewer::ctor. Error: cannot open port for grasp information input."));

        gaze = std::unique_ptr<iCubGaze>(new iCubGaze("roft-samples-grasp-viewer"));
    }

    /**************************************************************************/
    void start(const double period) {
        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(period);
        vtk_updateCallback = vtkSmartPointer<vtkUpdateHandler>::New();
        vtk_updateCallback->SetViewer(this);
        vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, vtk_updateCallback);
        vtk_renderWindowInteractor->Start();
    }

    /**************************************************************************/
    void stop() {
        vtk_updateCallback->ShutDown();
        port_grasp_data.close();
        gaze->close();
    }

    /**************************************************************************/
    void loadObjects(const std::unordered_map<std::string, Eigen::VectorXd>& properties, const std::string& meshes_path) {

        object_properties = properties;

        for (const auto& pair : object_properties)
        {
            const std::string& name = pair.first;

            vtk_meshes[name] = std::unique_ptr<VtkMeshOBJ>(new VtkMeshOBJ(name, meshes_path));
            vtk_meshes[name]->addToRenderer(*vtk_renderer);
            vtk_meshes[name]->hide();
        }
    }

    /**************************************************************************/
    void update() {
        bool blocking = false;
        GraspData* grasp_data = port_grasp_data.read(blocking);

        if (grasp_data != nullptr)
        {
            /* Extract camera information. */
            yarp::sig::Vector cam_x;
            yarp::sig::Vector cam_o;
            gaze->controller().getLeftEyePose(cam_x, cam_o);

            yarp::os::Bottle info;
            gaze->controller().getInfo(info);
            const auto w = info.find("camera_width_left").asInt32();
            const auto fov_h = info.find("camera_intrinsics_left").asList()->get(0).asFloat64();
            const auto view_angle = 2. * std::atan((w / 2.) / fov_h) * (180. / M_PI);

            /* Extract grasp data. */
            Eigen::Transform<double, 3, Eigen::Affine> object_pose;
            object_pose.matrix() = yarp::eigen::toEigen(grasp_data->object_pose);

            std::vector<rankable_candidate> candidates;
            for (const auto& candidate : grasp_data->candidates)
                candidates.push_back(std::make_tuple(candidate.hand, candidate.cost, candidate.candidate, candidate.center));

            /* Update viewer. */
            updateCamera
            (
                {cam_x[0], cam_x[1], cam_x[2]},
                {object_pose.translation()(0), object_pose.translation()(1), object_pose.translation()(2)},
                {0., 0., 1.},
                view_angle
            );
            updateObject(grasp_data->object_name, object_pose);
            focusOnObject();
            showCandidates(candidates);
        }
    }

private:
    /**************************************************************************/
    void updateCamera(const std::vector<double>& position, const std::vector<double>& focalpoint,
                      const std::vector<double>& viewup, const double view_angle) {
        vtk_camera = vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(position.data());
        vtk_camera->SetFocalPoint(focalpoint.data());
        vtk_camera->SetViewUp(viewup.data());
        vtk_camera->SetViewAngle(view_angle);
        vtk_renderer->SetActiveCamera(vtk_camera);
    }

    // /**************************************************************************/
    void updateObject(const std::string& name, const Eigen::Transform<double, 3, Eigen::Affine>& pose) {

        object_sizes.resize(3);
        object_sizes(0) = object_properties.at(name)(0) / 2;
        object_sizes(1) = object_properties.at(name)(1) / 2;
        object_sizes(2) = object_properties.at(name)(2) / 2;

        object_pose = pose;

        for (const auto& pair : vtk_meshes)
            pair.second->hide();

        vtk_meshes[name]->setPose(pose);
        vtk_meshes[name]->show();
    }

    /**************************************************************************/
    void focusOnObject() {
        std::vector<double> centroid(3);
        centroid[0] = object_pose.translation()(0);
        centroid[1] = object_pose.translation()(1);
        centroid[2] = object_pose.translation()(2);
        vtk_camera->SetPosition(0., 0., centroid[2] + .15);
        vtk_camera->SetFocalPoint(centroid.data());
    }

    /**************************************************************************/
    bool showCandidates(const std::vector<rankable_candidate>& candidates) {
        if (!vtk_arrows_actors.empty()) {
            for (auto vtk_actor:vtk_arrows_actors) {
                vtk_renderer->RemoveActor(vtk_actor);
            }
            vtk_arrows.clear();
            vtk_arrows_mappers.clear();
            vtk_arrows_transforms.clear();
            vtk_arrows_actors.clear();
        }

        const auto x = object_pose.translation()(0);
        const auto y = object_pose.translation()(1);
        const auto z = object_pose.translation()(2);
        const auto bx = object_sizes(0);
        const auto by = object_sizes(1);
        const auto bz = object_sizes(2);

        const yarp::sig::Vector sqCenter{x, y, z};
        std::vector<yarp::sig::Vector> sqPoints{{bx, 0., 0., 1.}, {0., -by, 0., 1.},
                                                {-bx, 0., 0., 1.}, {0., by, 0., 1.},
                                                {0., 0., bz, 1.}};
        yarp::sig::Matrix rotation;
        rotation.resize(3, 3);
        yarp::eigen::toEigen(rotation) = object_pose.rotation();
        for (auto& sq_p_:sqPoints) {
            sq_p_ = sqCenter + (rotation * sq_p_).subVector(0, 2);
        }

        for (const auto& c:candidates) {
            const auto& type = std::get<0>(c);
            const auto& err = std::get<1>(c);
            auto T = std::get<2>(c);
            const auto L = .1 * (1. - err); // arrows' max length

            const auto p = T.getCol(3).subVector(0, 2);
            yarp::sig::Vector sq_p;
            auto max_d{std::numeric_limits<double>::infinity()};
            for (const auto& sq_p_:sqPoints) {
                const auto d = yarp::math::norm(p - sq_p_);
                if (d < max_d) {
                    max_d = d;
                    sq_p = sq_p_;
                }
            }
            const auto axis_x = (sqCenter - sq_p) / yarp::math::norm(sqCenter - sq_p);
            // take a generic vector normal to axis_x
            yarp::sig::Vector axis_y;
            if (std::abs(axis_x[0]) > .001) {
                axis_y = yarp::sig::Vector{-axis_x[1] / axis_x[0], 1., 0.};
            } else if (std::abs(axis_x[1]) > .001) {
                axis_y = yarp::sig::Vector{1., -axis_x[0] / axis_x[1], 0.};
            } else {
                axis_y = yarp::sig::Vector{0., 1., -axis_x[1] / axis_x[2]};
            }
            axis_y /= yarp::math::norm(axis_y);
            const auto axis_z = yarp::math::cross(axis_x, axis_y);
            T.setSubcol(axis_x, 0, 0);
            T.setSubcol(axis_y, 0, 1);
            T.setSubcol(axis_z, 0, 2);
            T.setSubcol(sq_p, 0, 3);

            vtkSmartPointer<vtkArrowSource> vtk_arrow = vtkSmartPointer<vtkArrowSource>::New();
            vtk_arrow->SetTipResolution(10);
            vtk_arrow->SetShaftResolution(10);

            vtkSmartPointer<vtkPolyDataMapper> vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtk_mapper->SetInputConnection(vtk_arrow->GetOutputPort());

            vtkSmartPointer<vtkActor> vtk_actor = vtkSmartPointer<vtkActor>::New();
            vtk_actor->SetMapper(vtk_mapper);
            if (type == "right") {
                vtk_actor->GetProperty()->SetColor(0., 0., 1.);
            } else {
                vtk_actor->GetProperty()->SetColor(1., 0., 0.);
            }
            vtk_actor->GetProperty()->SetOpacity(c == candidates.front() ? 1. : .25);

            vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();
            vtk_transform->Translate(T.getCol(3).subVector(0, 2).data());
            const auto axisangle = yarp::math::dcm2axis(T);
            vtk_transform->RotateWXYZ((180. / M_PI) * axisangle[3], axisangle.subVector(0, 2).data());
            vtk_transform->Translate(-L, 0., 0.);
            vtk_transform->Scale(L, L, L);
            vtk_actor->SetUserTransform(vtk_transform);

            vtk_arrows.push_back(vtk_arrow);
            vtk_arrows_mappers.push_back(vtk_mapper);
            vtk_arrows_actors.push_back(vtk_actor);
            vtk_arrows_transforms.push_back(vtk_transform);

            vtk_renderer->AddActor(vtk_actor);
        }

        return true;
    }
};

}

#endif
