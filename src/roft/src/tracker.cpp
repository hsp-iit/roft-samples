/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <tracker.h>

#include <iostream>
#include <memory>

#include <yarp/sig/Image.h>

#include <ROFT/CameraMeasurement.h>
#include <ROFT/ImageOpticalFlowSource.h>
#include <ROFT/ImageOpticalFlowNVOF.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/RealsenseCameraYarp.h>
#include <RobotsIO/Utils/Segmentation.h>
#include <RobotsIO/Utils/SegmentationYarpPort.h>
#include <RobotsIO/Utils/Transform.h>
#include <RobotsIO/Utils/TransformYarpPort.h>
#include <RobotsIO/Utils/YarpImageOfProbe.hpp>
#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>

using namespace Eigen;
using namespace ROFT;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace yarp::os;
using namespace yarp::sig;


Tracker::Tracker(const ResourceFinder& rf)
{
    /* Sample time. */

    const double sample_time = rf.check("sample_time", Value(1.0 / 30.0)).asFloat64();

    /* Camera. */

    const Bottle rf_camera = rf.findGroup("CAMERA");
    const std::string camera_source = rf_camera.check("source", Value("")).asString();
    const std::size_t camera_width = rf_camera.check("width", Value(1280)).asInt32();
    const std::size_t camera_height = rf_camera.check("height", Value(720)).asInt32();
    const double camera_fx = rf_camera.check("fx", Value(1230.0)).asFloat64();
    const double camera_fy = rf_camera.check("fy", Value(1230.0)).asFloat64();
    const double camera_cx = rf_camera.check("cx", Value(1280 / 2.0)).asFloat64();
    const double camera_cy = rf_camera.check("cy", Value(720 / 2.0)).asFloat64();

    /* Initial condition. */

    const Bottle rf_initial_conditions = rf.findGroup("INITIAL_CONDITION");

    const VectorXd p_v_0 = load_vector_double(rf_initial_conditions, "p_v_0", 3);
    const VectorXd p_w_0 = load_vector_double(rf_initial_conditions, "p_w_0", 3);

    const VectorXd p_cov_v_0 = load_vector_double(rf_initial_conditions, "p_cov_v_0", 3);
    const VectorXd p_cov_w_0 = load_vector_double(rf_initial_conditions, "p_cov_w_0", 3);
    const VectorXd p_cov_x_0 = load_vector_double(rf_initial_conditions, "p_cov_x_0", 3);
    const VectorXd p_cov_q_0 = load_vector_double(rf_initial_conditions, "p_cov_q_0", 3);

    const VectorXd v_v_0 = load_vector_double(rf_initial_conditions, "v_v_0", 3);
    const VectorXd v_w_0 = load_vector_double(rf_initial_conditions, "v_w_0", 3);
    const VectorXd v_cov_v_0 = load_vector_double(rf_initial_conditions, "v_cov_v_0", 3);
    const VectorXd v_cov_w_0 = load_vector_double(rf_initial_conditions, "v_cov_w_0", 3);

    /* Kinematic model. */

    const Bottle rf_kinematic_model = rf.findGroup("KINEMATIC_MODEL");

    const VectorXd psd_lin_acc = load_vector_double(rf_kinematic_model, "psd_lin_acc", 3);
    const VectorXd sigma_ang_vel = load_vector_double(rf_kinematic_model, "sigma_ang_vel", 3);

    const VectorXd kin_q_v = load_vector_double(rf_kinematic_model, "q_v", 3);
    const VectorXd kin_q_w = load_vector_double(rf_kinematic_model, "q_w", 3);

    /* Measurement model. */

    const Bottle rf_measurement = rf.findGroup("MEASUREMENT_MODEL");
    const VectorXd v_meas_cov_flow = load_vector_double(rf_measurement, "v_cov_flow", 2);

    const VectorXd p_meas_cov_v = load_vector_double(rf_measurement, "p_cov_v", 3);
    const VectorXd p_meas_cov_w = load_vector_double(rf_measurement, "p_cov_w", 3);
    const VectorXd p_meas_cov_x = load_vector_double(rf_measurement, "p_cov_x", 3);
    const VectorXd p_meas_cov_q = load_vector_double(rf_measurement, "p_cov_q", 3);

    const bool use_pose_measurement = rf_measurement.check("use_pose_measurement", Value(false)).asBool();
    const bool use_pose_resync = rf_measurement.check("use_pose_resync", Value(false)).asBool();
    const bool use_velocity_measurement = rf_measurement.check("use_vel_measurement", Value(false)).asBool();

    const double depth_maximum = rf_measurement.check("depth_maximum", Value(2.0)).asFloat64();
    const double subsampling_radius = rf_measurement.check("subsampling_radius", Value(1.0)).asFloat64();
    const bool flow_weighting = rf_measurement.check("flow_weighting", Value(false)).asBool();

    /* Model. */

    const Bottle rf_model = rf.findGroup("MODEL");
    const std::string model_name = rf_model.check("name", Value("")).asString();
    const bool model_use_internal_db = rf_model.check("use_internal_db", Value(true)).asBool();
    const std::string model_internal_db_name = rf_model.check("internal_db_name", Value("YCBVideo")).asString();
    const std::string model_external_path = rf_model.check("external_path", Value("")).asString();
    textured_model_external_path_root_ = rf_model.check("textured_mesh_path", Value("")).asString();
    const std::string textured_model_external_path = textured_model_external_path_root_ + "/" + model_name + "/textured.obj";

    /* Optical flow. */

    const Bottle rf_optical_flow = rf.findGroup("OPTICAL_FLOW");
    const std::string optical_flow_source = rf_optical_flow.check("source", Value("")).asString();

    /* Outlier rejection. */

    const Bottle rf_outlier = rf.findGroup("OUTLIER_REJECTION");
    const bool outlier_rejection_enable = rf_outlier.check("enable", Value(false)).asBool();
    const double outlier_rejection_gain = rf_outlier.check("gain", Value(1.0)).asFloat64();

    /* Output format. */

    const Bottle rf_output_format = rf.findGroup("OUTPUT_FORMAT");
    const std::string output_format_reference_frame = rf_output_format.check("reference_frame", Value("camera")).asString();

    /* Pose .*/

    const Bottle& pose_bottle = rf.findGroup("POSE");
    const std::string pose_source = pose_bottle.check("source", Value("YARP")).asString();

   /* Segmentation. */
    const Bottle rf_segmentation = rf.findGroup("SEGMENTATION");
    const std::string segmentation_source = rf_segmentation.check("source", Value("")).asString();
    const bool flow_aided_segmentation = rf_segmentation.check("flow_aided", Value(false)).asBool();
    const bool wait_segmentation_initialization = rf_segmentation.check("wait_initialization", Value(false)).asBool();

    /* Unscented transform. */
    const Bottle rf_unscented_transform = rf.findGroup("UNSCENTED_TRANSFORM");
    const double ut_alpha = rf_unscented_transform.check("alpha", Value("1.0")).asFloat64();
    const double ut_beta = rf_unscented_transform.check("beta", Value("2.0")).asFloat64();
    const double ut_kappa = rf_unscented_transform.check("kappa", Value("0.0")).asFloat64();

    /* Summary. */

    std::cout << log_name_ << " parameters:" << std::endl << std::endl;

    std::cout << "sample_time: " << sample_time << std::endl << std::endl;

    std::cout << "Camera:" << std::endl;

    std::cout << "- source: " << camera_source << std::endl << std::endl;
    if (camera_source == "YARP")
    {
        std::cout << "- width: " << camera_width << std::endl;
        std::cout << "- height: " << camera_height << std::endl;
        std::cout << "- fx: " << camera_fx << std::endl;
        std::cout << "- fy: " << camera_fy << std::endl;
        std::cout << "- cx: " << camera_cx << std::endl;
        std::cout << "- cy: " << camera_cy << std::endl;
    }

    std::cout << "Initial conditions:" << std::endl;

    std::cout << "  - position:" << std::endl;
    std::cout << "    - v_0: " << p_v_0.transpose() << std::endl;
    std::cout << "    - w_0: " << p_w_0.transpose() << std::endl;
    std::cout << "    - cov_v_0: " << p_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w_0: " << p_cov_w_0.transpose() << std::endl;
    std::cout << "    - cov_x_0: " << p_cov_x_0.transpose() << std::endl;
    std::cout << "    - cov_q_0: " << p_cov_q_0.transpose() << std::endl << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - v_0: " << v_v_0.transpose() << std::endl;
    std::cout << "    - w_0: " << v_w_0.transpose() << std::endl;
    std::cout << "    - cov_v_0: " << v_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w_0: " << v_cov_w_0.transpose() << std::endl << std::endl;

    std::cout << "Kinematic model:" << std::endl;

    std::cout << "  - position:" << std::endl;
    std::cout << "    - psd_lin_acc: " << psd_lin_acc.transpose() << std::endl;
    std::cout << "    - sigma_ang_vel: " << sigma_ang_vel.transpose() << std::endl << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - q_v: " << kin_q_v.transpose() << std::endl;
    std::cout << "    - q_w: " << kin_q_w.transpose() << std::endl << std::endl;

    std::cout << "Measurement model:" << std::endl;

    std::cout << "  - position: " << std::endl;
    std::cout << "    - cov_v: " << p_meas_cov_v.transpose() << std::endl;
    std::cout << "    - cov_w: " << p_meas_cov_w.transpose() << std::endl;
    std::cout << "    - cov_x: " << p_meas_cov_x.transpose() << std::endl;
    std::cout << "    - cov_q: " << p_meas_cov_q.transpose() << std::endl;
    std::cout << "    - use_pose_measurement: " << use_pose_measurement << std::endl;
    std::cout << "    - use_pose_resync: " << use_pose_resync << std::endl;
    std::cout << "    - use_velocity_measurement: " << use_velocity_measurement << std::endl << std::endl;

    std::cout << "  - velocity: " << std::endl;
    std::cout << "    - cov_flow: " << v_meas_cov_flow.transpose() << std::endl;
    std::cout << "    - depth_maximum: " << depth_maximum << std::endl;
    std::cout << "    - subsampling_radius: " << subsampling_radius << std::endl;
    std::cout << "    - flow_weighting: " << flow_weighting << std::endl << std::endl;

    std::cout << "Model:" << std::endl;

    std::cout << "- model_name: " << model_name << std::endl;
    std::cout << "- model_use_internal_db: " << model_use_internal_db << std::endl;
    std::cout << "- model_internal_db_name: " << model_internal_db_name << std::endl;
    std::cout << "- model_external_path: " << model_external_path << std::endl << std::endl;
    std::cout << "- textured_model_external_path: " << textured_model_external_path << std::endl << std::endl;

    std::cout << "Optical flow:" << std::endl;

    std::cout << "- source: " << optical_flow_source << std::endl << std::endl;

    std::cout << "Outlier rejection:" << std::endl;

    std::cout << "- enable: " << outlier_rejection_enable << std::endl;
    std::cout << "- gain: " << outlier_rejection_gain << std::endl << std::endl;

    std::cout << "Output format:" << std::endl;

    std::cout << "- reference_frame: " << output_format_reference_frame << std::endl << std::endl;

    std::cout << "Pose:" << std::endl;

    std::cout << "- source: " << pose_source << std::endl << std::endl;

    std::cout << "Segmentation:" << std::endl;

    std::cout << "- source: " << segmentation_source << std::endl;
    std::cout << "- flow_aided: " << flow_aided_segmentation << std::endl << std::endl;
    std::cout << "- wait_initialization: " << wait_segmentation_initialization << std::endl << std::endl;

    std::cout << "Unscented transform:" << std::endl;
    std::cout << "- alpha: " << ut_alpha << std::endl;
    std::cout << "- beta: "  << ut_beta << std::endl;
    std::cout << "- kappa: " << ut_kappa << std::endl << std::endl;

    /* Compose initial condition vectors. */

    VectorXd p_initial_condition(13);
    p_initial_condition.head<3>() = p_v_0;
    p_initial_condition.segment<3>(3) = p_w_0;
    p_initial_condition.segment<3>(6) = Vector3d::Zero();
    Quaterniond q_0 (AngleAxisd(0.0, Vector3d::UnitX()));
    p_initial_condition.tail<4>()(0) = q_0.w();
    p_initial_condition.tail<4>()(1) = q_0.x();
    p_initial_condition.tail<4>()(2) = q_0.y();
    p_initial_condition.tail<4>()(3) = q_0.z();

    VectorXd p_initial_covariance(12);
    p_initial_covariance.head<3>() = p_cov_v_0;
    p_initial_covariance.segment<3>(3) = p_cov_w_0;
    p_initial_covariance.segment<3>(6) = p_cov_x_0;
    p_initial_covariance.tail<3>() = p_cov_q_0;

    VectorXd v_initial_condition(6);
    v_initial_condition.head<3>() = v_v_0;
    v_initial_condition.tail<3>() = v_w_0;

    VectorXd v_initial_covariance(6);
    v_initial_covariance.head<3>() = v_cov_v_0;
    v_initial_covariance.tail<3>() = v_cov_w_0;

    VectorXd p_model_covariance(6);
    p_model_covariance.head<3>() = sigma_ang_vel;
    p_model_covariance.tail<3>() = psd_lin_acc;

    VectorXd v_model_covariance(6);
    v_model_covariance.head<3>() = kin_q_v;
    v_model_covariance.tail<3>() = kin_q_w;

    VectorXd p_measurement_covariance(12);
    p_measurement_covariance.head<3>() = p_meas_cov_v;
    p_measurement_covariance.segment<3>(3) = p_meas_cov_w;
    p_measurement_covariance.segment<3>(6) = p_meas_cov_x;
    p_measurement_covariance.tail<3>() = p_meas_cov_q;

    VectorXd v_measurement_covariance = v_meas_cov_flow;

    /* Object model. */
    model_parameters_.name(model_name);
    model_parameters_.use_internal_db(model_use_internal_db);
    model_parameters_.internal_db_name(model_internal_db_name);
    model_parameters_.mesh_external_path(model_external_path);
    model_parameters_.textured_mesh_external_path(textured_model_external_path);

    /* Output format. */
    bool yarp_camera_enable_camera_pose;
    if (output_format_reference_frame == "camera")
        yarp_camera_enable_camera_pose = false;
    else if (output_format_reference_frame == "root")
        yarp_camera_enable_camera_pose = true;
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown output format reference frame " + output_format_reference_frame + "."));

    /* Camera. */
    std::shared_ptr<Camera> camera_src;
    if (camera_source == "RealSense")
    {
        camera_src = std::make_shared<RealsenseCameraYarp>(log_name_);
    }
    else if (camera_source == "YARP")
    {
        camera_src = std::make_shared<YarpCamera>(camera_width, camera_height, camera_fx, camera_cx, camera_fy, camera_cy, log_name_ + "/camera", yarp_camera_enable_camera_pose);
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown camera source " + camera_source + "."));
    std::shared_ptr<CameraMeasurement> camera = std::make_shared<CameraMeasurement>(std::move(camera_src));

    /* Segmentation. */
    std::shared_ptr<Segmentation> segmentation;
    if (segmentation_source == "YARP")
    {
        segmentation = std::make_shared<SegmentationYarpPort>("/" + log_name_ + "/segmentation", true);
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown segmentation source " + segmentation_source + "."));

    /* Pose. */
    std::shared_ptr<RobotsIO::Utils::Transform> pose;
    if (pose_source == "YARP")
    {
        pose = std::make_shared<TransformYarpPort>("/" + log_name_ + "/pose", true);
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown pose source " + pose_source + "."));

    /* Flow. */
    std::shared_ptr<ImageOpticalFlowSource> flow;
    if (optical_flow_source == "NVOF_1_0")
    {
        flow = std::make_shared<ROFT::ImageOpticalFlowNVOF>(camera, ROFT::ImageOpticalFlowNVOF::NVOFPerformance_1_0::Slow, false);
    }
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
    else if (optical_flow_source == "NVOF_2_0")
    {
        flow = std::make_shared<ROFT::ImageOpticalFlowNVOF>(camera, ROFT::ImageOpticalFlowNVOF::NVOFPerformance_2_0::Slow, false);
    }
#endif
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown optical flow source " + pose_source + "."));

    /* Filter. */
    filter_ = std::make_unique<ROFTFilter>
    (
        /* Sources. */
        camera, segmentation, flow, pose,
        /* Object model. */
        model_parameters_,
        /* Initial pose and covariance. */
        p_initial_condition, p_initial_covariance, p_model_covariance, p_measurement_covariance,
        /* Initial velocity and covariance. */
        v_initial_condition, v_initial_covariance, v_model_covariance, v_measurement_covariance,
        /* Unscented Transform (UT) parameters. */
        ut_alpha, ut_beta, ut_kappa,
        /* Sample time. */
        sample_time,
        /* Thresholds. */
        depth_maximum, subsampling_radius,
        /* Flags for enabling/disabling internal mechanisms. */
        use_pose_measurement, use_pose_resync, outlier_rejection_enable, outlier_rejection_gain,
        use_velocity_measurement, flow_weighting, flow_aided_segmentation, wait_segmentation_initialization,
        /* The reference frame in which the pose has to be expressed. */
        output_format_reference_frame
    );

    {
        auto probe = std::make_unique<YarpImageOfProbe<PixelRgb>>("/" + log_name_ + "/probe/segmentation:o");
        filter_->set_probe("output_segmentation_refined", std::move(probe));
    }

    {
        auto probe = std::make_unique<YarpImageOfProbe<PixelRgb>>("/" + log_name_ + "/probe/segmentation_source:o");
        filter_->set_probe("output_segmentation_source", std::move(probe));
    }

    {
        auto probe = std::make_unique<YarpImageOfProbe<PixelRgb>>("/" + log_name_ + "/probe/outlier_rejection:o");
        filter_->set_probe("output_outlier_rejection", std::move(probe));
    }

    {
        auto probe = std::make_unique<YarpImageOfProbe<PixelRgb>>("/" + log_name_ + "/probe/pose_render:o");
        filter_->set_probe("output_pose_render", std::move(probe));
    }

    {
        auto probe = std::make_unique<YarpImageOfProbe<PixelRgb>>("/" + log_name_ + "/probe/pose_source_render:o");
        filter_->set_probe("output_pose_source_render", std::move(probe));
    }

    {
        auto probe = std::make_unique<YarpVectorOfProbe<double, Eigen::VectorXd>>("/" + log_name_ + "/probe/velocity:o");
        filter_->set_probe("output_velocity", std::move(probe));
    }

    {
        auto probe = std::make_unique<YarpVectorOfProbe<double, Eigen::VectorXd>>("/" + log_name_ + "/probe/state:o");
        filter_->set_probe("output_state", std::move(probe));
    }

    /* Open RPC port and attach to respond handler. */
    if (!port_rpc_.open("/" + log_name_ + "/rpc:i"))
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open input RPC port"));
    if (!(this->yarp().attachAsServer(port_rpc_)))
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot attach RPC port to the respond handler"));

    /* Boot the filter. */
    filter_->boot();
    filter_->run();
    if (!filter_->wait())
        return;
}


std::string Tracker::quit()
{
    filter_->teardown();

    return "Command accepted.";
}


std::string Tracker::reset()
{
    filter_->reset();

    return "Command accepted.";
}


std::string Tracker::select_object(const std::string& object_name)
{
    model_parameters_.name(object_name);
    model_parameters_.textured_mesh_external_path(textured_model_external_path_root_ + "/" + object_name + "/textured.obj");

    /* Set the new model parameters. */
    filter_->set_model_parameters(model_parameters_);

    return "Command accepted.";
}


std::string Tracker::start()
{
    filter_->run();

    return "Command accepted.";
}


std::string Tracker::stop()
{
    filter_->reboot();

    return "Command accepted.";
}


VectorXd Tracker::load_vector_double(const Bottle& resource, const std::string& key, const std::size_t size)
{
    if (resource.find(key).isNull())
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Cannot find key " + key + "."));

    Bottle* b = resource.find(key).asList();
    if (b == nullptr)
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Cannot get vector having key " + key + " as a list."));


    if (b->size() != size)
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Vector having key " + key + " has size "  + std::to_string(b->size()) + " (expected is " + std::to_string(size) + ")."));

    VectorXd vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double." + std::to_string(i) + "-th element of of vector having key " + key + " is null."));

        if (!item_v.isFloat64())
            throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double." + std::to_string(i) + "-th element of of vector having key " + key + " is not a double."));

        vector(i) = item_v.asFloat64();
    }

    return vector;
}
