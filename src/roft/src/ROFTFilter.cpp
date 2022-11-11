/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFTFilter.h>

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/KFPrediction.h>
#include <BayesFilters/LinearStateModel.h>
#include <BayesFilters/UKFPrediction.h>

#include <ROFT/CartesianQuaternionMeasurement.h>
#include <ROFT/CartesianQuaternionModel.h>
#include <ROFT/ImageSegmentationOFAidedSource.hpp>
#include <ROFT/MeshResource.h>
#include <ROFT/SpatialVelocityModel.h>
#include <ROFT/SKFCorrection.h>
#include <ROFT/UKFCorrection.h>

#include <RobotsIO/Camera/CameraParameters.h>

using namespace Eigen;
using namespace ROFT;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace bfl;


ROFTFilter::ROFTFilter
(
    std::shared_ptr<ROFT::CameraMeasurement> camera_measurement,
    std::shared_ptr<RobotsIO::Utils::Segmentation> segmentation_source,
    std::shared_ptr<ROFT::ImageOpticalFlowSource> flow_source,
    std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement,
    const ModelParameters& model_parameters,
    const Eigen::Ref<const Eigen::VectorXd>& initial_condition_p,
    const Eigen::Ref<const Eigen::VectorXd>& initial_covariance_p,
    const Eigen::Ref<const Eigen::VectorXd>& model_covariance_p,
    const Eigen::Ref<const Eigen::VectorXd>& measurement_covariance_p,
    const Eigen::Ref<const Eigen::VectorXd> initial_condition_v,
    const Eigen::Ref<const Eigen::VectorXd> initial_covariance_v,
    const Eigen::Ref<const Eigen::VectorXd> model_covariance_v,
    const Eigen::Ref<const Eigen::VectorXd> measurement_covariance_v,
    const double& ut_alpha,
    const double& ut_beta,
    const double& ut_kappa,
    const double& sample_time,
    const double& maximum_depth,
    const double& subsampling_radius,
    const bool pose_meas,
    const bool pose_resync,
    const bool pose_outlier_rejection,
    const bool pose_outlier_rejection_gain,
    const bool velocity_meas,
    const bool flow_weighting,
    const bool flow_aided_segmentation,
    const bool wait_segmentation_initialization,
    const std::string& pose_reference_frame,
    const std::string& pose_meas_feedback,
    const std::string& pose_rendering_style
) :
    p_pred_belief_(9, 1, true),
    p_corr_belief_(9, 1, true),
    v_pred_belief_(6, 0, false),
    v_corr_belief_(6, 0, false),
    camera_(camera_measurement),
    sample_time_(sample_time),
    maximum_depth_(maximum_depth),
    outlier_rejection_(pose_outlier_rejection),
    outlier_rejection_gain_(pose_outlier_rejection_gain),
    pose_resync_(pose_resync),
    output_pose_reference_frame_(pose_reference_frame),
    pose_rendering_style_(pose_rendering_style),
    model_parameters_(model_parameters)
{
    /* Extract initial conditions. */
    p_v_0_ = initial_condition_p.head<3>();
    p_w_0_ = initial_condition_p.segment<3>(3);
    p_x_0_ = initial_condition_p.segment<3>(6);
    p_q_0_ = initial_condition_p.tail<4>();

    v_v_0_ = initial_condition_v.head<3>();
    v_w_0_ = initial_condition_v.tail<3>();

    p_covariance_0_ = initial_covariance_p.asDiagonal();

    v_covariance_0_ = initial_covariance_v.asDiagonal();

    /* Extract model power spectral densities and variances .*/
    MatrixXd model_sigma_angular_velocity = model_covariance_p.head<3>().asDiagonal();
    MatrixXd model_psd_linear_acceleration = model_covariance_p.tail<3>().asDiagonal();

    MatrixXd v_cov_v = model_covariance_v.head<3>().asDiagonal();
    MatrixXd v_cov_w = model_covariance_v.tail<3>().asDiagonal();

    /* Extract measurement covariance .*/
    MatrixXd meas_sigma_linear_velocity = measurement_covariance_p.head<3>().asDiagonal();
    MatrixXd meas_sigma_angular_velocity = measurement_covariance_p.segment<3>(3).asDiagonal();
    MatrixXd meas_sigma_position = measurement_covariance_p.segment<3>(6).asDiagonal();
    MatrixXd meas_sigma_quaternion = measurement_covariance_p.tail<3>().asDiagonal();

    MatrixXd v_cov_measurement = measurement_covariance_v.asDiagonal();

    /* Kinematic model. */
    auto p_kinematic_model = std::unique_ptr<CartesianQuaternionModel>
    (
        new CartesianQuaternionModel(model_psd_linear_acceleration, model_sigma_angular_velocity, sample_time)
    );

    auto v_kinematic_model = std::unique_ptr<SpatialVelocityModel>
    (
        new SpatialVelocityModel(v_cov_v, v_cov_w)
    );

    /* These are only required to extract the incoming mask and pose measurements , for visualization purposes. */
    segmentation_source_ = segmentation_source;
    pose_measurement_ = pose_measurement;
    /* */

    /* Segmentation measurement. */
    if (flow_aided_segmentation)
    {
        RobotsIO::Camera::CameraParameters parameters;
        std::tie(std::ignore, parameters) = camera_->camera_parameters();
        if (flow_source->get_matrix_type() == CV_32FC2)
            segmentation_ = std::make_shared<ImageSegmentationMeasurement>(std::make_shared<ImageSegmentationOFAidedSource<cv::Vec2f>>(segmentation_source, flow_source, parameters, wait_segmentation_initialization), camera_);
        else if (flow_source->get_matrix_type() == CV_16SC2)
            segmentation_ = std::make_shared<ImageSegmentationMeasurement>(std::make_shared<ImageSegmentationOFAidedSource<cv::Vec2s>>(segmentation_source, flow_source, parameters, wait_segmentation_initialization), camera_);
    }
    else
        segmentation_ = std::make_shared<ImageSegmentationMeasurement>(segmentation_source, camera_);

    /* Flow measurement. */
    std::unique_ptr<LinearMeasurementModel> flow;
    if (flow_source->get_matrix_type() == CV_32FC2)
    {
        flow = std::unique_ptr<ImageOpticalFlowMeasurement<cv::Vec2f>>
        (
            new ImageOpticalFlowMeasurement<cv::Vec2f>(flow_source, camera_, segmentation_, subsampling_radius, maximum_depth_, v_cov_measurement, false)
        );
    }
    else if (flow_source->get_matrix_type() == CV_16SC2)
    {
        flow = std::unique_ptr<ImageOpticalFlowMeasurement<cv::Vec2s>>
        (
            new ImageOpticalFlowMeasurement<cv::Vec2s>(flow_source, camera_, segmentation_, subsampling_radius, maximum_depth_, v_cov_measurement, false)
        );
    }

    /* Velocity measurement. */
    velocity_ = std::make_shared<SpatialVelocityBuffer>();

    /* Pose/Velocity measurement. */
    auto measurement_model = std::unique_ptr<CartesianQuaternionMeasurement>
    (
        new CartesianQuaternionMeasurement(pose_measurement, velocity_, camera_, segmentation_, false, pose_meas, velocity_meas, meas_sigma_position, meas_sigma_quaternion, meas_sigma_linear_velocity, meas_sigma_angular_velocity, /* wait_pose_initialization = */ true, /* enable_log = */ false)
    );
    if (pose_meas_feedback == "RGB")
        measurement_model->setProperty("transform_feedback_rgb");
    else if (pose_meas_feedback == "DepthSegmentation")
        measurement_model->setProperty("transform_feedback_depth_segmentation");

    /* Prediction. */
    p_prediction_ = std::unique_ptr<UKFPrediction>
    (
        new UKFPrediction(std::move(p_kinematic_model), ut_alpha, ut_beta, ut_kappa)
    );

    v_prediction_ = std::unique_ptr<KFPrediction>
    (
        new KFPrediction(std::move(v_kinematic_model))
    );

    /* Correction. */
    p_correction_ = std::unique_ptr<ROFT::UKFCorrection>
    (
        new ROFT::UKFCorrection(std::move(measurement_model), ut_alpha, ut_beta, ut_kappa)
    );

    v_correction_ = std::unique_ptr<SKFCorrection>
    (
        new SKFCorrection(std::move(flow), 2, flow_weighting)
    );

    /* Initialize renderers. */
    initialize_renderers();
}


ROFTFilter::~ROFTFilter()
{
    disable_log();
}


bool ROFTFilter::run_condition()
{
    return true;
}


bool ROFTFilter::initialization_step()
{
    /* Initialize timers. */
    auto now = std::chrono::steady_clock::now();
    time_since_last_measurement_0_ = now;
    time_since_last_measurement_1_ = now;

    /* Initialize Gaussian belief. */
    v_corr_belief_.mean().head<3>() = v_v_0_;
    v_corr_belief_.mean().tail<3>() = v_w_0_;

    v_corr_belief_.covariance() = v_covariance_0_;

    p_corr_belief_.mean().head<3>() = p_v_0_;
    p_corr_belief_.mean().segment<3>(3) = p_w_0_;
    p_corr_belief_.mean().segment<3>(6) = p_x_0_;
    p_corr_belief_.mean().tail<4>() = p_q_0_;

    p_corr_belief_.covariance() = p_covariance_0_;

    buffered_belief_ = p_corr_belief_;

    /* Reset segmentation. */
    segmentation_->reset();

    /* Reset measurement models. */
    p_correction_->getMeasurementModel().setProperty("reset");
    v_correction_->getMeasurementModel().setProperty("reset");

    /* Reset flags. */
    outlier_rejection_features_initialized_ = false;

    /* Initialize renderers if required. */
    if (!rendering_initialized_)
        initialize_renderers();

    return true;
}


bool ROFTFilter::skip(const std::string& what_step, const bool status)
{
    /* Not implemented. */
    return false;
}


void ROFTFilter::set_model_parameters(const ROFT::ModelParameters& model_parameters)
{
    model_parameters_ = model_parameters;

    /* This will cause rendering reinit when FilteringAlgorithm::initialization() is called.
     No effect until initialization() is called again. */
    rendering_initialized_ = false;
}


std::vector<std::string> ROFTFilter::log_file_names(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "pose_estimate",
             prefix_path + "/" + prefix_name + "velocity_estimate",
             prefix_path + "/" + prefix_name + "execution_times"};
}


void ROFTFilter::filtering_step()
{
    /* Check if we are stuck. */
    auto now = std::chrono::steady_clock::now();
    double elapsed_0 = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_since_last_measurement_0_).count() / 1000.0;
    double elapsed_1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_since_last_measurement_1_).count() / 1000.0;
    if ((elapsed_0 > timeout_) || (elapsed_1 > timeout_))
    {
        time_since_last_measurement_0_ = now;
        time_since_last_measurement_1_ = now;
        reset();
        std::cout << log_name_ + "::filteringStep. Warning: resetting as not hearing from the pose/segmentation sources." << std::endl;
        return;
    }

    bool data_in;

    /* Freeze camera. */
    if (!(data_in = camera_->freeze(CameraMeasurementType::RGBD)))
    {
        std::cout << log_name_ + "::filteringStep. Error: cannot continue without a continuous camera stream" << std::endl;
        teardown();
        return;
    }

    /* Evaluate difference between current and previous image time stamp. */
    double elapsed_time = sample_time_;
    double camera_stamp;
    std::tie(std::ignore, camera_stamp) = camera_->camera_time_stamp_rgb();
    if (last_camera_stamp_ != -1)
        elapsed_time = camera_stamp - last_camera_stamp_;
    last_camera_stamp_ = camera_stamp;
    p_prediction_->getStateModel().setSamplingTime(elapsed_time);

    /* Ask for flow source stepping beforehand
       This enables optical flow aided segmentation sources to work in case. */
    v_correction_->getMeasurementModel().freeze(std::make_pair(ImageOpticalFlowMeasurementBase::FreezeType::OnlyStepSource, elapsed_time));

    /* Freeze segmentation. */
    data_in &= segmentation_->freeze();

    if (!data_in)
        return;

    /* Freeze flow. */
    data_in &= v_correction_->getMeasurementModel().freeze(std::make_pair(ImageOpticalFlowMeasurementBase::FreezeType::ExceptStepSource, elapsed_time));

    /* Check if segmentation source is still alive. */
    bool is_segmentation_alive;
    std::tie(is_segmentation_alive, std::ignore) = segmentation_source_->latest_segmentation();
    if (is_segmentation_alive)
        time_since_last_measurement_1_ = std::chrono::steady_clock::now();

    if (data_in)
    {
        /* UKF velocity filtering. */
        Gaussian v_corr_belief_copy = v_corr_belief_;
        v_prediction_->predict(v_corr_belief_, v_pred_belief_);
        v_correction_->correct(v_pred_belief_, v_corr_belief_);
        if (!v_correction_->getMeasurementModel().setProperty("check_observability"))
        {
            std::cout << "The state is unobservable. Skipping this correction." << std::endl;
            return;
        }
    }

    /* Populate velocity buffer for subsequent filtering steps. */
    velocity_->set_twist(v_corr_belief_.mean().head<3>(), v_corr_belief_.mean().tail<3>());

    /* Get refined segmentation. */
    cv::Mat segmentation_image;
    bfl::Data segmentation_data;
    std::tie(std::ignore, segmentation_data) = segmentation_->measure();
    std::tie(std::ignore, segmentation_image) = any::any_cast<std::pair<bool, cv::Mat>>(segmentation_data);

    if (pose_resync_)
    {
        if (!outlier_rejection_features_initialized_)
        {
            if (!buffer_outlier_rejection_features())
                throw(std::runtime_error(log_name_ + "::filteringStep(). Error: cannot initialize the outlier rejection features."));

            outlier_rejection_features_initialized_ = true;
        }
    }

    /* This is only required to extract the incoming pose measurement, for visualization purposes. */
    VectorXd pose_measurement;
    MatrixXd bbox_measured_points;

    /* UKF pose filtering. */
    p_prediction_->predict(p_corr_belief_, p_pred_belief_);

    if (p_correction_->getMeasurementModel().freeze(CartesianQuaternionMeasurement::MeasurementMode::Standard))
    {
        if (p_correction_->getMeasurementModel().getMeasurementDescription().total_size() == 13)
        {
            time_since_last_measurement_0_ = std::chrono::steady_clock::now();

            /* This is only required to extract the incoming pose measurement, for visualization purposes. */
            MatrixXd measurement;
            bfl::Data measurement_data;
            std::tie(std::ignore, measurement_data) = p_correction_->getMeasurementModel().measure();
            measurement = bfl::any::any_cast<MatrixXd>(measurement_data);
            pose_measurement = measurement.col(0);

            bbox_measured_points = pose_measurement_->bounding_box();
            /* */

            if (pose_resync_)
            {
                /* Copy last buffered belief. */
                Gaussian buffered_belief_copy = buffered_belief_;

                /* Store the corrected belief for the next pose re-sync. */
                buffered_belief_ = p_corr_belief_;

                /* Reset to the buffered belief. */
                p_corr_belief_ = buffered_belief_copy;

                while(p_correction_->getMeasurementModel().freeze(CartesianQuaternionMeasurement::MeasurementMode::PopBufferedMeasurement))
                {
                    p_prediction_->predict(p_corr_belief_, p_pred_belief_);

                    if (outlier_rejection_ && p_correction_->getMeasurementModel().getMeasurementDescription().total_size() == 13)
                        p_corr_belief_ = correct_outlier_rejection(p_pred_belief_, true);
                    else
                        p_correction_->correct(p_pred_belief_, p_corr_belief_);
                }

                /* Store the outlier rejection features for the next pose re-sync. */
                buffer_outlier_rejection_features();
            }
            else
            {
                if (outlier_rejection_)
                    p_corr_belief_ = correct_outlier_rejection(p_pred_belief_, false);
                else
                    p_correction_->correct(p_pred_belief_, p_corr_belief_);
            }
        }
        else
            p_correction_->correct(p_pred_belief_, p_corr_belief_);
    }
    else
        p_corr_belief_ = p_pred_belief_;

    /* Update internal bounding box representation. */
    update_bounding_box_representation(p_corr_belief_.mean(), pose_measurement, bbox_measured_points);

    /* Get current RGB input and pose, required to provide several outputs. */
    bfl::Data camera_data;
    std::tie(std::ignore, camera_data) = camera_->measure();
    cv::Mat rgb;
    Eigen::Transform<double, 3, Affine> camera_pose;
    std::tie(camera_pose, rgb, std::ignore) = bfl::any::any_cast<CameraMeasurement::CameraMeasurementTuple>(camera_data);

    /* Output segmentation masks. */
    if (is_probe("output_segmentation_refined"))
    {
        cv::Mat rgb_frame = rgb.clone();

        /* Draw synchronized segmentation stored in variable segmentation_image. */
        cv::Mat mask = rgb.clone();
        mask.setTo(cv::Scalar(0, 255, 0), segmentation_image);

        double alpha = 0.8;
        cv::addWeighted(mask, alpha, rgb_frame, 1 - alpha, 0, rgb_frame);

        /* Send data to the probe. */
        get_probe("output_segmentation_refined").set_data(rgb_frame);
    }
    if (is_probe("output_segmentation_source"))
    {
        cv::Mat rgb_frame = rgb.clone();

        /* Get current segmentation from the segmentation source. */
        bool is_segmentation;
        cv::Mat segmentation;
        std::tie(is_segmentation, segmentation) = segmentation_source_->latest_segmentation();

        if (is_segmentation)
        {
            /* Draw segmentation. */
            cv::Mat mask = rgb.clone();
            mask.setTo(cv::Scalar(0, 0, 255), segmentation);

            double alpha = 0.8;
            cv::addWeighted(mask, alpha, rgb_frame, 1 - alpha, 0, rgb_frame);

            /* Send data to the probe. */
            get_probe("output_segmentation_source").set_data(rgb_frame);
        }
    }

    /* Output object pose render over RGB input. */
    cv::Mat render_0;
    cv::Mat render_1;
    if (is_probe("output_pose_render") || is_probe("output_pose_source_render"))
    {
        cv::Mat rgb_frame = rgb.clone();
        if (pose_rendering_style_ == "mesh")
            std::tie(render_0, render_1) = render_pose_as_mesh(rgb_frame, p_corr_belief_.mean(), pose_measurement);
        else if (pose_rendering_style_ == "bounding_box")
            std::tie(render_0, render_1) = render_pose_as_bounding_box(rgb_frame, bbox_measured_points);

    }
    if ((pose_rendering_style_ == "mesh") || (pose_rendering_style_ == "bounding_box"))
    {
        if (is_probe("output_pose_render"))
            get_probe("output_pose_render").set_data(render_0);
        if (is_probe("output_pose_source_render") && (!render_1.empty()))
            get_probe("output_pose_source_render").set_data(render_1);
    }

    /* Output object pose and velocity. */
    VectorXd p_mean(13);
    p_mean.head<6>() = p_corr_belief_.mean().head<6>();
    p_mean.segment<3>(6) = p_corr_belief_.mean().segment<3>(6);
    Quaterniond quaternion(p_corr_belief_.mean(9), p_corr_belief_.mean(10), p_corr_belief_.mean(11), p_corr_belief_.mean(12));
    AngleAxisd angle_axis(quaternion);
    p_mean.segment<3>(9) = angle_axis.axis();
    p_mean(12) = angle_axis.angle();

    MatrixXd bbox_tracked_points = bbox_tracked_points_;
    if (output_pose_reference_frame_ == "root")
    {
        /* Transform the output pose in the root frame,
           taking into account the pose of the camera with respect to the root. */

        /* Warning: the velocity is not transformed as the camera velocity is not available. */

        Eigen::Transform<double, 3, Affine> object_pose;
        object_pose = Translation<double, 3>(p_mean.segment<3>(6));
        object_pose.rotate(quaternion);

        Eigen::Transform<double, 3, Affine> object_root_pose = camera_pose * object_pose;

        p_mean.segment<3>(6) = object_root_pose.translation();
        AngleAxisd angle_axis_root(object_root_pose.rotation());
        p_mean.segment<3>(9) = angle_axis_root.axis();
        p_mean(12) = angle_axis_root.angle();

        if (bbox_tracked_points.size() != 0)
            bbox_tracked_points = camera_pose * bbox_tracked_points.colwise().homogeneous();
    }

    VectorXd v_mean = v_corr_belief_.mean();

    if (is_probe("output_state"))
    {
        std::size_t total_size = 13 + bbox_tracked_points.size();

        VectorXd state(total_size);
        state.head<7>() = p_mean.tail<7>();
        state.tail<6>() = p_mean.head<6>();

        if (bbox_tracked_points.size() != 0)
            for (std::size_t i = 0; i < bbox_tracked_points.cols(); i++)
            {
                state(13 + i * 3 + 0) = bbox_tracked_points.col(i)(0);
                state(13 + i * 3 + 1) = bbox_tracked_points.col(i)(1);
                state(13 + i * 3 + 2) = bbox_tracked_points.col(i)(2);
            }

        get_probe("output_state").set_data(state);
    }

    if (is_probe("output_velocity"))
        get_probe("output_velocity").set_data(v_mean);
}


void ROFTFilter::start_time_count()
{
    std_time_0_ = std::chrono::steady_clock::now();
}


double ROFTFilter::stop_time_count()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - std_time_0_).count();
}


std::pair<bool, bfl::Gaussian> ROFTFilter::pick_best_alternative(const std::vector<bfl::Gaussian>& alternatives, const bool use_buffered_features)
{
    auto failed = std::make_pair(false, Gaussian());

    MatrixXf depth;
    cv::Mat segmentation;

    if (use_buffered_features)
    {
        depth = buffered_depth_;
        segmentation = buffered_segmentation_;
    }
    else
    {
        /* Measure depth. */
        bfl::Data camera_data;
        bool valid_data = false;
        std::tie(valid_data, camera_data) = camera_->measure();
        if (!valid_data)
            return failed;

        std::tie(std::ignore, std::ignore, depth) = any::any_cast<CameraMeasurement::CameraMeasurementTuple>(camera_data);

        /* Measure segmentation. */
        bfl::Data segmentation_data;
        bool valid_segmentation = false;
        std::tie(valid_segmentation, segmentation_data) = segmentation_->measure();
        if (!valid_segmentation)
            return failed;

        std::tie(std::ignore, segmentation) = any::any_cast<std::pair<bool, cv::Mat>>(segmentation_data);
    }

    /* Find non zero coordinates within the segmentation. */
    cv::Mat coordinates;
    cv::findNonZero(segmentation, coordinates);

    /* Render depth. */
    cv::Mat placeholder;
    cv::Mat rendered_depth;

    double cam_x [4] = {0.0, 0.0, 0.0};
    double cam_o [4] = {1.0, 0.0, 0.0, 0.0};

    std::vector<SICAD::ModelPoseContainer> poses;
    for (std::size_t i = 0; i < alternatives.size(); i++)
    {
        const Ref<const VectorXd> alternative = alternatives.at(i).mean();
        SICAD::ModelPose pose;
        pose.push_back(alternative(6));
        pose.push_back(alternative(7));
        pose.push_back(alternative(8));

        AngleAxisd angle_axis(Quaterniond(alternative(9), alternative(10), alternative(11), alternative(12)));
        Vector3d axis = angle_axis.axis();
        pose.push_back(axis(0));
        pose.push_back(axis(1));
        pose.push_back(axis(2));
        pose.push_back(angle_axis.angle());

        SICAD::ModelPoseContainer container;
        container.emplace("object", pose);
        poses.push_back(container);
    }

    bool render_outcome = renderer_->superimpose(poses, cam_x, cam_o, placeholder, rendered_depth);

    cv::Mat mask_float;
    cv::Mat mask;
    cv::threshold(rendered_depth, mask_float, 0.001, 255, cv::THRESH_BINARY);
    mask_float.convertTo(mask, CV_8UC1);

    if (render_outcome)
    {
        VectorXd likelihoods(alternatives.size());
        std::size_t q = 0;
        for (std::size_t i = 0; i < rendered_depth.rows; i += camera_parameters_.height() / divider_)
        {
            for (std::size_t j = 0; j < rendered_depth.cols; j+= camera_parameters_.width() / divider_)
            {
                /* Extract the depth relative to one of the alternatives. */
                cv::Mat depth_alternative = cv::Mat(rendered_depth, cv::Rect(j, i, camera_parameters_.width() / divider_, camera_parameters_.height() / divider_));

                cv::Mat mask_alternative = cv::Mat(mask, cv::Rect(j, i, camera_parameters_.width() / divider_, camera_parameters_.height() / divider_));
                int pixel_in_alternative = cv::countNonZero(mask_alternative);

                /* Evaluate the depth error on the segmentation mask .*/
                double error = 0.0;
                std::size_t samples = 0;
                for (std::size_t k = 0; k < coordinates.total(); k+=2)
                {
                    const int& u = coordinates.at<cv::Point>(k).x;
                    const int& v = coordinates.at<cv::Point>(k).y;

                    if ((depth(v, u) > 0) && (depth(v, u) < 2.0) && (depth_alternative.at<float>(v / divider_, u / divider_) != 0.0))
                    {
                        error += std::abs(depth(v, u) - depth_alternative.at<float>(v / divider_, u / divider_));
                        samples++;
                    }
                }
                error /= samples;

                if (samples == 0)
                {
                    // This might happen if either the depth or the rendered depth are invalid within the current segmentation mask
                    // In this case we assign the biggest number possible here
                    likelihoods(q) = std::numeric_limits<double>::max();
                }
                else
                    likelihoods(q) = error / outlier_rejection_gain_;
                q++;
            }
        }

        std::size_t selected = 0;
        if (likelihoods(0) > 2.0 * likelihoods(1))
            selected = 1;

        /* Show the outlier rejection mechanism. */
        if (is_probe("output_outlier_rejection"))
        {
            std::size_t width = camera_parameters_.width() / divider_;
            std::size_t height = camera_parameters_.height() / divider_;
            std::size_t x = 0;
            std::size_t y = 0;
            if (selected == 1)
                x = width;

            cv::Mat mask_best = cv::Mat(mask, cv::Rect(x, y, width, height));

            std::vector<std::vector<cv::Point>> contours;
            findContours(mask_best, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
            for (std::size_t i = 0; i < contours.size(); i++)
                cv::drawContours(cv::Mat(mask, cv::Rect(x, y, width, height)), contours, i, cv::Scalar(0, 0, 255), 4);

            std::vector<std::vector<cv::Point>> contours_segmentation;
            cv::resize(segmentation, segmentation, cv::Size(width, height));
            findContours(segmentation, contours_segmentation, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (std::size_t i = 0; i < rendered_depth.rows; i += height)
                for (std::size_t j = 0; j < rendered_depth.cols; j+= width)
                    for (std::size_t k = 0; k < contours_segmentation.size(); k++)
                        cv::drawContours(cv::Mat(mask, cv::Rect(j, i, width, height)), contours_segmentation, k, cv::Scalar(0, 255, 0), 4);

            get_probe("output_outlier_rejection").set_data(mask);
        }

        return std::make_pair(true, alternatives.at(selected));
    }

    return failed;
}


bool ROFTFilter::buffer_outlier_rejection_features()
{
    /* Measure depth. */
    bfl::Data camera_data;
    bool valid_data = false;
    std::tie(valid_data, camera_data) = camera_->measure();
    if (!valid_data)
        return false;

    std::tie(std::ignore, std::ignore, buffered_depth_) = any::any_cast<CameraMeasurement::CameraMeasurementTuple>(camera_data);

    /* Measure segmentation. */
    bfl::Data segmentation_data;
    bool valid_segmentation = false;
    std::tie(valid_segmentation, segmentation_data) = segmentation_->measure();
    if (!valid_segmentation)
        return false;

    cv::Mat segmentation;
    std::tie(std::ignore, buffered_segmentation_) = any::any_cast<std::pair<bool, cv::Mat>>(segmentation_data);

    return true;
}


Gaussian ROFTFilter::correct_outlier_rejection(const Gaussian& prediction, const bool use_buffered_features)
{
    Gaussian correction = prediction;
    Gaussian corr_belief_v = prediction;
    Gaussian corr_belief_p_v = prediction;

    /* Standard correction. */
    p_correction_->correct(prediction, corr_belief_p_v);

    /* Correction with velocity only. */
    p_correction_->getMeasurementModel().freeze(CartesianQuaternionMeasurement::MeasurementMode::RepeatOnlyVelocity);
    p_correction_->correct(prediction, corr_belief_v);

    std::vector<Gaussian> alternatives;
    alternatives.push_back(corr_belief_p_v);
    alternatives.push_back(corr_belief_v);

    bool valid_alternative_selection;
    Gaussian best_alternative;
    std::tie(valid_alternative_selection, best_alternative) = pick_best_alternative(alternatives, use_buffered_features);

    if (valid_alternative_selection)
        correction = best_alternative;
    else
        correction = corr_belief_p_v;

    return correction;
}


std::pair<cv::Mat, cv::Mat> ROFTFilter::render_pose_as_mesh(const cv::Mat& rgb_frame, const VectorXd& tracker_pose, const VectorXd& pose_measurement)
{
    std::vector<SICAD::ModelPoseContainer> poses;
    for (const VectorXd& pose : {tracker_pose, pose_measurement})
    {
        if (pose.rows() != 0)
        {
            SICAD::ModelPose sicad_pose;

            Quaterniond quaternion(pose(9), pose(10), pose(11), pose(12));
            AngleAxisd angle_axis(quaternion);

            sicad_pose.push_back(pose(6));
            sicad_pose.push_back(pose(7));
            sicad_pose.push_back(pose(8));
            sicad_pose.push_back(angle_axis.axis()(0));
            sicad_pose.push_back(angle_axis.axis()(1));
            sicad_pose.push_back(angle_axis.axis()(2));
            sicad_pose.push_back(angle_axis.angle());

            SICAD::ModelPoseContainer sicad_pose_container;
            sicad_pose_container.emplace("object", sicad_pose);
            poses.push_back(sicad_pose_container);
        }
    }

    cv::Mat render_0;
    cv::Mat render_1;

    /* Camera pose placeholder. */
    double cam_x [4] = {0.0, 0.0, 0.0};
    double cam_o [4] = {1.0, 0.0, 0.0, 0.0};

    /* Render on grayed out background. */
    cv::cvtColor(rgb_frame, render_0, cv::COLOR_RGB2GRAY);
    cv::cvtColor(render_0, render_0, cv::COLOR_GRAY2RGB);

    /* Do rendering if the position is not all zeros. */
    if (!tracker_pose.segment<3>(6).isZero(1e-4))
    {
        output_renderer_0_->superimpose(poses.at(0), cam_x, cam_o, render_0);
        cv::cvtColor(render_0, render_0, cv::COLOR_RGB2BGR);
    }

    if (poses.size() == 2)
    {
        /* Render on grayed out background. */
        cv::cvtColor(rgb_frame, render_1, cv::COLOR_RGB2GRAY);
        cv::cvtColor(render_1, render_1, cv::COLOR_GRAY2RGB);

        output_renderer_1_->superimpose(poses.at(1), cam_x, cam_o, render_1);
        cv::cvtColor(render_1, render_1, cv::COLOR_RGB2BGR);
    }

    return std::make_pair(render_0, render_1);
}


void ROFTFilter::update_bounding_box_representation(const VectorXd& tracker_pose, const VectorXd& pose_measurement, const MatrixXd& measured_points)
{
    if (measured_points.size() != 0)
    {
        const Vector3d& center = pose_measurement.segment<3>(6);
        Quaterniond quaternion(pose_measurement(9), pose_measurement(10), pose_measurement(11), pose_measurement(12));
        Matrix3d rotation = quaternion.toRotationMatrix();

        bbox_local_points_ = rotation.transpose() * (measured_points.colwise() + (-center));
    }

    if (bbox_local_points_.size() != 0)
    {
        const Vector3d& center = tracker_pose.segment<3>(6);
        Quaterniond quaternion(tracker_pose(9), tracker_pose(10), tracker_pose(11), tracker_pose(12));
        Matrix3d rotation = quaternion.toRotationMatrix();

        /* Evaluate the new points only if the position is not all zeros. */
        if (center.isZero(1e-4))
            bbox_tracked_points_ = MatrixXd();
        else
            bbox_tracked_points_ = (rotation * bbox_local_points_).colwise() + center;
    }
}


std::pair<cv::Mat, cv::Mat> ROFTFilter::render_pose_as_bounding_box(const cv::Mat& rgb_frame, const MatrixXd& measured_points)
{
    /* Utilities. */
    auto index_to_str = [](const Eigen::MatrixXd& local_points) -> std::unordered_map<int, std::string>
    {
        std::unordered_map<int, std::string> i_to_str;
        for (std::size_t i = 0; i < local_points.cols(); i++)
        {
            const double& x = local_points.col(i)(0);
            const double& y = local_points.col(i)(1);
            const double& z = local_points.col(i)(2);

            if (x > 0 && y > 0 && z > 0)
                i_to_str[i] = "top_right_front";
            else if (x > 0 && y > 0 && z < 0)
                i_to_str[i] = "top_right_back";
            else if (x > 0 && y < 0 && z > 0)
                i_to_str[i] = "top_left_front";
            else if (x > 0 && y < 0 && z < 0)
                i_to_str[i] = "top_left_back";
            else if (x < 0 && y > 0 && z > 0)
                i_to_str[i] = "bottom_right_front";
            else if (x < 0 && y > 0 && z < 0)
                i_to_str[i] = "bottom_right_back";
            else if (x < 0 && y < 0 && z > 0)
                i_to_str[i] = "bottom_left_front";
            else if (x < 0 && y < 0 && z < 0)
                i_to_str[i] = "bottom_left_back";
        }

        return i_to_str;
    };

    auto str_to_uv = [this] (const Eigen::MatrixXd& global_points, const std::unordered_map<int, std::string>& i_to_str) -> std::unordered_map<std::string, cv::Point>
    {

        std::unordered_map<std::string, cv::Point> str_to_point;

        for (std::size_t i = 0; i < global_points.cols(); i++)
        {
            const VectorXd& point = global_points.col(i);
            const unsigned int u = camera_parameters_.cx() + camera_parameters_.fx() * point(0) / point(2);
            const unsigned int v = camera_parameters_.cy() + camera_parameters_.fy() * point(1) / point(2);
            str_to_point[i_to_str.at(i)] = cv::Point(u, v);
        }

        return str_to_point;
    };

    auto draw_points = [] (const std::unordered_map<std::string, std::vector<std::string>>& structure, const std::unordered_map<std::string, cv::Point>& mapping, cv::Scalar& color, cv::Mat& output)
    {
        for (const auto& item : structure)
        {
            const std::string& parent = item.first;
            cv::circle(output, mapping.at(parent), 5, color, cv::FILLED);

            const std::vector<std::string>& children = item.second;
            for (const auto& child : children)
                cv::line(output, mapping.at(parent), mapping.at(child), color, 1.5);
        }
    };

    std::unordered_map<std::string, std::vector<std::string>> box_structure
    {
        {"top_right_front", {"bottom_right_front", "top_left_front", "top_right_back"}},//, "top_left_back"}},
        {"top_left_back", {"bottom_left_back", "top_left_front", "top_right_back"}},
        {"top_right_back", {"bottom_right_back"}},
        {"top_left_front", {"bottom_left_front"}},
        {"bottom_right_front", {"bottom_left_front", "bottom_right_back"}},
        {"bottom_left_back", {"bottom_left_front", "bottom_right_back"}},
        {"bottom_right_back", {}},
        {"bottom_left_front", {}}
    };

    /* */
    cv::Mat render_0;
    cv::Mat render_1;

    render_0 = rgb_frame.clone();

    if ((measured_points.size() != 0) && (bbox_local_points_.size() != 0))
    {
        render_1 = rgb_frame.clone();

        auto color = cv::Scalar(0, 0, 255);
        bbox_local_points_mapping_ = index_to_str(bbox_local_points_);
        auto box_str_to_uv = str_to_uv(measured_points, bbox_local_points_mapping_);
        draw_points(box_structure, box_str_to_uv, color, render_1);
    }

    if (bbox_tracked_points_.size() != 0)
    {
        auto color = cv::Scalar(0, 255, 0);
        auto box_str_to_uv = str_to_uv(bbox_tracked_points_, bbox_local_points_mapping_);
        draw_points(box_structure, box_str_to_uv, color, render_0);
    }

    return std::make_pair(render_0, render_1);
}


void ROFTFilter::initialize_renderers()
{
    /* Depth rendering for outlier rejection. */
    std::tie(std::ignore, camera_parameters_) = camera_->camera_parameters();

    if (outlier_rejection_)
    {
        MeshResource mesh_resource(model_parameters_);
        std::istringstream mesh_resource_stream(mesh_resource.as_string());
        SICAD::ModelStreamContainer model;
        model["object"] = &mesh_resource_stream;
        std::size_t desired_images = 2;
        divider_ = 4;
        if (camera_parameters_.width() == 640)
            divider_ = 2;
        renderer_ = std::unique_ptr<SICAD>
            (
                new SICAD
                (
                    model,
                    camera_parameters_.width() / divider_, camera_parameters_.height() / divider_,
                    camera_parameters_.fx() / divider_, camera_parameters_.fy() / divider_,
                    camera_parameters_.cx() / divider_, camera_parameters_.cy() / divider_,
                    desired_images
                    )
                );
        renderer_->setOglToCam({1.0, 0.0, 0.0, static_cast<float>(M_PI)});
        if (renderer_->getTilesNumber() != desired_images)
            throw(std::runtime_error(log_name_ + "::ctor. Depth rendering cannot provide desired_images = " + std::to_string(desired_images) + " images per iteration."));
    }

    /* Output rendering. */
    if (pose_rendering_style_ == "mesh")
    {
        if (model_parameters_.textured_mesh_external_path().empty())
        {
            std::cout << log_name_ + "::ctor. Warning: the path to the textured mesh model is not available. "
                      << "Output rendering will not be available." << std::endl;
        }
        else
        {
            SICAD::ModelPathContainer model;
            model["object"] = model_parameters_.textured_mesh_external_path();

            output_renderer_0_ = std::unique_ptr<SICAD>
                (
                    new SICAD
                    (
                        model,
                        camera_parameters_.width(), camera_parameters_.height(),
                        camera_parameters_.fx(), camera_parameters_.fy(),
                        camera_parameters_.cx(), camera_parameters_.cy(),
                        1
                        )
                    );
            output_renderer_0_->setBackgroundOpt(true);
            output_renderer_0_->setOglToCam({1.0, 0.0, 0.0, static_cast<float>(M_PI)});
            if (output_renderer_0_->getTilesNumber() != 1)
                throw(std::runtime_error(log_name_ + "::ctor. Output rendering cannot allocate space."));

            output_renderer_1_ = std::unique_ptr<SICAD>
                (
                    new SICAD
                    (
                        model,
                        camera_parameters_.width(), camera_parameters_.height(),
                        camera_parameters_.fx(), camera_parameters_.fy(),
                        camera_parameters_.cx(), camera_parameters_.cy(),
                        1
                        )
                    );
            output_renderer_1_->setBackgroundOpt(true);
            output_renderer_1_->setOglToCam({1.0, 0.0, 0.0, static_cast<float>(M_PI)});
            if (output_renderer_1_->getTilesNumber() != 1)
                throw(std::runtime_error(log_name_ + "::ctor. Output rendering cannot allocate space."));
        }
    }

    /* Set status flag. */
    rendering_initialized_ = true;
}
