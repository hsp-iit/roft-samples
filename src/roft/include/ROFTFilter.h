/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFTFILTER_H
#define ROFTFILTER_H

#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianPrediction.h>

#include <Eigen/Dense>

#include <ROFT/CameraMeasurement.h>
#include <ROFT/ImageOpticalFlowMeasurement.hpp>
#include <ROFT/ImageOpticalFlowSource.h>
#include <ROFT/ImageSegmentationMeasurement.h>
#include <ROFT/ModelParameters.h>
#include <ROFT/SICAD.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Utils/ProbeContainer.h>
#include <RobotsIO/Utils/Segmentation.h>
#include <RobotsIO/Utils/SpatialVelocityBuffer.h>
#include <RobotsIO/Utils/Transform.h>

#include <opencv2/opencv.hpp>


class ROFTFilter : public bfl::FilteringAlgorithm,
                   public RobotsIO::Utils::ProbeContainer
{
public:
    ROFTFilter
    (
        std::shared_ptr<ROFT::CameraMeasurement> camera_measurement,
        std::shared_ptr<RobotsIO::Utils::Segmentation> segmentation_source,
        std::shared_ptr<ROFT::ImageOpticalFlowSource> flow_source,
        std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement,
        const ROFT::ModelParameters& model_parameters,
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
        const std::string& pose_meas_feedback
    );

    virtual ~ROFTFilter();

    bool run_condition() override;

    bool initialization_step() override;

    bool skip(const std::string& what_step, const bool status) override;

    void set_model_parameters(const ROFT::ModelParameters& model_parameters);

protected:
    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void filtering_step() override;

private:
    void start_time_count();

    double stop_time_count();

    std::pair<bool, bfl::Gaussian> pick_best_alternative(const std::vector<bfl::Gaussian>& alternatives, const bool use_buffered_features);

    bool buffer_outlier_rejection_features();

    bfl::Gaussian correct_outlier_rejection(const bfl::Gaussian& prediction, const bool use_buffered_features);

    std::pair<cv::Mat, cv::Mat> render_pose(const cv::Mat& rgb_frame, const Eigen::VectorXd& tracker_pose, const Eigen::VectorXd& pose_measurement);

    void initialize_renderers();

    /* Initial conditions for flow aided velocity estimation. */

    Eigen::VectorXd v_v_0_;

    Eigen::VectorXd v_w_0_;

    Eigen::MatrixXd v_covariance_0_;

    /* Initial conditions for pose tracking .*/

    Eigen::VectorXd p_v_0_;

    Eigen::VectorXd p_w_0_;

    Eigen::VectorXd p_x_0_;

    Eigen::VectorXd p_q_0_;

    Eigen::MatrixXd p_covariance_0_;

    /* Measurement models. */

    std::shared_ptr<ROFT::CameraMeasurement> camera_;

    std::shared_ptr<ROFT::ImageSegmentationMeasurement> segmentation_;

    std::shared_ptr<RobotsIO::Utils::Segmentation> segmentation_source_;

    std::shared_ptr<RobotsIO::Utils::SpatialVelocityBuffer> velocity_;

    const double maximum_depth_;

    /* Prediction and correction. */

    std::unique_ptr<bfl::GaussianPrediction> p_prediction_;

    std::unique_ptr<bfl::GaussianCorrection> p_correction_;

    std::unique_ptr<bfl::GaussianPrediction> v_prediction_;

    std::unique_ptr<bfl::GaussianCorrection> v_correction_;

    /* Beliefs. */

    bfl::Gaussian p_pred_belief_;

    bfl::Gaussian p_corr_belief_;

    bfl::Gaussian v_pred_belief_;

    bfl::Gaussian v_corr_belief_;

    /* Depth measurement and rendering for outlier rejection .*/

    RobotsIO::Camera::CameraParameters camera_parameters_;

    std::unique_ptr<ROFT::SICAD> renderer_;

    const bool outlier_rejection_;

    const double outlier_rejection_gain_;

    int divider_ = 4;

    /* Pose re-synchronization related. */

    bfl::Gaussian buffered_belief_;

    Eigen::MatrixXf buffered_depth_;

    cv::Mat buffered_segmentation_;

    const bool pose_resync_;

    bool outlier_rejection_features_initialized_ = false;

    /* Timing. */

    std::chrono::steady_clock::time_point std_time_0_;

    double last_camera_stamp_ = -1;

    const double sample_time_;

    /* Output related. */

    std::unique_ptr<ROFT::SICAD> output_renderer_0_;

    std::unique_ptr<ROFT::SICAD> output_renderer_1_;

    const std::string output_pose_reference_frame_;

    /* Renderers initialization status and related. */

    bool rendering_initialized_ = false;

    ROFT::ModelParameters model_parameters_;

    /* Log name. */

    const std::string log_name_ = "OFAidedFilterOnline";
};

#endif /* ROFTFILTER_H */
