/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MODULE_H
#define MODULE_H

#include <Eigen/Dense>

#include <iCubCartesian.h>
#include <iCubGaze.h>
#include <iCubMotorsPositions.h>
#include <Utils.h>
#include <cardinal_points_grasp.h>

#include <thrift/GraspData.h>
#include <thrift/ModuleIDL.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <chrono>
#include <string>
#include <unordered_map>


class Module : public yarp::os::RFModule,
               public ModuleIDL
{
public:
    bool configure(yarp::os::ResourceFinder& rf) override;

    bool close() override;

    double getPeriod() override;

    bool updateModule() override;

    /**
     * IDL interface.
     */

    std::string select_object(const std::string& object_name) override;

private:
    /**
     * Get object state and validation.
     */
    std::tuple<bool, Eigen::Transform<double, 3, Eigen::Affine>, Eigen::Vector3d, Eigen::MatrixXd> get_object_state();

    bool is_pose_gaze_safe(const Eigen::Transform<double, 3, Eigen::Affine>& pose);

    bool is_pose_grasp_safe(const Eigen::Transform<double, 3, Eigen::Affine>& pose);

    bool is_position_cart_safe(const yarp::sig::Vector& position);

    bool is_object_steady(const Eigen::Vector3d& velocity);

    /**
     * Get/set reception time and elapsed time.
     */
    double get_rx_elapsed_time();

    void set_rx_time();

    /**
     * Timer counting for the main state machine of the module.
     */
    void start_counting(const double& total_time);

    bool is_elapsed_from_start_counting();

    /**
     * Send an RPC message.
     */
    bool send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages);

    /**
     * Move robot to home configuration.
     */
    void go_home_arm();

    void go_home_hand();

    /**
     * Grasping.
     */
    bool execute_grasp(const Eigen::Transform<double, 3, Eigen::Affine>& pose, const Eigen::MatrixXd& object_points, const Eigen::Transform<double, 3, Eigen::Affine>& feedback, const bool& valid_feedback);

    /**
     * Face expression handling.
     */
    void set_face_expression(const std::string& type);

    /**
     * iCub gaze controller.
     */
    std::shared_ptr<iCubGaze> gaze_;

    bool enable_gaze_limit_x_;
    bool enable_gaze_limit_y_;
    bool enable_gaze_limit_z_;
    bool enable_gaze_lower_limit_x_;

    double gaze_limit_x_;
    double gaze_limit_y_;
    double gaze_limit_z_;
    double gaze_lower_limit_x_;

    double grasp_tweak_rot_y_;

    /**
     * iCub Cartesian controllers.
     */
    std::shared_ptr<iCubCartesian> cart_left_;
    std::shared_ptr<iCubCartesian> cart_right_;

    double approach_traj_time_;
    double cart_limit_x_lower_;
    double cart_limit_x_upper_;
    double cart_limit_y_lower_;
    double cart_limit_y_upper_;
    double cart_limit_z_lower_;
    double cart_limit_z_upper_;

    bool grasp_use_angular_error_;
    double grasp_translation_error_;
    double grasp_angular_error_;

    double grasp_limit_x_lower_;
    double grasp_limit_x_upper_;
    double grasp_limit_y_lower_;
    double grasp_limit_y_upper_;
    double grasp_limit_z_lower_;
    double grasp_limit_z_upper_;

    double wait_hand_pregrasp_;
    double wait_pregrasp_;
    double wait_reach_;
    double wait_grasp_;
    double wait_lift_;
    double wait_after_lift_;
    double wait_release_;
    double wait_idle_;
    double wait_idle_no_gaze_;

    /**
     * iCub joint controllers.
     */
    std::shared_ptr<iCubMotorsPositions> joints_left_arm_;
    std::shared_ptr<iCubMotorsPositions> joints_right_arm_;
    std::shared_ptr<iCubMotorsPositions> joints_torso_;
    std::shared_ptr<iCubMotorsPositions> joints_left_hand_;
    std::shared_ptr<iCubMotorsPositions> joints_right_hand_;

    Eigen::VectorXd home_torso_joints_;
    Eigen::VectorXd home_arm_joints_;
    Eigen::VectorXd home_hand_joints_;
    Eigen::VectorXd home_torso_joints_vels_;
    Eigen::VectorXd home_arm_joints_vels_;

    Eigen::VectorXd pregrasp_hand_joints_;
    Eigen::VectorXd grasp_hand_joints_left_;
    Eigen::VectorXd grasp_hand_joints_right_;
    Eigen::VectorXd postgrasp_hand_joints_;
    Eigen::VectorXd home_hand_joints_vels_;
    Eigen::VectorXd pregrasp_hand_joints_vels_;
    Eigen::VectorXd grasp_hand_joints_vels_left_;
    Eigen::VectorXd grasp_hand_joints_vels_right_;
    Eigen::VectorXd postgrasp_hand_joints_vels_;

    const std::vector<std::string> home_torso_considered_joints_ =
    {
        "torso_yaw", "torso_roll", "torso_pitch"
    };
    const std::vector<std::string> home_arm_considered_joints_ =
    {
        "arm_shoulder_pitch", "arm_shoulder_roll", "arm_shoulder_yaw",
        "arm_elbow",
        "arm_wrist_prosup", "arm_wrist_pitch", "arm_wrist_yaw"
    };
    const std::vector<std::string> home_hand_considered_joints_ =
    {
        "hand_hand_finger",
        "hand_thumb_oppose", "hand_thumb_proximal", "hand_thumb_distal",
        "hand_index_proximal", "hand_index_distal",
        "hand_middle_proximal", "hand_middle_distal",
        "hand_little"
    };
    const std::vector<std::string> hand_considered_joints_ = home_hand_considered_joints_;

    /**
     * Objects name.
     */
    std::string object_name_;

    /**
     * Objects short name to long name mapping.
     */
    std::unordered_map<std::string, std::string> objects_map_;

    /**
     * Object pose input.
     */
    yarp::os::BufferedPort<yarp::sig::Vector> port_state_;

    Eigen::Transform<double, 3, Eigen::Affine> last_object_pose_;

    Eigen::Transform<double, 3, Eigen::Affine> grasp_object_pose_;

    Eigen::MatrixXd grasp_object_points_;

    Eigen::Vector3d last_object_velocity_;

    Eigen::MatrixXd last_object_points_;

    const bool is_pose_input_buffered_ = false;

    bool is_first_state_ = false;

    /**
     * Object input reception time.
     */
    double alpha_ema_ = 0.2;

    double input_delta_rx_;

    const double input_delta_rx_max_ = 1500;

    std::chrono::steady_clock::time_point last_rx_time_;

    bool is_first_time_ = true;

    /**
     * Object steady state detector.
     */
    bool obj_ss_timer_init_ = false;

    double obj_ss_velocity_thr_;

    double obj_ss_time_thr_;

    std::chrono::steady_clock::time_point obj_ss_start_time_;

    /**
     * Face expression.
     */
    yarp::os::Port port_face_;

    bool use_face_expression_;

    /**
     * RPC ports.
     */
    yarp::os::Port port_rpc_;

    yarp::os::RpcClient port_rpc_segm_;

    yarp::os::RpcClient port_rpc_pose_est_;

    yarp::os::RpcClient port_rpc_trk_;

    /**
     * Parameters.
     */
    int frequency_;

    const double feedback_wait_threshold_ = 40.0;

    /**
     * Module state.
     */
    enum class State { Idle, IdleNoGaze, GoHomeArms, GoHomeGaze, Grasp, Tracking, WaitForFeedback, WaitForHome };

    State state_ = State::Idle;

    std::chrono::steady_clock::time_point time_0_;

    double total_time_;

    /**
     * Grasp state.
     */
    enum class GraspState
    {
        Idle,
        Evaluation, WaitHandPregrasp,
        ArmPregrasp, WaitArmPregrasp,
        ArmReach, WaitArmReach,
        Grasp, WaitGrasp,
        Lift, WaitLift, WaitAfterLift,
        Release, WaitRelease,
        ObjectMoved,
        Cleanup, Done
    };

    GraspState grasp_state_ = GraspState::Idle;

    std::shared_ptr<iCubCartesian> grasp_cart_;

    std::shared_ptr<iCubMotorsPositions> grasp_joints_hand_;

    int backup_grasp_context_;

    int grasp_context_;

    yarp::sig::Vector grasp_target_position_;

    yarp::sig::Vector grasp_target_orientation_;

    yarp::sig::Vector grasp_center_;

    std::string grasp_type_;

    /**
     * Debugging.
     */
    yarp::os::BufferedPort<GraspData> port_grasp_data_;

    void send_grasp_data(const std::string& name, const Eigen::Transform<double, 3, Eigen::Affine>& pose, const std::vector<rankable_candidate>& candidates);

    /**
     * Name for log messages.
     */
    const std::string log_name_ = "roft-samples-handover";
};

#endif /* MODULE_H */
