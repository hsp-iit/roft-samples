/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <module.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>

#include <thread>

using namespace Eigen;
using namespace Utils;
using namespace cardinal_points_grasp;
using namespace std::literals::chrono_literals;
using namespace yarp::eigen;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using Pose = Eigen::Transform<double, 3, Eigen::Affine>;


bool Module::configure(yarp::os::ResourceFinder& rf)
{
    /* Get parameters. */
    const std::string robot = rf.check("robot", Value("icub")).asString();
    frequency_ = rf.check("frequency", Value(30)).asInt32();
    use_face_expression_ = rf.check("use_face_expression", Value(false)).asBool();

    const Bottle rf_cartesian_control = rf.findGroup("CARTESIAN_CONTROL");
    approach_traj_time_ = rf_cartesian_control.check("approach_traj_time", Value(5.0)).asFloat64();

    bool enable_torso_pitch = rf_cartesian_control.check("torso_pitch", Value(false)).asBool();
    bool enable_torso_roll = rf_cartesian_control.check("torso_roll", Value(false)).asBool();
    bool enable_torso_yaw = rf_cartesian_control.check("torso_yaw", Value(false)).asBool();

    double torso_pitch_max = rf_cartesian_control.check("torso_pitch_max", Value(10.0)).asFloat64();
    double torso_pitch_min = rf_cartesian_control.check("torso_pitch_min", Value(-10.0)).asFloat64();
    double torso_roll_max = rf_cartesian_control.check("torso_roll_max", Value(30.0)).asFloat64();
    double torso_roll_min = rf_cartesian_control.check("torso_roll_min", Value(-30.0)).asFloat64();

    bool enable_wrist_limits = rf_cartesian_control.check("enable_wrist_limits", Value(false)).asBool();
    double wrist_pitch_max = rf_cartesian_control.check("wrist_pitch_max", Value(20.0)).asFloat64();
    double wrist_pitch_min = rf_cartesian_control.check("wrist_pitch_min", Value(-80.0)).asFloat64();

    cart_limit_x_lower_ = rf_cartesian_control.check("limit_x_lower", Value(-0.45)).asFloat64();
    cart_limit_x_upper_ = rf_cartesian_control.check("limit_x_upper", Value(-0.2)).asFloat64();
    cart_limit_y_lower_ = rf_cartesian_control.check("limit_y_lower", Value(-0.2)).asFloat64();
    cart_limit_y_upper_ = rf_cartesian_control.check("limit_y_upper", Value(0.2)).asFloat64();
    cart_limit_z_lower_ = rf_cartesian_control.check("limit_z_lower", Value(0.0)).asFloat64();
    cart_limit_z_upper_ = rf_cartesian_control.check("limit_z_upper", Value(0.3)).asFloat64();

    const Bottle rf_gaze_limits = rf.findGroup("GAZE_LIMITS");
    enable_gaze_limit_x_ = rf_gaze_limits.check("enable_limit_x", Value(true)).asBool();
    enable_gaze_limit_y_ = rf_gaze_limits.check("enable_limit_y", Value(false)).asBool();
    enable_gaze_limit_z_ = rf_gaze_limits.check("enable_limit_z", Value(false)).asBool();
    enable_gaze_lower_limit_x_ = rf_gaze_limits.check("enable_lower_limit_x", Value(true)).asBool();
    gaze_limit_x_ = rf_gaze_limits.check("limit_x", Value(0.9)).asFloat64();
    gaze_limit_y_ = rf_gaze_limits.check("limit_y", Value(0.0)).asFloat64();
    gaze_limit_z_ = rf_gaze_limits.check("limit_z", Value(0.0)).asFloat64();
    gaze_lower_limit_x_ = rf_gaze_limits.check("lower_limit_x", Value(0.3)).asFloat64();

    const Bottle rf_gaze_resp = rf.findGroup("GAZE_RESP");
    double gaze_neck_time_trk = rf_gaze_resp.check("neck_time_trk", Value(3.0)).asFloat64();
    double gaze_eyes_time_trk = rf_gaze_resp.check("eyes_time_trk", Value(3.0)).asFloat64();
    double gaze_neck_time_home = rf_gaze_resp.check("neck_time_home", Value(3.0)).asFloat64();
    double gaze_eyes_time_home = rf_gaze_resp.check("eyes_time_home", Value(3.0)).asFloat64();

    const Bottle rf_grasp_abort = rf.findGroup("GRASP_ABORT");
    grasp_use_angular_error_ = rf_grasp_abort.check("angular", Value(false)).asBool();
    grasp_angular_error_ = rf_grasp_abort.check("angular_error", Value(10.0)).asFloat64();
    grasp_translation_error_ = rf_grasp_abort.check("translation_error", Value(0.05)).asFloat64();

    const Bottle rf_grasp_limits = rf.findGroup("GRASP_LIMITS");
    grasp_limit_x_lower_ = rf_grasp_limits.check("limit_x_lower", Value(-0.4)).asFloat64();
    grasp_limit_x_upper_ = rf_grasp_limits.check("limit_x_upper", Value(-0.2)).asFloat64();
    grasp_limit_y_lower_ = rf_grasp_limits.check("limit_y_lower", Value(-0.2)).asFloat64();
    grasp_limit_y_upper_ = rf_grasp_limits.check("limit_y_upper", Value(0.2)).asFloat64();
    grasp_limit_z_lower_ = rf_grasp_limits.check("limit_z_lower", Value(0.0)).asFloat64();
    grasp_limit_z_upper_ = rf_grasp_limits.check("limit_z_upper", Value(0.3)).asFloat64();

    const Bottle rf_grasp_tweaks = rf.findGroup("GRASP_POSE_TWEAKS");
    grasp_tweak_rot_y_ = rf_grasp_tweaks.check("rot_y", Value(0.0)).asFloat64();

    const Bottle rf_joint_control = rf.findGroup("JOINT_CONTROL");
    bool is_vector;
    yarp::sig::Vector arm_joint_home_configuration;
    std::tie(is_vector, arm_joint_home_configuration) = load_vector_double(rf_joint_control, "home_arm_joints", 7);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::home_arm_joints";
        return false;
    }
    yarp::sig::Vector hand_joint_grasp_configuration_left;
    std::tie(is_vector, hand_joint_grasp_configuration_left) = load_vector_double(rf_joint_control, "left_grasp_joints", 9);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::left_grasp_joints";
        return false;
    }
    yarp::sig::Vector hand_joint_grasp_configuration_right;
    std::tie(is_vector, hand_joint_grasp_configuration_right) = load_vector_double(rf_joint_control, "right_grasp_joints", 9);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::right_grasp_joints";
        return false;
    }
    yarp::sig::Vector hand_joint_home_configuration;
    std::tie(is_vector, hand_joint_home_configuration) = load_vector_double(rf_joint_control, "home_hand_joints", 9);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::home_hand_joints";
        return false;
    }
    yarp::sig::Vector hand_joint_pregrasp_configuration;
    std::tie(is_vector, hand_joint_pregrasp_configuration) = load_vector_double(rf_joint_control, "pregrasp_hand_joints", 9);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::pregrasp_hand_joints";
        return false;
    }
    yarp::sig::Vector hand_joint_grasp_vels_left;
    std::tie(is_vector, hand_joint_grasp_vels_left) = load_vector_double(rf_joint_control, "left_grasp_vels", 9);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::left_grasp_vels";
        return false;
    }
    yarp::sig::Vector hand_joint_grasp_vels_right;
    std::tie(is_vector, hand_joint_grasp_vels_right) = load_vector_double(rf_joint_control, "right_grasp_vels", 9);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::right_grasp_vels";
        return false;
    }

    const Bottle rf_object = rf.findGroup("OBJECT");
    object_name_ = rf_object.check("name", Value("006_mustard_bottle")).asString();

    const Bottle rf_parts = rf.findGroup("PARTS");
    bool enable_part_left = rf_parts.check("left", Value(false)).asBool();
    bool enable_part_right = rf_parts.check("right", Value(false)).asBool();

    const Bottle rf_steady_state_detector = rf.findGroup("STEADY_STATE_DETECTOR");
    obj_ss_time_thr_ = rf_steady_state_detector.check("time_threshold", Value(4.0)).asFloat64();
    obj_ss_velocity_thr_ = rf_steady_state_detector.check("vel_threshold", Value(0.05)).asFloat64();

    const Bottle rf_timings = rf.findGroup("TIMINGS");
    wait_hand_pregrasp_ = rf_timings.check("hand_pregrasp", Value(1.0)).asFloat64();
    wait_pregrasp_ = rf_timings.check("pregrasp", Value(3.0)).asFloat64();
    wait_reach_ = rf_timings.check("reach", Value(3.0)).asFloat64();
    wait_grasp_ = rf_timings.check("grasp", Value(3.0)).asFloat64();
    wait_lift_ = rf_timings.check("lift", Value(3.0)).asFloat64();
    wait_after_lift_ = rf_timings.check("after_lift", Value(3.0)).asFloat64();
    wait_release_ = rf_timings.check("release", Value(3.0)).asFloat64();
    wait_idle_ = rf_timings.check("idle", Value(1.0)).asFloat64();
    wait_idle_no_gaze_ = rf_timings.check("idle_no_gaze", Value(1.0)).asFloat64();

    /* Open RPC port and attach to respond handler. */
    if (!port_rpc_.open("/" + log_name_ + "/rpc:i"))
    {
        yError() << log_name_ + "::configure. Error: cannot open input RPC port.";

        return false;
    }
    if (!(this->yarp().attachAsServer(port_rpc_)))
    {
        yError() << log_name_ + "::configure. Error: cannot attach RPC port to the respond handler.";
        return false;
    }

    /* Open RPC clients. */
    if (!port_rpc_segm_.open("/" + log_name_ + "/segmentation/rpc:o"))
    {
        yError() << log_name_ + "::configure. Error: cannot open output RPC port towards segmentation module.";

        return false;
    }

    if (!port_rpc_pose_est_.open("/" + log_name_ + "/pose/rpc:o"))
    {
        yError() << log_name_ + "::configure. Error: cannot open output RPC port towards pose estimation module.";

        return false;
    }

    if (!port_rpc_trk_.open("/" + log_name_ + "/tracker/rpc:o"))
    {
        yError() << log_name_ + "::configure. Error: cannot open output RPC port towards pose tracker module.";

        return false;
    }

    /* Open port for object state. */
    if (!port_state_.open("/" + log_name_ + "/tracker/state:i"))
    {
        yError() << log_name_ + "::configure. Error: cannot open port for object state.";

        return false;
    }

    /* Open port for face expression. */
    if (use_face_expression_)
    {
        if (!port_face_.open("/" + log_name_ + "/face_expression:o"))
        {
            yError() << log_name_ + "::configure. Error: cannot open port for face expression.";

            return false;
        }
    }

    /* Open port for grasp data debugging. */
    if(!port_grasp_data_.open("/" + log_name_ + "/grasp-data:o"))
    {
        yError() << log_name_ + "::configure. Error: cannot open port for grasp data debugging.";

        return false;
    }

    /* Objects maps. */
    objects_map_["o003"] = "003_cracker_box";
    objects_map_["o004"] = "004_sugar_box";
    objects_map_["o006"] = "006_mustard_bottle";

    /* Object sizes. */
    object_sizes_["003_cracker_box"] = Eigen::Vector3d::Zero();
    object_sizes_.at("003_cracker_box")(0) = 0.04; // it should be 0.0718, pretending it is thinner than real
    object_sizes_.at("003_cracker_box")(1) = 0.1640;
    object_sizes_.at("003_cracker_box")(2) = 0.2134;

    object_sizes_["004_sugar_box"] = Eigen::Vector3d::Zero();
    object_sizes_.at("004_sugar_box")(0) = 0.0451;
    object_sizes_.at("004_sugar_box")(1) = 0.0927;
    object_sizes_.at("004_sugar_box")(2) = 0.1763;

    object_sizes_["006_mustard_bottle"] = Eigen::Vector3d::Zero();
    object_sizes_.at("006_mustard_bottle")(0) = 0.0582;
    object_sizes_.at("006_mustard_bottle")(1) = 0.0960;
    object_sizes_.at("006_mustard_bottle")(2) = 0.1913;

    /* Object offsets. */
    // offset due to non-centered frames in object meshes
    // valid for DOPE meshes only, written in DOPE reference frame
    object_offsets_["003_cracker_box"] = Eigen::Vector3d::Zero();
    object_offsets_["004_sugar_box"] = Eigen::Vector3d::Zero();
    object_offsets_["006_mustard_bottle"] = Eigen::Vector3d::Zero();
    object_offsets_.at("006_mustard_bottle")(1) = 0.005;

    /* Configure iCub gaze controller. */
    gaze_ = std::make_shared<iCubGaze>(robot, log_name_, gaze_neck_time_trk, gaze_eyes_time_trk, gaze_neck_time_home, gaze_eyes_time_home);

    /* Configure iCub joint controllers. */
    joints_torso_ = std::make_shared<iCubMotorsPositions>(robot, "no_laterality", log_name_ + "/torso", /* use_arm = */ false, /* use_torso = */ true);
    if (enable_part_left)
    {
        joints_left_arm_ = std::make_shared<iCubMotorsPositions>(robot, "left", log_name_ + "/left_arm", /* use_arm = */ true, /* use_torso = */ false);
        joints_left_hand_ = std::make_shared<iCubMotorsPositions>(robot, "left", log_name_ + "/left_hand", /* use_arm = */ true, /* use_torso = */ false);
    }
    if (enable_part_right)
    {
        joints_right_arm_ = std::make_shared<iCubMotorsPositions>(robot, "right", log_name_ + "/right_arm", /* use_arm = */ true, /* use_torso = */ false);
        joints_right_hand_ = std::make_shared<iCubMotorsPositions>(robot, "right", log_name_ + "/right_hand", /* use_arm = */ true, /* use_torso = */ false);
    }

    /* Configure iCub Cartesian controllers. */
    if (enable_part_left)
    {
        cart_left_ = std::make_shared<iCubCartesian>(robot, "left", log_name_);
        cart_left_->enable_torso(enable_torso_yaw, enable_torso_pitch, enable_torso_roll);
        cart_left_->enable_torso_limits("pitch", torso_pitch_min, torso_pitch_max);
        cart_left_->enable_torso_limits("roll", torso_roll_min, torso_roll_max);

        if (enable_wrist_limits)
            cart_left_->enable_arm_limits("wrist_pitch", wrist_pitch_min, wrist_pitch_max);
    }
    if (enable_part_right)
    {
        cart_right_ = std::make_shared<iCubCartesian>(robot, "right", log_name_);
        cart_right_->enable_torso(enable_torso_yaw, enable_torso_pitch, enable_torso_roll);
        cart_right_->enable_torso_limits("pitch", torso_pitch_min, torso_pitch_max);
        cart_right_->enable_torso_limits("roll", torso_roll_min, torso_roll_max);

        if (enable_wrist_limits)
            cart_right_->enable_arm_limits("wrist_pitch", wrist_pitch_min, wrist_pitch_max);
    }

    /* Set torso/arm joints for home configuration. */
    home_torso_joints_ = Vector3d::Zero();
    home_arm_joints_ = toEigen(arm_joint_home_configuration);

    /* Set hand joints for home configuration. */
    home_hand_joints_ = toEigen(hand_joint_home_configuration);

    /* Set hand joints for post grasp configuration. */
    pregrasp_hand_joints_ = toEigen(hand_joint_pregrasp_configuration);

    /* Set hand joints for post grasp configuration. */
    postgrasp_hand_joints_ = home_hand_joints_;
    postgrasp_hand_joints_(1) = 80.0;

    /* Set hand joints for grasp configuration. */
    grasp_hand_joints_left_ = toEigen(hand_joint_grasp_configuration_left);
    grasp_hand_joints_right_ = toEigen(hand_joint_grasp_configuration_right);

    /* Set torso/arm joints velocities for home configuration. */
    home_torso_joints_vels_ = VectorXd::Ones(home_torso_joints_.size()) * 10.0;
    home_arm_joints_vels_ = VectorXd::Ones(home_arm_joints_.size()) * 10.0;

    /* Set hand joints velocities for home configuration. */
    home_hand_joints_vels_ = VectorXd::Ones(home_hand_joints_.size()) * 100.0;
    home_hand_joints_vels_(1) = 50.0;

    /* Set hand joints velocities for {pre, ,post} grasp. */
    pregrasp_hand_joints_vels_ = VectorXd::Ones(pregrasp_hand_joints_.size()) * 100.0;
    pregrasp_hand_joints_vels_(1) = 50.0;
    postgrasp_hand_joints_vels_ = pregrasp_hand_joints_vels_;
    grasp_hand_joints_vels_left_ = toEigen(hand_joint_grasp_vels_left);
    grasp_hand_joints_vels_right_ = toEigen(hand_joint_grasp_vels_right);

    return true;
}


bool Module::close()
{
    if (cart_left_)
        cart_left_->stop();

    if (cart_right_)
        cart_right_->stop();

    if (joints_left_arm_)
        joints_left_arm_->stop(home_arm_considered_joints_);

    if (joints_right_arm_)
        joints_right_arm_->stop(home_arm_considered_joints_);

    if (joints_torso_)
        joints_torso_->stop(home_torso_considered_joints_);

    if (gaze_)
        gaze_->stop();

    if (joints_left_hand_)
        joints_left_hand_->stop(home_hand_considered_joints_);

    if (joints_right_hand_)
        joints_right_hand_->stop(home_hand_considered_joints_);;

    if (cart_left_)
        cart_left_->close();

    if (cart_right_)
        cart_right_->close();

    if (joints_left_arm_)
        joints_left_arm_->close();

    if (joints_right_arm_)
        joints_right_arm_->close();

    if (joints_torso_)
        joints_torso_->close();

    if (gaze_)
        gaze_->close();

    if (joints_left_hand_)
        joints_left_hand_->close();

    if (joints_right_hand_)
        joints_right_hand_->close();

    port_rpc_.close();
    port_rpc_segm_.close();
    port_rpc_pose_est_.close();
    port_rpc_trk_.close();
    port_state_.close();
    if (use_face_expression_)
        port_face_.close();

    return true;
}


double Module::getPeriod()
{
    return (1.0 / frequency_);
}


bool Module::updateModule()
{
    /* Elapsed time from last received valid pose. */
    double elapsed = get_rx_elapsed_time();

    /* Get last pose if available. */
    bool valid_pose;
    Pose pose;
    Vector3d velocity;
    MatrixXd points;

    std::tie(valid_pose, pose, velocity, points) = get_object_state();

    /* Update reception time. */
    if (valid_pose)
        set_rx_time();

    /* Default face type. */
    std::string face_type = "hap";

    /* State machine. */
    if (state_ == State::Idle)
    {
        /* Restore gaze idle configuration. */
        gaze_->go_home();

        /* Restore torso and arms idle configuration. */
        go_home_arm();

        /* Restore hand idle configuration. */
        go_home_hand();

        start_counting(wait_idle_);

        yInfo() << "[Idle -> WaitForHome]";
        state_ = State::WaitForHome;
    }
    else if (state_ == State::IdleNoGaze)
    {
        /* Restore torso and arms idle configuration. */
        go_home_arm();

        /* Restore hand idle configuration. */
        go_home_hand();

        start_counting(wait_idle_no_gaze_);

        yInfo() << "[IdleNoGaze -> WaitForHome]";
        state_ = State::WaitForHome;
    }
    else if (state_ == State::GoHomeGaze)
    {
        /* Restore idle gaze configuration. */
        gaze_->go_home();

        yInfo() << "[GoHomeGaze -> WaitForFeedback]";
        state_ = State::WaitForFeedback;
    }
    else if (state_ == State::GoHomeArms)
    {
        /* Restore torso and arms idle configuration. */
        go_home_arm();

        /* Restore hand idle configuration. */
        go_home_hand();

        yInfo() << "[GoHomeArms -> WaitForFeedback]";
        state_ = State::WaitForFeedback;
    }
    else if (state_ == State::Grasp)
    {
        if (grasp_state_ == GraspState::Done)
        {
            yInfo() << "[Grasp][Done -> Idle]";

            grasp_state_ = GraspState::Idle;

            yInfo() << "[Grasp -> Idle]";

            state_ = State::IdleNoGaze;
        }
        else if (grasp_state_ == GraspState::ObjectMoved)
        {

            yInfo() << "[Grasp][? -> ObjectMoved]";

            grasp_state_ = GraspState::Idle;

            yInfo() << "[Grasp -> Idle]";

            state_ = State::IdleNoGaze;
        }

        if(!execute_grasp(grasp_object_pose_, grasp_object_points_, pose, valid_pose))
        {
            yInfo() << "[Grasp -> WaitForFeedback]";

            state_ = State::WaitForFeedback;
        }
    }
    else if (state_ == State::Tracking)
    {
        if (elapsed > feedback_wait_threshold_)
        {
            yInfo() << "[Tracking -> GoHomeGaze]";
            state_ = State::GoHomeGaze;
        }
        else
        {
            /* Track object with gaze. */
            if (valid_pose && is_pose_gaze_safe(pose))
            {
                yarp::sig::Vector target(3);
                target(0) = pose.translation()(0);
                target(1) = pose.translation()(1);
                target(2) = pose.translation()(2);
                gaze_->look_at_stream(target);
            }

            if (is_object_steady(velocity))
            {
                if (is_pose_grasp_safe(pose))
                {
                    grasp_object_pose_ = pose;
                    grasp_object_points_ = points;
                    grasp_state_ = GraspState::Evaluation;

                    yInfo() << "[Tracking -> Grasp]";
                    state_ = State::Grasp;
                }
                else
                {
                    /* Update face expression. */
                    face_type = "sad";
                }
            }
        }
    }
    else if (state_ == State::WaitForFeedback)
    {
        if (elapsed < feedback_wait_threshold_)
        {
            yInfo() << "[WaitForFeedback -> Tracking]";
            state_ = State::Tracking;
        }
    }
    else if (state_ == State::WaitForHome)
    {
        if (is_elapsed_from_start_counting())
        {
            yInfo() << "[WaitForHome -> WaitForFeedback]";
            state_ = State::WaitForFeedback;
        }
    }

    if (use_face_expression_)
        set_face_expression(face_type);

    return true;
}


std::string Module::select_object(const std::string& object_name)
{
    bool found = false;
    for (const auto& pair : objects_map_)
    {
        if (object_name == pair.first)
        {
            found = true;
            break;
        }
    }
    if (!found)
    {
        std::string reply = "Available objects are ";
        std::size_t counter = 0;
        for (const auto& pair : objects_map_)
        {
            reply += pair.first + " -> " + pair.second;
            if (counter != (objects_map_.size() - 1))
                reply += ", ";

            counter++;
        }

        return reply;
    }

    object_name_ = objects_map_.at(object_name);

    yInfo() << "Selected new object" << object_name_;

    send_rpc(port_rpc_segm_, {"select_object", objects_map_.at(object_name)});
    std::this_thread::sleep_for(100ms);

    send_rpc(port_rpc_pose_est_, {"select_object", objects_map_.at(object_name)});
    std::this_thread::sleep_for(100ms);

    send_rpc(port_rpc_trk_, {"select_object", objects_map_.at(object_name)});
    std::this_thread::sleep_for(100ms);
    send_rpc(port_rpc_trk_, {"stop"});
    std::this_thread::sleep_for(100ms);
    send_rpc(port_rpc_trk_, {"reset"});
    std::this_thread::sleep_for(100ms);
    send_rpc(port_rpc_trk_, {"start"});

    return "Command accepted";
}


std::tuple<bool, Eigen::Transform<double, 3, Eigen::Affine>, Vector3d, MatrixXd> Module::get_object_state()
{
    yarp::sig::Vector* state_yarp = port_state_.read(is_pose_input_buffered_);

    auto yarp_to_transform = [](const yarp::sig::Vector& vector) -> Pose
    {
        Pose pose;
        pose = Translation<double, 3>(vector[0], vector[1], vector[2]);
        pose.rotate(AngleAxisd(vector[6], Vector3d(vector[3], vector[4], vector[5])));

        return pose;
    };

    auto yarp_to_velocity = [](const yarp::sig::Vector& vector) -> Vector3d
    {
        Vector3d velocity(vector[7], vector[8], vector[9]);

        return velocity;
    };

    auto yarp_to_points = [](const yarp::sig::Vector& vector) -> MatrixXd
    {
        Eigen::VectorXd bbox_points_data = toEigen(vector).segment<24>(7 + 6);
        MatrixXd points(3, 8);
        for (std::size_t i = 0; i < 8; i++)
            points.col(i) = bbox_points_data.segment<3>(3 * i);

        return points;
    };

    if (state_yarp == nullptr)
    {
        if(is_pose_input_buffered_ && is_first_state_)
            return std::make_tuple(true, last_object_pose_, last_object_velocity_, last_object_points_);
        else
            return std::make_tuple(false, Pose::Identity(), Vector3d::Zero(), MatrixXd());
    }
    else
    {
        last_object_pose_ = yarp_to_transform(*state_yarp);
        last_object_velocity_ = yarp_to_velocity(*state_yarp);

        /* FIXME: we should use a more general object here that handles the object pose, velocity and bounding box points. */
        /* Check if object bounding box points have been transmitted. */
        if(state_yarp->size() == (7 + 6 + 8 * 3))
            last_object_points_ = yarp_to_points(*state_yarp);

        is_first_state_ = true;

        return std::make_tuple(true, last_object_pose_, last_object_velocity_, last_object_points_);
    }
}


bool Module::is_pose_gaze_safe(const Pose& pose)
{
    const double& x = pose.translation()(0);
    const double& y = pose.translation()(1);
    const double& z = pose.translation()(2);

    /* Do not look backward. */
    if (x > 0.0)
        return false;

    if (enable_gaze_limit_x_ && (-x > gaze_limit_x_))
        return false;

    if (enable_gaze_lower_limit_x_ && (-x < gaze_lower_limit_x_))
        return false;

    if (enable_gaze_limit_y_ && (abs(y) > gaze_limit_y_))
        return false;

    if (enable_gaze_limit_z_ && (abs(z) > gaze_limit_z_))
        return false;

    return true;
}


bool Module::is_pose_grasp_safe(const Pose& pose)
{
    const double& x = pose.translation()(0);
    const double& y = pose.translation()(1);
    const double& z = pose.translation()(2);

    if (x > grasp_limit_x_upper_)
    {
        // yError() << log_name_ << "::is_pose_grasp_safe(). Error on x:" << x << ">" << grasp_limit_x_upper_ ;
        return false;
    }

    if (x < grasp_limit_x_lower_)
    {
        // yError() << log_name_ << "::is_pose_grasp_safe(). Error on x:" << x << "<" << grasp_limit_x_lower_ ;
        return false;
    }

    if (y > grasp_limit_y_upper_)
    {
        // yError() << log_name_ << "::is_pose_grasp_safe(). Error on y:" << y << ">" << grasp_limit_y_upper_ ;
        return false;
    }

    if (y < grasp_limit_y_lower_)
    {
        // yError() << log_name_ << "::is_pose_grasp_safe(). Error on y:" << y << "<" << grasp_limit_y_lower_ ;
        return false;
    }

    if (z > grasp_limit_z_upper_)
    {
        // yError() << log_name_ << "::is_pose_grasp_safe(). Error on z:" << z << "<" << grasp_limit_z_upper_ ;
        return false;
    }

    if (z < grasp_limit_z_lower_)
    {
        // yError() << log_name_ << "::is_pose_grasp_safe(). Error on z:" << z << "<" << grasp_limit_z_lower_ ;
        return false;
    }

    return true;
}


bool Module::is_position_cart_safe(const yarp::sig::Vector& position)
{
    const double& x = position[0];
    const double& y = position[1];
    const double& z = position[2];

    if (x > cart_limit_x_upper_)
    {
        yError() << log_name_ << "::is_position_cart_safe(). Error on x:" << x << ">" << cart_limit_x_upper_ ;
        return false;
    }

    if (x < cart_limit_x_lower_)
    {
        yError() << log_name_ << "::is_position_cart_safe(). Error on x:" << x << "<" << cart_limit_x_lower_ ;
        return false;
    }

    if (y > cart_limit_y_upper_)
    {
        yError() << log_name_ << "::is_position_cart_safe(). Error on y:" << y << ">" << cart_limit_y_upper_ ;
        return false;
    }

    if (y < cart_limit_y_lower_)
    {
        yError() << log_name_ << "::is_position_cart_safe(). Error on y:" << y << "<" << cart_limit_y_lower_ ;
        return false;
    }

    if (z > cart_limit_z_upper_)
    {
        yError() << log_name_ << "::is_position_cart_safe(). Error on z:" << z << "<" << cart_limit_z_upper_ ;
        return false;
    }

    if (z < cart_limit_z_lower_)
    {
        yError() << log_name_ << "::is_position_cart_safe(). Error on z:" << z << "<" << cart_limit_z_lower_ ;
        return false;
    }

    return true;
}


bool Module::is_object_steady(const Eigen::Vector3d& velocity)
{
    /* Check if object is steady state. */
    if (velocity.norm() < obj_ss_velocity_thr_)
    {
        if (obj_ss_timer_init_)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - obj_ss_start_time_).count() / 1000.0;

            if (elapsed > obj_ss_time_thr_)
            {
                obj_ss_timer_init_ = false;
                return true;
            }
        }
        else
        {
            obj_ss_start_time_ = std::chrono::steady_clock::now();

            obj_ss_timer_init_ = true;
        }
    }
    else
        obj_ss_timer_init_ = false;

    return false;
}


double Module::get_rx_elapsed_time()
{
    auto now = std::chrono::steady_clock::now();

    if (is_first_time_)
    {
        input_delta_rx_ = 1000.0;
        last_rx_time_ = now;

        is_first_time_ = false;
    }
    else
    {
        double delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_time_).count();
        input_delta_rx_ = alpha_ema_ * delta + (1 - alpha_ema_) * input_delta_rx_;
    }

    if (input_delta_rx_ > input_delta_rx_max_)
        input_delta_rx_ = input_delta_rx_max_;

    return input_delta_rx_;
}


void Module::set_rx_time()
{
    last_rx_time_ = std::chrono::steady_clock::now();
}


void Module::start_counting(const double& total_time)
{
    time_0_ = std::chrono::steady_clock::now();

    total_time_ = total_time;
}


bool Module::is_elapsed_from_start_counting()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_0_).count() / 1000.0 > total_time_;
}


bool Module::send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages)
{
    Bottle cmd, reply;

    for (const std::string& message : messages)
        cmd.addString(message);

    return port.write(cmd, reply);
}


void Module::go_home_arm()
{
    /* Switch to POSITION control. */
    if (joints_torso_)
        joints_torso_->set_mode(home_torso_considered_joints_);
    if (joints_left_arm_)
        joints_left_arm_->set_mode(home_arm_considered_joints_);
    if (joints_right_arm_)
        joints_right_arm_->set_mode(home_arm_considered_joints_);

    /* Send setpoint. */
    if (joints_torso_)
        joints_torso_->set_positions(home_torso_joints_, home_torso_joints_vels_, home_torso_considered_joints_);
    if (joints_left_arm_)
        joints_left_arm_->set_positions(home_arm_joints_, home_arm_joints_vels_, home_arm_considered_joints_);
    if (joints_right_arm_)
        joints_right_arm_->set_positions(home_arm_joints_, home_arm_joints_vels_, home_arm_considered_joints_);
}

void Module::go_home_hand()
{
    /* Switch to POSITION control. */
    if (joints_left_hand_)
        joints_left_hand_->set_mode(home_hand_considered_joints_);
    if (joints_right_hand_)
        joints_right_hand_->set_mode(home_hand_considered_joints_);

    /* Send setpoint. */
    if (joints_left_hand_)
        joints_left_hand_->set_positions(home_hand_joints_, home_hand_joints_vels_, home_hand_considered_joints_);
    if (joints_right_hand_)
        joints_right_hand_->set_positions(home_hand_joints_, home_hand_joints_vels_, home_hand_considered_joints_);
}


bool Module::execute_grasp(const Pose& pose, const MatrixXd& object_points, const Pose& feedback, const bool& valid_feedback)
{

    bool interruptible = !
    (
        (grasp_state_ == GraspState::Idle) ||
        (grasp_state_ == GraspState::Evaluation) ||
        (grasp_state_ == GraspState::Grasp) ||
        (grasp_state_ == GraspState::WaitGrasp) ||
        (grasp_state_ == GraspState::Lift) ||
        (grasp_state_ == GraspState::WaitLift) ||
        (grasp_state_ == GraspState::WaitAfterLift) ||
        (grasp_state_ == GraspState::Release) ||
        (grasp_state_ == GraspState::WaitRelease) ||
        (grasp_state_ == GraspState::Cleanup)
    );

    if (valid_feedback && interruptible)
    {
        bool abort = false;

        Transform<double, 3, Affine> error = pose * feedback.inverse();

        if (error.translation().norm() > grasp_translation_error_)
            abort = true;

        if (grasp_use_angular_error_ && (AngleAxisd(error.rotation()).angle() > (grasp_angular_error_ * M_PI / 180.0)))
            abort = true;

        if (abort)
        {
            gaze_->stop();
            gaze_->controller().setTrackingMode(false);

            grasp_cart_->stop();
            grasp_cart_->controller().restoreContext(backup_grasp_context_);

            grasp_cart_ = nullptr;
            grasp_joints_hand_ = nullptr;

            grasp_state_ = GraspState::ObjectMoved;

            return true;
        }
    }

    if (grasp_state_ == GraspState::Evaluation)
    {
        /* Cardinal points grasp strategy. */

        if ((cart_left_ == nullptr) && (cart_right_ == nullptr))
            return false;

        std::vector<rankable_candidate> candidates;
        std::vector<rankable_candidate> candidates_l;
        std::vector<rankable_candidate> candidates_r;
        int context_l;
        int context_r;
        Vector3d object_sizes;
        Vector3d object_offsets;
        Eigen::Matrix3d rotation_offset;
        object_offsets.setZero();
        rotation_offset.setIdentity();

        /* FIXME: These transformations might be unified at some point. */
        if (object_points.size() != 0)
        {
            /* If the object is unknown we rely on the oriented bounding box points to evaluate object properties. */

            /* Check that the points have been received. */
            if (object_points.size() == 0)
            {
                yError() << log_name_ << "::execute_grasp(). The object is unknown but the vertices of the oriented bounding box have not been received. Aborting.";
                return false;
            }

            /* Evaluate object sizes with x, y and z axis in descending order. */
            VectorXd tmp_object_sizes = evaluate_object_sizes(pose, object_points);

            /* Re-order the sizes so that they adhere to the common standards in the cardinal points grasp code. */
            object_sizes(0) = tmp_object_sizes(2);
            object_sizes(1) = tmp_object_sizes(1);
            object_sizes(2) = tmp_object_sizes(0);

            /* Make sure the rotation offset transforms the reference frame such that the z axis points upward. */
            Eigen::Vector3d x_axis = pose.rotation().col(0);
            if (abs(std::acos(x_axis.dot(Eigen::Vector3d::UnitZ()))) > M_PI / 2.0)
                rotation_offset = rotation_offset * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
            rotation_offset = rotation_offset * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
        }
        else
        {
            /* Here we are considering precomputed object properties assuming NVidia NVDU DOPE reference frames. */
            object_sizes = object_sizes_.at(object_name_);
            object_offsets = object_offsets_.at(object_name_);

            /* Make sure the rotation offset transforms the reference frame such that the z axis points upward. */
            Eigen::Vector3d y_axis = pose.rotation().col(1);
            if (abs(std::acos(y_axis.dot(Eigen::Vector3d::UnitZ()))) < M_PI / 2.0)
                rotation_offset = rotation_offset * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
            Eigen::Matrix3d rot_offset_0 = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
            Eigen::Matrix3d rot_offset_1 = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            rotation_offset = rotation_offset * rot_offset_0 * rot_offset_1;
        }

        if (cart_left_)
        {
            auto grasper = std::make_unique<CardinalPointsGrasp>("left", pregrasp_hand_joints_);
            grasper->setObjectSizes(object_sizes);
            grasper->setObjectOffsets(object_offsets);
            grasper->setReferenceFrameOffset(rotation_offset);
            std::tie(candidates_l, context_l) = grasper->getCandidates(pose, &(cart_left_->controller()));
        }

        if (cart_right_)
        {
            auto grasper = std::make_unique<CardinalPointsGrasp>("right", pregrasp_hand_joints_);
            grasper->setObjectSizes(object_sizes);
            grasper->setObjectOffsets(object_offsets);
            grasper->setReferenceFrameOffset(rotation_offset);
            std::tie(candidates_r, context_r) = grasper->getCandidates(pose, &(cart_right_->controller()));
        }

        if (cart_right_ && (cart_left_ == nullptr))
            candidates = candidates_r;
        else if(cart_left_ && (cart_right_ == nullptr))
            candidates = candidates_l;
        else
        {
            candidates = candidates_r;
            candidates.insert(candidates.end(), candidates_l.begin(), candidates_l.end());
            std::sort(candidates.begin(), candidates.end(), CardinalPointsGrasp::compareCandidates);
        }

        if (candidates.empty())
        {
            yError() << log_name_ << "::execute_grasp(). No valid grasping candidates.";
            return false;
        }

        send_grasp_data(object_name_, pose, candidates);

        const auto& best = candidates[0];
        grasp_type_ = std::get<0>(best);
        const auto& T = std::get<2>(best);
        grasp_center_ = std::get<3>(best);
        grasp_target_position_ = T.getCol(3).subVector(0, 2);

        if (abs(grasp_tweak_rot_y_) > 0.0)
        {
            double angle = grasp_tweak_rot_y_ * M_PI / 180.0;
            if (grasp_type_ == "right")
                angle *= -1;

            Matrix3d orientation = toEigen(T.submatrix(0, 2, 0, 2)) * AngleAxisd(angle, Vector3d::UnitY()).toRotationMatrix();
            AngleAxisd angle_axis(orientation);
            VectorXd axis_angle(4);
            axis_angle.head(3) = angle_axis.axis();
            axis_angle(3) = angle_axis.angle();

            grasp_target_orientation_.resize(4);
            toEigen(grasp_target_orientation_) = axis_angle;
        }
        else
            grasp_target_orientation_ = dcm2axis(T);

        yInfo() << log_name_ + "::execute_grasp(). Found grasp pose using" << grasp_type_ << "arm at" << grasp_target_position_.toString();

        /* Arm selection. */

        int context;
        std::shared_ptr<iCubCartesian> cart;
        std::shared_ptr<iCubMotorsPositions> joints_hand;
        if (grasp_type_ == "left")
        {
            grasp_context_ = context_l;
            grasp_cart_ = cart_left_;
            grasp_joints_hand_ = joints_left_hand_;
        }
        else
        {
            grasp_context_ = context_r;
            grasp_cart_ = cart_right_;
            grasp_joints_hand_ = joints_right_hand_;
        }

        if ((grasp_cart_ == nullptr) || (grasp_joints_hand_ == nullptr))
        {
            yError() << log_name_ << "::execute_grasp. Unexpected state. Either the cartesian or joints driver pointers are nullptr";
            return false;
        }

        /* Keep gazing at the object. */
        gaze_->controller().setTrackingMode(true);
        yarp::sig::Vector target(3);
        target(0) = pose.translation()(0);
        target(1) = pose.translation()(1);
        target(2) = pose.translation()(2);
        gaze_->look_at_stream(target);

        /* Cartesian configuration .*/
        grasp_cart_->controller().stopControl();
        grasp_cart_->controller().storeContext(&backup_grasp_context_);
        grasp_cart_->controller().restoreContext(grasp_context_);
        grasp_cart_->controller().setInTargetTol(.001);
        grasp_cart_->controller().setTrajTime(approach_traj_time_);

        /* Hand pregrasp configuration .*/
        grasp_joints_hand_->set_positions(pregrasp_hand_joints_, pregrasp_hand_joints_vels_, hand_considered_joints_);

        yInfo() << "[Grasp][Evaluate -> WaitHandPregrasp]";

        grasp_state_ = GraspState::WaitHandPregrasp;
        start_counting(wait_hand_pregrasp_);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitHandPregrasp)
    {
        if (is_elapsed_from_start_counting())
        {
            yInfo() << log_name_ + "::execute_grasp(). Hand ready in pregrasp configuration";

            yInfo() << "[Grasp][Evaluate -> ArmPregrasp]";

            grasp_state_ = GraspState::ArmPregrasp;

            return true;
        }

        return true;
    }
    else if (grasp_state_ == GraspState::ArmPregrasp)
    {

        /* Arm pregrasp configuration. */
        const auto dir = grasp_target_position_ - grasp_center_;
        const auto target = grasp_target_position_ + 0.05 * dir / norm(dir);

        if (!is_position_cart_safe(target))
        {
            yError() << "[Grasp][ArmPregrasp] *********** The commanded position is not safe ***********";

            yInfo() << "[Grasp][ArmPregrasp -> Cleanup]";

            grasp_state_ = GraspState::Cleanup;

            return true;
        }

        grasp_cart_->go_to_pose(target, grasp_target_orientation_);

        yInfo() << "[Grasp][ArmPregrasp -> WaitArmPregrasp]";

        grasp_state_ = GraspState::WaitArmPregrasp;
        start_counting(wait_pregrasp_);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitArmPregrasp)
    {
        if (is_elapsed_from_start_counting())
        {
            // if(grasp_cart_->check_motion_done())
            // {
                yInfo() << log_name_ + "::execute_grasp(). " + grasp_type_ << "arm ready in pregrasp configuration";

                yInfo() << "[Grasp][WaitArmPregrasp -> ArmReach]";

                grasp_state_ = GraspState::ArmReach;
            // }
            // else
            //     start_counting(0.5);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::ArmReach)
    {
        /* Arm final configuration. */
        const auto dir = grasp_center_ - grasp_target_position_;
        const auto target = grasp_target_position_ + 0.015 * dir / norm(dir);

        if (!is_position_cart_safe(target))
        {
            yError() << "[Grasp][ArmReach] *********** The commanded position is not safe ***********";

            yInfo() << "[Grasp][ArmReach -> Cleanup]";

            grasp_state_ = GraspState::Cleanup;

            return true;
        }

        grasp_cart_->go_to_pose(target, grasp_target_orientation_);

        yInfo() << "[Grasp][ArmReach -> WaitArmReach]";

        grasp_state_ = GraspState::WaitArmReach;
        start_counting(wait_reach_);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitArmReach)
    {
        if (is_elapsed_from_start_counting())
        {
            // if(grasp_cart_->check_motion_done())
            // {
                yInfo() << log_name_ + "::execute_grasp(). " + grasp_type_ << "arm ready in grasping configuration";

                yInfo() << "[Grasp][WaitArmReach -> Grasp]";

                grasp_state_ = GraspState::Grasp;
            // }
            // else
            //     start_counting(0.5);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Grasp)
    {
        /* Hand grasp configuration .*/

        VectorXd grasp_hand_joints = (grasp_type_ == "left") ? grasp_hand_joints_left_ : grasp_hand_joints_right_;
        VectorXd grasp_hand_joints_vels = (grasp_type_ == "left") ? grasp_hand_joints_vels_left_ : grasp_hand_joints_vels_right_;
        grasp_joints_hand_->set_positions(grasp_hand_joints, grasp_hand_joints_vels, hand_considered_joints_);

        yInfo() << "[Grasp][Grasp -> WaitGrasp]";

        grasp_state_ = GraspState::WaitGrasp;
        start_counting(wait_grasp_);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitGrasp)
    {
        if (is_elapsed_from_start_counting())
        {
            if (grasp_joints_hand_->check_motion_done(hand_considered_joints_))
            {
                yInfo() << log_name_ + "::execute_grasp()." << grasp_type_ << "hand ready in grasping configuration";

                yInfo() << "[Grasp][WaitGrasp -> Lift]";

                grasp_state_ = GraspState::Lift;

                return true;
            }
            else
                start_counting(1.0);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Lift)
    {
        /* Object lifting .*/
        const auto target = grasp_target_position_ + yarp::sig::Vector{0.0, 0.0, 0.1};

        /* Keep gazing at the object. */
        yarp::sig::Vector gaze_target(3);
        gaze_target(0) = pose.translation()(0);
        gaze_target(1) = pose.translation()(1);
        gaze_target(2) = pose.translation()(2) + 0.1;
        gaze_->look_at_stream(gaze_target);

        if (!is_position_cart_safe(target))
        {
            yError() << "[Grasp][Lift] *********** The commanded position is not safe ***********";

            yInfo() << "[Grasp][Lift -> Cleanup]";

            grasp_state_ = GraspState::Cleanup;

            return true;
        }

        grasp_cart_->go_to_pose(target, grasp_target_orientation_);

        yInfo() << "[Grasp][Lift -> WaitLift]";

        grasp_state_ = GraspState::WaitLift;
        start_counting(wait_lift_);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitLift)
    {
        if (is_elapsed_from_start_counting())
        {
            // if(grasp_cart_->check_motion_done())
            // {
                yInfo() << log_name_ + "::execute_grasp(). Lifted completed.";

                yInfo() << "[Grasp][WaitLift -> WaitAfterLift]";

                grasp_state_ = GraspState::WaitAfterLift;

                start_counting(wait_after_lift_);
            // }
            // else
            //     start_counting(0.5);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::WaitAfterLift)
    {
        if (is_elapsed_from_start_counting())
        {
            yInfo() << log_name_ + "::execute_grasp(). Wait after lift completed.";

            yInfo() << "[Grasp][WaitAfterLift -> Release]";

            grasp_state_ = GraspState::Release;
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Release)
    {
        /* Object release .*/
        grasp_joints_hand_->set_positions(postgrasp_hand_joints_, postgrasp_hand_joints_vels_, hand_considered_joints_);
        yInfo() << "[Grasp][Release -> WaitRelease]";

        grasp_state_ = GraspState::WaitRelease;
        start_counting(wait_release_);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitRelease)
    {
        if (is_elapsed_from_start_counting())
        {
            if (grasp_joints_hand_->check_motion_done(hand_considered_joints_))
            {
                yInfo() << log_name_ + "::execute_grasp(). Release done.";

                yInfo() << "[Grasp][WaitRelease -> Cleanup]";

                grasp_state_ = GraspState::Cleanup;

                return true;
            }
            else
                start_counting(1.0);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Cleanup)
    {
        yInfo() << log_name_ + "::execute_grasp(). Cleaning up.";

        gaze_->stop();
        gaze_->controller().setTrackingMode(false);

        grasp_cart_->stop();
        grasp_cart_->controller().restoreContext(backup_grasp_context_);

        grasp_cart_ = nullptr;
        grasp_joints_hand_ = nullptr;

        yInfo() << "[Grasp][Cleanup -> Done]";

        grasp_state_ = GraspState::Done;

        return true;
    }

    return true;
}


Vector3d Module::evaluate_object_sizes(const Eigen::Transform<double, 3, Eigen::Affine>& pose, const Eigen::MatrixXd& points)
{
    MatrixXd local_points = pose.rotation().transpose() * (points.colwise() + (-pose.translation()));
    std::unordered_map<std::string, VectorXd> mapping;

    for (std::size_t i = 0; i < local_points.cols(); i++)
    {
        const double& x = local_points.col(i)(0);
        const double& y = local_points.col(i)(1);
        const double& z = local_points.col(i)(2);

        if (x > 0 && y > 0 && z > 0)
            mapping["top_right_front"] = local_points.col(i);
        else if (x > 0 && y > 0 && z < 0)
            mapping["top_right_back"] = local_points.col(i);
        else if (x > 0 && y < 0 && z > 0)
            mapping["top_left_front"] = local_points.col(i);
        else if (x > 0 && y < 0 && z < 0)
            mapping["top_left_back"] = local_points.col(i);
        else if (x < 0 && y > 0 && z > 0)
            mapping["bottom_right_front"] = local_points.col(i);
        else if (x < 0 && y > 0 && z < 0)
            mapping["bottom_right_back"] = local_points.col(i);
        else if (x < 0 && y < 0 && z > 0)
            mapping["bottom_left_front"] = local_points.col(i);
        else if (x < 0 && y < 0 && z < 0)
            mapping["bottom_left_back"] = local_points.col(i);
    }

    VectorXd sizes(3);
    sizes[0] = (mapping["top_right_front"] - mapping["bottom_right_front"]).norm();
    sizes[1] = (mapping["top_right_front"] - mapping["top_left_front"]).norm();
    sizes[2] = (mapping["top_right_front"] - mapping["top_right_back"]).norm();

    return sizes;
}


void Module::set_face_expression(const std::string& type)
{
    Bottle in, out;

    out.addVocab32(Vocab32::encode("set"));
    out.addVocab32(Vocab32::encode("mou"));
    out.addVocab32(Vocab32::encode(type));
    // port_face_.write(out,in);
    port_face_.write(out);

    out.clear();

    out.addVocab32(Vocab32::encode("set"));
    out.addVocab32(Vocab32::encode("leb"));
    out.addVocab32(Vocab32::encode(type));
    // port_face_.write(out,in);
    port_face_.write(out);

    out.clear();

    out.addVocab32(Vocab32::encode("set"));
    out.addVocab32(Vocab32::encode("reb"));
    out.addVocab32(Vocab32::encode(type));
    port_face_.write(out);
    // port_face_.write(out,in);
}


void Module::send_grasp_data
(
    const std::string& name,
    const Eigen::Transform<double, 3, Eigen::Affine>& pose,
    const std::vector<rankable_candidate>& candidates
)
{
    GraspData& grasp_data = port_grasp_data_.prepare();
    grasp_data.candidates.clear();

    grasp_data.object_name = name;

    for (const auto& candidate : candidates)
        grasp_data.candidates.push_back
        (
            RankableCandidate
            (
                std::get<0>(candidate),
                std::get<1>(candidate),
                std::get<2>(candidate),
                std::get<3>(candidate)
            )
        );

    grasp_data.object_pose.resize(4, 4);
    toEigen(grasp_data.object_pose) = pose.matrix();

    port_grasp_data_.write();
}
