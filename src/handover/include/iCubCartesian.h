/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBCARTESIAN_H
#define ICUBCARTESIAN_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <unordered_map>


class iCubCartesian
{
public:

    iCubCartesian(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix);

    void close();

    bool check_motion_done();

    yarp::dev::ICartesianControl& controller();

    bool enable_torso(const bool& enable_yaw, const bool& enable_pitch, const bool& enable_roll);

    bool enable_torso_limits(const std::string& torso_part, const double& min, const double& max);

    bool enable_arm_limits(const std::string& arm_part, const double& min, const double& max);

    bool go_to_pose(const yarp::sig::Vector& position, const yarp::sig::Vector& orientation);

    bool go_to_pose_stream(const yarp::sig::Vector& position, const yarp::sig::Vector& orientation);

    bool go_to_position(const yarp::sig::Vector& position);

    bool go_to_position_stream(const yarp::sig::Vector& position);

    bool restore_context();

    bool store_context();

    bool stop();

private:
    yarp::os::Network yarp_;

    std::string laterality_;

    /**
     * Joint map.
     */
    std::unordered_map<std::string, std::size_t> arm_joints_map_ =
    {
        {"shoulder_pitch", 0 + 3},
        {"shoulder_roll",  1 + 3},
        {"shoulder_yaw",   2 + 3},
        {"elbow",          3 + 3},
        {"wrist_prosup",   4 + 3},
        {"wrist_pitch",    5 + 3},
        {"wrist_yaw",      6 + 3}
     };

    /**
     * Driver.
     */
    yarp::dev::PolyDriver driver_cartesian_;

    /**
     * View.
     */
    yarp::dev::ICartesianControl* controller_;

    /**
     * Contexts.
     */
    int initial_context_;

    int current_context_;

    /**
     * Log names to be used in messages printed by the class.
     */

    const std::string log_name_ = "iCubCartesian";
};

#endif /* ICUBCARTESIAN_H */
