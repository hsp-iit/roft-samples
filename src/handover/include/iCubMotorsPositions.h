/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBMOTORSPOSITIONS_H
#define ICUBMOTORSPOSITIONS_H

#include <Eigen/Dense>

#include <iCubMotors.h>

#include <yarp/dev/IPositionControl.h>

#include <unordered_map>
#include <vector>


class iCubMotorsPositions : public iCubMotors
{
public:

    iCubMotorsPositions(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_arm, const bool& use_torso);

    void close();

    bool set_mode(const std::vector<std::string>& considered_joints = std::vector<std::string>());

    bool set_positions(const Eigen::VectorXd& positions, const Eigen::VectorXd& reference_velocities, const std::vector<std::string>& considered_joints);

    bool check_motion_done(const std::vector<std::string>& considered_joints);

private:
    /**
     * View.
     */
    std::unordered_map<std::string, yarp::dev::IPositionControl*> controllers_;
};

#endif /* ICUBMOTORSPOSITIONS_H */
