/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubMotorsPositions.h>

#include <yarp/os/LogStream.h>

using namespace Eigen;


iCubMotorsPositions::iCubMotorsPositions
(
    const std::string& robot_name,
    const std::string& laterality,
    const std::string& port_prefix,
    const bool& use_arm,
    const bool& use_torso
) :
    iCubMotors(robot_name, laterality, port_prefix, "position", use_arm, use_torso)
{
    /* Position control configuration (arm). */
    if (use_arm)
    {
        /* Try to get the view. */
        if (!(driver("arm").view(controllers_["arm"]) && (controllers_.at("arm") != nullptr)))
        {
            throw(std::runtime_error(log_name() + "::ctor. Error: cannot open the " + laterality + " arm controller (view)."));
        }
    }

    /* Position control configuration (torso). */
    if (use_torso)
    {
        /* Try to get the view. */
        if (!(driver("torso").view(controllers_["torso"]) && (controllers_.at("torso") != nullptr)))
        {
            throw(std::runtime_error(log_name() + "::ctor. Error: cannot open the " + laterality + " arm controller (view)."));
        }
    }
}


void iCubMotorsPositions::close()
{
    iCubMotors::close();
}


bool iCubMotorsPositions::set_mode(const std::vector<std::string>& considered_joints)
{
    return iCubMotors::set_mode(VOCAB_CM_POSITION, considered_joints);
}


bool iCubMotorsPositions::set_positions(const VectorXd& positions, const VectorXd& reference_velocities, const std::vector<std::string>& considered_joints)
{
    std::unordered_map<std::string, std::vector<int>> joints;
    std::unordered_map<std::string, std::vector<double>> parts_positions;
    std::unordered_map<std::string, std::vector<double>> parts_reference_velocities;

    if (positions.size() != reference_velocities.size())
    {
        yError() << log_name() << "::set_positions(). Error: expected positions.size() == reference_velocities.size(), instead they are "
                 << positions.size() << ", " << reference_velocities.size() << " respectively";

        return false;
    }

    if (positions.size() != considered_joints.size())
    {
        yError() << log_name() << "::set_positions(). Error: expected positions.size() == considered_joints.size(), instead they are "
                 << positions.size() << ", " << considered_joints.size() << " respectively";

        return false;
    }

    if (reference_velocities.size() != considered_joints.size())
    {
        yError() << log_name() << "::set_positions(). Error: expected reference_velocities.size() == considered_joints.size(), instead they are "
                 << reference_velocities.size() << ", " << considered_joints.size() << " respectively";

        return false;
    }

    for (const auto& part : enabled_parts_)
    {
        joints[part] = std::vector<int>(0);
        parts_positions[part] = std::vector<double>(0);
        parts_reference_velocities[part] = std::vector<double>(0);
    }

    for (std::size_t i = 0; i < considered_joints.size(); i++)
    {
        const std::string joint_name = considered_joints.at(i);
        const auto& map = name_to_index_.at(joint_name);
        joints.at(map.first).push_back(map.second);
        parts_positions.at(map.first).push_back(positions(i));

        double reference_velocity = reference_velocities(i);
        bool limit_velocity = false;
        for (const std::string& prefix : {"arm", "torso"})
            limit_velocity |= (joint_name.find(prefix) != std::string::npos);
        if (limit_velocity && reference_velocity > 10.0)
        {
            yWarning() << log_name() << "::set_positions(). Limit velocity to 10.0 for joint" << joint_name << "(requested was" << reference_velocity << ").";
            reference_velocity = 10.0;
        }

        parts_reference_velocities.at(map.first).push_back(reference_velocity);
    }

    bool ok = true;
    for (const auto& part : enabled_parts_)
    {
        if (joints.at(part).size() == 0)
            continue;

        ok &= (controllers_.at(part)->setRefSpeeds(joints.at(part).size(), joints.at(part).data(), parts_reference_velocities.at(part).data()));

        if (!ok)
        {
            yError() << log_name() << "::set_positions(). Cannot set reference speeds for part " << part;

            return false;
        }
    }

    for (const auto& part : enabled_parts_)
    {
        if (joints.at(part).size() == 0)
            continue;

        ok &= (controllers_.at(part)->positionMove(joints.at(part).size(), joints.at(part).data(), parts_positions.at(part).data()));

        if (!ok)
            yError() << log_name() << "::set_positions(). Cannot set position reference for part " << part;
    }

    return ok;
}


bool iCubMotorsPositions::check_motion_done(const std::vector<std::string>& considered_joints)
{
    std::unordered_map<std::string, std::vector<int>> joints;

    for (const auto& part : enabled_parts_)
        joints[part] = std::vector<int>(0);

    for (std::size_t i = 0; i < considered_joints.size(); i++)
    {
        const std::string joint_name = considered_joints.at(i);
        const auto& map = name_to_index_.at(joint_name);
        joints.at(map.first).push_back(map.second);
    }

    bool ok = true;
    bool done = true;
    for (const auto& part : enabled_parts_)
    {
        bool done_part;

        if (joints.at(part).size() == 0)
            continue;

        ok &= (controllers_.at(part)->checkMotionDone(joints.at(part).size(), joints.at(part).data(), &done_part));
        if (!ok)
        {
            yError() << log_name() << "::check_motion_done(). Cannot check if motion is done for part " << part;

            return false;
        }

        done &= done_part;
    }

    return done;
}
