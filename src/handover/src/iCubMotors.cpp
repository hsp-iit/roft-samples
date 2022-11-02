/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubMotors.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>


using namespace yarp::dev;
using namespace yarp::os;


iCubMotors::iCubMotors
(
    const std::string& robot_name,
    const std::string& laterality,
    const std::string& port_prefix,
    const std::string& control_mode,
    const bool& use_arm,
    const bool& use_torso
) :
    log_name_("iCubMotors[" + control_mode + "]")
{
    /* Check for laterality. */
    if ((laterality != "right") && (laterality != "left") && (laterality != "no_laterality"))
    {
        throw std::runtime_error(log_name_ + "::ctor. Error: invalid laterality " + laterality + ".");
    }

    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP is not available."));
    }

    /* Control mode driver configuration (arm). */
    if (use_arm)
    {
        enabled_parts_.push_back("arm");

        Property properties;
        properties.put("device", "remote_controlboard");
        properties.put("local", "/" + port_prefix + "/" + laterality + "_arm/motors/" + control_mode);
        properties.put("remote", "/" + robot_name + "/" + laterality + "_arm");

        /* Try to open the driver. */
        if (!drivers_["arm"].open(properties))
        {
            throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open the " + laterality + " arm (driver)."));
        }

        /* Try to get the view. */
        if (!(drivers_.at("arm").view(modes_["arm"]) && (modes_.at("arm") != nullptr)))
        {
            throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open the " + laterality + " arm (view)."));
        }
    }

    /* Control mode driver configuration (torso). */
    if (use_torso)
    {
        enabled_parts_.push_back("torso");

        Property properties;
        properties.put("device", "remote_controlboard");
        properties.put("local", "/" + port_prefix + "/torso/" + laterality + "/motors/" + control_mode);
        properties.put("remote", "/" + robot_name + "/torso");

        /* Try to open the driver. */
        if (!drivers_["torso"].open(properties))
        {
            throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open the torso (driver)."));
        }

        /* Try to get the view. */
        if (!(drivers_.at("torso").view(modes_["torso"]) && (modes_.at("torso") != nullptr)))
        {
            throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open the torso (view)."));
        }
    }
}


iCubMotors::~iCubMotors()
{}


void iCubMotors::close()
{
   for (auto& driver : drivers_)
    {
        if (driver.second.isValid())
            driver.second.close();
    }
}


yarp::dev::PolyDriver& iCubMotors::driver(const std::string& part)
{
    return drivers_.at(part);
}


yarp::dev::IControlMode& iCubMotors::control_mode(const std::string& part)
{
    return *(modes_.at(part));
}


bool iCubMotors::set_mode(const int& mode, const std::vector<std::string>& considered_joints) const
{
    std::unordered_map<std::string, std::vector<int>> joints;
    std::unordered_map<std::string, std::vector<int>> control_modes;

    if (considered_joints.size() == 0)
    {
        control_modes["torso"] = std::vector<int>(3, mode);
        control_modes["arm"] = std::vector<int>(16, mode);

        joints["torso"] = {0, 1, 2};
        joints["arm"] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    }
    else
    {
        for (const auto& part : enabled_parts_)
        {
            joints[part] = std::vector<int>(0);
            control_modes[part] = std::vector<int>(0);
        }

        for (std::size_t i = 0; i < considered_joints.size(); i++)
        {
            const auto& map = name_to_index_.at(considered_joints.at(i));
            joints.at(map.first).push_back(map.second);
            control_modes.at(map.first).push_back(mode);
        }
    }

    bool ok = true;
    for (const auto& part : enabled_parts_)
    {
        ok &= (modes_.at(part)->setControlModes(joints.at(part).size(), joints.at(part).data(), control_modes.at(part).data()));

        if (!ok)
            yError() << log_name_ << "::set_mode. Cannot set mode for part" << part;
    }

    return ok;
}


bool iCubMotors::stop(const std::vector<std::string>& considered_joints) const
{
    return set_mode(VOCAB_CM_POSITION, considered_joints);
}


const std::string iCubMotors::log_name() const
{
    return log_name_;
}
