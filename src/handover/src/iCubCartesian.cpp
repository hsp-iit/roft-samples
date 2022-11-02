/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCartesian.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

using namespace yarp::dev;
using namespace yarp::os;


iCubCartesian::iCubCartesian
(
    const std::string& robot_name,
    const std::string& laterality,
    const std::string& port_prefix
) :
    laterality_(laterality)
{
    /* Check for laterality. */
    if ((laterality != "right") && (laterality != "left"))
    {
        throw std::runtime_error(log_name_ + "::ctor. Error: invalid laterality" + laterality + ".");
    }

    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP is not available."));
    }

    /* Drivers configuration. */
    yarp::os::Property properties;
    properties.put("device", "cartesiancontrollerclient");
    properties.put("remote", "/" + robot_name + "/cartesianController/" + laterality_ + "_arm");
    properties.put("local", "/" + port_prefix + "/" + laterality_ + "_arm/cartesiancontroller");

    /* Open driver. */
    bool ok = driver_cartesian_.open(properties) && driver_cartesian_.view(controller_) && (controller_ != nullptr);

    if (!ok)
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open the " + laterality_ + " arm cartesian controller (driver)."));

    /* Cartesian Controller configuration. */
    controller_->storeContext(&initial_context_);
    controller_->setTrajTime(5.0);
}


void iCubCartesian::close()
{
    stop();
    controller_->restoreContext(initial_context_);

    if (driver_cartesian_.isValid())
        driver_cartesian_.close();
}


bool iCubCartesian::check_motion_done()
{
    bool done;
    controller_->checkMotionDone(&done);

    return done;
}


yarp::dev::ICartesianControl& iCubCartesian::controller()
{
    return *controller_;
}



bool iCubCartesian::enable_torso(const bool& enable_yaw, const bool& enable_pitch, const bool& enable_roll)
{
    yarp::sig::Vector current_dof;
    yarp::sig::Vector new_dof;

    bool ok;

    /* Get current configuration. */
    ok = controller_->getDOF(current_dof);
    if (!ok)
    {
        yError() << log_name_ + "::enable_torso. Error: cannot get current DOF configuration of the arm.";

        return false;
    }

    /* Enable the torso. */
    new_dof = current_dof;
    new_dof[0] = (enable_pitch ? 1 : 0);
    new_dof[1] = (enable_roll ? 1 : 0);
    new_dof[2] = (enable_yaw ? 1 : 0);

    ok = controller_->setDOF(new_dof, current_dof);
    if (!ok)
    {
        yError() << log_name_ + "::enable_torso. Error: cannot set the new DOF configuration of the arm.";

        return false;
    }

    return true;
}


bool iCubCartesian::enable_torso_limits(const std::string& torso_part, const double& min, const double& max)
{
    std::size_t axis;
    if (torso_part == "yaw")
        axis = 2;
    else if (torso_part == "pitch")
        axis = 0;
    else if (torso_part == "roll")
        axis = 1;
    else
    {
        yError() << log_name_ + "::enable_torso_limits. Error: the requested torso part is invalid.";

        return false;
    }

    return controller_->setLimits(axis, min, max);
}


bool iCubCartesian::enable_arm_limits(const std::string& arm_part, const double& min, const double& max)
{
    return controller_->setLimits(arm_joints_map_.at(arm_part), min, max);
}


bool iCubCartesian::go_to_pose(const yarp::sig::Vector& position, const yarp::sig::Vector& orientation)
{
    return controller_->goToPose(position, orientation);
}


bool iCubCartesian::go_to_pose_stream(const yarp::sig::Vector& position, const yarp::sig::Vector& orientation)
{
    return controller_->goToPoseSync(position, orientation);
}


bool iCubCartesian::go_to_position(const yarp::sig::Vector& position)
{
    yarp::sig::Vector current_position;
    yarp::sig::Vector current_orientation;
    controller_->getPose(current_position, current_orientation);

    return controller_->goToPose(position, current_orientation);
}


bool iCubCartesian::go_to_position_stream(const yarp::sig::Vector& position)
{
    yarp::sig::Vector current_position;
    yarp::sig::Vector current_orientation;
    controller_->getPose(current_position, current_orientation);

    return controller_->goToPoseSync(position, current_orientation);
}


bool iCubCartesian::restore_context()
{
    return controller_->restoreContext(current_context_);
}


bool iCubCartesian::store_context()
{
    return controller_->storeContext(&current_context_);
}


bool iCubCartesian::stop()
{
    return controller_->stopControl();
}
