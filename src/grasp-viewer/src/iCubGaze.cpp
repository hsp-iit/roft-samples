/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubGaze.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;


iCubGaze::iCubGaze(const std::string& port_prefix)
{
    Property properties;
    properties.put("device", "gazecontrollerclient");
    properties.put("remote", "/iKinGazeCtrl");
    properties.put("local", "/" + port_prefix + "/gazecontroller");

    /* Open driver. */
    bool ok = driver_gaze_.open(properties) && driver_gaze_.view(gaze_) && (gaze_ != nullptr);

    if (!ok)
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open IGazeControl interface."));
}


void iCubGaze::close()
{
    if (driver_gaze_.isValid())
        driver_gaze_.close();
}


yarp::dev::IGazeControl& iCubGaze::controller()
{
    return *gaze_;
}
