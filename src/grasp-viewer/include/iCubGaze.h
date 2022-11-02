/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBGAZE_H
#define ICUBGAZE_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>


class iCubGaze
{
public:
    iCubGaze(const std::string& port_prefix);

    void close();

    yarp::dev::IGazeControl& controller();

private:
    yarp::dev::PolyDriver driver_gaze_;

    yarp::dev::IGazeControl* gaze_;

    const std::string log_name_ = "iCubGaze";
};

#endif /* ICUBGAZE_H */
