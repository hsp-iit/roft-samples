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
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>


class iCubGaze
{
public:
    iCubGaze(const std::string& robot_name, const std::string& port_prefix, const double& neck_time, const double& eyes_time, const double& neck_time_home, const double& eyes_time_home);

    void close();

    yarp::dev::IGazeControl& controller();

    bool look_at(const yarp::sig::Vector& target);

    bool look_at_stream(const yarp::sig::Vector& target);

    bool go_home();

    void set_home_configuation(const yarp::sig::Vector& configuration);

    bool stop();

private:
    void restore_from_home_context();

    yarp::os::Network yarp_;

    /* IGazeInterface related. */
    yarp::dev::PolyDriver driver_gaze_;

    yarp::dev::IGazeControl* gaze_;

    int gaze_startup_context_;

    yarp::sig::Vector home_configuration_;

    double neck_time_;
    double eyes_time_;
    double neck_time_home_;
    double eyes_time_home_;

    bool home_context_set_ = false;
    int context_before_home_;

    /* Log name for messages. */
    const std::string log_name_ = "iCubGaze";
};

#endif /* ICUBGAZE_H */
