/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>
#include <iostream>

#include <tracker.h>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;


int main(int argc, char** argv)
{
    const std::string log_name = "roft-samples-tracker";

    /* Check YARP network. */
    Network yarp;
    if (!yarp.checkNetwork())
    {
        std::cerr << log_name + "::main(). Error: YARP network is not available." << std::endl;
        return EXIT_FAILURE;
    }

    /* Load configuration file. */
    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("roft-samples-tracker");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    /* Initialize the tracker. */
    Tracker tracker(rf);

    return EXIT_SUCCESS;
}
