/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MODULE_H
#define MODULE_H

#include <Eigen/Dense>
#include <viewer.h>

#include <yarp/os/RFModule.h>


class Module
{
public:
    bool run(yarp::os::ResourceFinder& rf);

private:
    std::shared_ptr<viewer::Viewer> viewer_;

    const std::string log_name_ = "roft-samples-grasp-viewer";
};

#endif /* MODULE_H */
