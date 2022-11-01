/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <Eigen/Dense>

#include <ROFTFilter.h>

#include <ROFT/ModelParameters.h>

#include <thrift/TrackerIDL.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Port.h>

#include <string>


class Tracker : public TrackerIDL
{
public:
    Tracker(yarp::os::ResourceFinder& resource_finder);

    /**
     * IDL interface.
     */

    std::string quit() override;

    std::string reset() override;

    std::string select_object(const std::string& object_name) override;

    std::string start() override;

    std::string stop() override;

private:
    Eigen::VectorXd load_vector_double(const yarp::os::Bottle& resource, const std::string& key, const std::size_t size);

    std::unique_ptr<ROFTFilter> filter_;

    yarp::os::Port port_rpc_;

    std::string textured_model_external_path_root_;

    ROFT::ModelParameters model_parameters_;

    const std::string log_name_ = "roft-samples-tracker";
};

#endif /* TRACKER_H */
