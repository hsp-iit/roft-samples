/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <module.h>

using namespace Eigen;
using namespace viewer;
using namespace yarp::os;

bool Module::run(yarp::os::ResourceFinder& rf)
{
    /* Get parameters. */
    const double period = rf.check("period", Value(30)).asInt32();
    const std::string object_meshes_path = rf.findPath("roft-samples-tracker") + "/meshes/DOPE_textured";
    const int width = rf.check("width", Value(320)).asInt32();
    const int height = rf.check("height", Value(320)).asInt32();

    /* Set object properties. */
    std::unordered_map<std::string, VectorXd> object_properties;

    object_properties["003_cracker_box"] = Vector3d::Zero();
    object_properties.at("003_cracker_box")(0) = 0.0718;
    object_properties.at("003_cracker_box")(1) = 0.1640;
    object_properties.at("003_cracker_box")(2) = 0.2134;

    object_properties["004_sugar_box"] = Vector3d::Zero();
    object_properties.at("004_sugar_box")(0) = 0.0451;
    object_properties.at("004_sugar_box")(1) = 0.0927;
    object_properties.at("004_sugar_box")(2) = 0.1763;

    object_properties["006_mustard_bottle"] = Vector3d::Zero();
    object_properties.at("006_mustard_bottle")(0) = 0.0582;
    object_properties.at("006_mustard_bottle")(1) = 0.0960;
    object_properties.at("006_mustard_bottle")(2) = 0.1913;

    /* Initialize the viewer and load the objects. */
    viewer_ = std::make_shared<Viewer>(1325, 255, width, height);
    viewer_->loadObjects(object_properties, object_meshes_path);

    /* Start the viewer. */
    viewer_->start(period);

    return true;
}
