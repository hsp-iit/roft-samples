/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>


namespace Utils {
    std::pair<bool, yarp::sig::Vector> load_vector_double(const yarp::os::Bottle& rf, const std::string& key, const int& size = -1);

    std::pair<bool, std::vector<std::string>> load_vector_string(const yarp::os::Bottle& rf, const std::string& key, const int& size = -1);
}

#endif /* UTILS_H */
