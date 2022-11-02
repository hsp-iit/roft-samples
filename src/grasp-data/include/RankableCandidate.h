/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef RANKABLE_CANDIDATE_H
#define RANKABLE_CANDIDATE_H

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

typedef std::tuple<std::string, double, yarp::sig::Matrix, yarp::sig::Vector> rankable_candidate;

#endif
