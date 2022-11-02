/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

struct YARPMatrix{}
(
    yarp.name = "yarp::sig::Matrix"
    yarp.includefile="yarp/sig/Matrix.h"
)

struct YARPVector{}
(
    yarp.name = "yarp::sig::Vector"
    yarp.includefile="yarp/sig/Vector.h"
)

struct RankableCandidate
{
    1: string hand;
    2: double cost;
    3: YARPMatrix candidate;
    4: YARPVector center;
}

struct GraspData
{
    1: string object_name;
    2: YARPMatrix object_pose;
    3: list<RankableCandidate> candidates;
}