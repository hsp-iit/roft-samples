/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 *
 *
 * Part of this code is taken from https://github.com/robotology/icub-gazebo-grasping-sandbox/blob/master/src/cardinal_points_grasp.h
 *
 * This is the original BSD 3-Clause LICENSE the original code was provided with:
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Fondazione Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CARDINAL_POINTS_GRASP_H
#define CARDINAL_POINTS_GRASP_H

#include <memory>
#include <vector>
#include <tuple>
#include <utility>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <iostream>

#include <Eigen/Dense>

#include <RankableCandidate.h>

#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/eigen/Eigen.h>

#include <iCub/iKin/iKinFwd.h>

namespace cardinal_points_grasp {

/******************************************************************************/
class CardinalPointsGrasp {
    const std::string hand;
    Eigen::VectorXd pregrasp_fingers_posture;
    Eigen::VectorXd object_sizes;
    Eigen::VectorXd object_offsets;
    Eigen::MatrixXd reference_frame_offset;

    double pregrasp_aperture{0.};
    double hand_half_height{0.};
    double fingers_max_length{0.};
    double dist_center_index_middle{0.};
    double approach_min_distance{0.};
    yarp::sig::Matrix approach;

    std::vector<std::unique_ptr<iCub::iKin::iCubFinger>> fingers;

    auto composeCandidate(const yarp::sig::Vector& axis_x, const yarp::sig::Vector& axis_y,
                          const yarp::sig::Vector& axis_z, const yarp::sig::Vector& point,
                          const yarp::sig::Vector& dir) const {
        auto candidate = yarp::math::zeros(4, 4);
        candidate.setSubcol(axis_x, 0, 0);
        candidate.setSubcol(axis_y, 0, 1);
        candidate.setSubcol(axis_z, 0, 2);
        candidate.setSubcol(point, 0, 3);
        candidate(3, 3) = 1.;
        candidate = candidate * approach;
        candidate.setSubcol(candidate.getCol(3).subVector(0, 2) + approach_min_distance * dir, 0, 3);
        return candidate;
    }

    /**************************************************************************/
    auto evaluateCandidate(const yarp::sig::Matrix& candidate,
                           yarp::dev::ICartesianControl* iarm) const {
        const auto xd = candidate.getCol(3).subVector(0, 2);
        const auto od = yarp::math::dcm2axis(candidate);

        yarp::sig::Vector xdhat, odhat, qdhat;
        if (iarm->askForPose(xd, od, xdhat, odhat, qdhat)) {
            // always enforce reaching in position first
            if (yarp::math::norm(xd - xdhat) < .005) {
                const auto rot = yarp::math::dcm2axis(yarp::math::axis2dcm(od) *
                                 yarp::math::axis2dcm(odhat).transposed());
                // cost in [0, 1] = normalized angle difference
                const auto cost = std::abs(rot[3] / M_PI);
                if (180. * cost < 10.) {
                    return cost;
                }
            }
        }
        return std::numeric_limits<double>::infinity();
    }

public:
    /**************************************************************************/
    CardinalPointsGrasp() = delete;

    /**************************************************************************/
    CardinalPointsGrasp(const std::string& hand_, const Eigen::VectorXd& pregrasp_fingers_posture_) :
        hand(hand_), pregrasp_fingers_posture(pregrasp_fingers_posture_)
    {
        // create fingers and set them up in the pregrasp posture
        std::vector<std::string> fingers_names{"thumb", "index", "middle", "ring", "little"};
        for (auto& name:fingers_names) {
            fingers.push_back(std::make_unique<iCub::iKin::iCubFinger>(iCub::iKin::iCubFinger(hand+"_"+name)));
            auto& finger = fingers.back();
            finger->asChain()->setAllConstraints(false);
            yarp::sig::Vector motorEncoders(pregrasp_fingers_posture.size(), pregrasp_fingers_posture.data());
            yarp::sig::Vector chainJoints;
            finger->getChainJoints(motorEncoders, chainJoints);
            finger->setAng(chainJoints * (M_PI / 180.0));
        }

        // compute the pre-grasp quantities
        const auto p_thumb = fingers[0]->EndEffPosition();
        const auto p_index = fingers[1]->EndEffPosition();
        const auto p_middle = fingers[2]->EndEffPosition();
        const auto dist_1 = yarp::math::norm(p_thumb - p_index);
        const auto dist_2 = yarp::math::norm(p_thumb - p_middle);
        pregrasp_aperture = std::min(dist_1, dist_2);
        const auto p_ = (dist_1 < dist_2 ? p_index : p_middle);
        const auto pregrasp_aperture_angle = std::acos(yarp::math::dot(p_thumb, p_) /
                                                       (yarp::math::norm(p_thumb) * yarp::math::norm(p_)));

        const auto sector_beg = std::atan2(std::abs(p_[2]), p_[0]);
        const auto sector_end = std::atan2(std::abs(p_thumb[2]), p_thumb[0]);

        // account for safety margin due to ring and little fingers that might hamper the approach
        // for (size_t i = 3; i < fingers.size(); i++) {
        //     auto& finger = fingers[i];
        //     for (size_t link = 0; link < finger->getN(); link++) {
        //         auto v = fingers[i]->Position(link);
        //         auto ang = std::atan2(std::abs(v[2]), v[0]);
        //         if ((ang >= sector_beg) && (ang <= sector_end)) {
        //             v[1] = 0.;
        //             approach_min_distance = std::max(yarp::math::norm(v), approach_min_distance);
        //         }
        //     }
        // }
        // approach_min_distance += .005;
        approach_min_distance = 0.02;

        // use little finger to account for safety margin to avoid hitting the table when side-grasping
        // hand_half_height = fingers.back()->EndEffPosition()[1] + .01;

        // compute the fingers max length
        for (auto& finger:fingers) {
            auto length = 0.;
            auto p = finger->EndEffPosition();
            for (size_t i = 0; i < 3; i++) {
                auto j = finger->getDOF() - i - 1;
                auto pi = (j >= 0 ? finger->Position(j) : finger->getH0().getCol(3).subVector(0, 2));
                length += yarp::math::norm(pi - p);
                p = pi;
            }
            fingers_max_length = std::max(length, fingers_max_length);
        }

        // for 3-fingers grasps, better off aligning the hand with the center between index and middle fingers
        const auto p0_index = fingers[1]->Position(1);
        const auto p0_middle = fingers[2]->getH0().getCol(3).subVector(0, 2);
        dist_center_index_middle = std::abs(p0_index[1] - p0_middle[1]) / 2.;

        // compute the approach frame wrt the canonical hand-centered frame
        approach = yarp::math::axis2dcm({0., 1., 0., sector_beg + pregrasp_aperture_angle * (hand == "right" ? -.3 : .3)});
    }

    /**************************************************************************/
    void setObjectSizes(const Eigen::VectorXd& vector)
    {
        object_sizes = vector;
    }

    /**************************************************************************/
    void setObjectOffsets(const Eigen::VectorXd& vector)
    {
        object_offsets = vector;
    }

    /**************************************************************************/
    void setReferenceFrameOffset(const Eigen::MatrixXd& matrix)
    {
        reference_frame_offset = matrix;
    }

    /**************************************************************************/
    static auto compareCandidates(const rankable_candidate& c1, const rankable_candidate& c2) {
        return (std::get<1>(c1) < std::get<1>(c2));
    }

    /**************************************************************************/
    auto getCandidates(Eigen::Transform<double, 3, Eigen::Affine> pose, yarp::dev::ICartesianControl* iarm) const {
        std::vector<rankable_candidate> candidates;
        if (iarm==nullptr) {
            return std::make_pair(candidates, 0);
        }

        // back up current context
        int backup;
        iarm->storeContext(&backup);

        // increase reaching accuracy
        yarp::os::Bottle options;
        auto& opt1 = options.addList();
        opt1.addString("max_iter"); opt1.addInt32(1000);
        auto& opt2 = options.addList();
        opt2.addString("tol"); opt1.addFloat64(.0001);
        auto& opt3 = options.addList();
        opt3.addString("constr_tol"); opt2.addFloat64(.000001);
        iarm->tweakSet(options);
        yarp::sig::Vector dof;

        // retrieve object parameters
        const auto bx = object_sizes(0) / 2;
        const auto by = object_sizes(1) / 2;
        const auto bz = object_sizes(2) / 2;
        const auto max_b = std::max(bx, std::max(by, bz));

        Eigen::Matrix3d rotation = pose.rotation();

        Eigen::Vector3d center_eigen = pose.translation() + rotation * object_offsets;
        yarp::sig::Vector center(3);
        yarp::eigen::toEigen(center) = center_eigen;

        // this moves everything to a common reference frame with the z axis pointing upward
        rotation = rotation * reference_frame_offset;

        const std::vector<Eigen::Vector3d> side_points{Eigen::Vector3d(bx, 0., 0.), Eigen::Vector3d(0., -by, 0.),
                                                       Eigen::Vector3d(-bx, 0., 0.), Eigen::Vector3d(0., by, 0.)};

        // generate side-grasp candidates
        for (size_t i = 0; i < side_points.size(); i++) {
            // prune by comparing the pre-grasp aperture with the object relative size
            if (((i & 0x01) && (2. * bx < .5 * pregrasp_aperture)) ||
                (!(i & 0x01) && (2. * by < .5 * pregrasp_aperture))) {
                Eigen::Vector3d dir_eigen = rotation * side_points.at(i);
                const Eigen::Vector3d side_eigen = center_eigen + dir_eigen;
                dir_eigen.normalize();

                const Eigen::Vector3d axis_y_eigen = -1.0 * rotation.col(2);
                const Eigen::Vector3d axis_z_eigen = (hand == "right" ? -1 : 1) * dir_eigen;
                yarp::sig::Vector dir(3);
                yarp::sig::Vector side(3);
                yarp::sig::Vector axis_y(3);
                yarp::sig::Vector axis_z(3);
                yarp::eigen::toEigen(dir) = dir_eigen;
                yarp::eigen::toEigen(side) = side_eigen;
                yarp::eigen::toEigen(axis_y) = axis_y_eigen;
                yarp::eigen::toEigen(axis_z) = axis_z_eigen;
                const auto axis_x = yarp::math::cross(axis_y, axis_z);
                const auto candidate = composeCandidate(axis_x, axis_y, axis_z,
                                                        side + axis_y * dist_center_index_middle, dir);
                auto cost = evaluateCandidate(candidate, iarm);
                if (cost != std::numeric_limits<double>::infinity()) {
                    // penalize candidate further from COG
                    if (bz != max_b) {
                        cost += 5. / 180.;
                    }
                    candidates.push_back(std::make_tuple(hand, cost, candidate, center));
                }
            }
        }

        // generate top-grasp candidates
        Eigen::Vector3d top_eigen = center_eigen + rotation * Eigen::Vector3d::UnitZ() * bz;
        yarp::sig::Vector top(3);
        yarp::eigen::toEigen(top) = top_eigen;
        for (size_t i = 0; i < side_points.size(); i++) {
            // prune by comparing the pre-grasp aperture with the SQ relative size
            if (((i & 0x01) && (2. * by < .5 * pregrasp_aperture)) ||
                (!(i & 0x01) && (2. * bx < .5 * pregrasp_aperture))) {

                Eigen::Vector3d dir_eigen = rotation * side_points.at(i);
                dir_eigen.normalize();
                yarp::sig::Vector axis_x(3);
                yarp::eigen::toEigen(axis_x) = dir_eigen;

                Eigen::Vector3d axis_z_eigen = (hand == "right" ? -1 : 1) * rotation.col(2);

                yarp::sig::Vector axis_z(3);
                yarp::eigen::toEigen(axis_z) = axis_z_eigen;
                const auto axis_y = yarp::math::cross(axis_z, axis_x);
                const auto candidate = composeCandidate(axis_x, axis_y, axis_z,
                                                        top + axis_y * dist_center_index_middle, {0., 0., 1.});
                // account for size limitations
                if (candidate(2, 3) - fingers_max_length > center[2] - bz) {
                    auto cost = evaluateCandidate(candidate, iarm);
                    if (cost != std::numeric_limits<double>::infinity()) {
                        // penalize candidate further from COG
                        if (bz == max_b) {
                            cost += 5. / 180.;
                        }
                        candidates.push_back(std::make_tuple(hand, cost, candidate, center));
                    }
                }
            }
        }

        std::sort(candidates.begin(), candidates.end(), compareCandidates);

        // restore backup context
        int context;
        iarm->storeContext(&context);
        iarm->restoreContext(backup);
        iarm->deleteContext(backup);
        return std::make_pair(candidates, context);
    }
};

}

#endif
