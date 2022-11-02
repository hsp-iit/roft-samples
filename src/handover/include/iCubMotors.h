/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBMOTORS_H
#define ICUBMOTORS_H

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

#include <string>
#include <vector>
#include <unordered_map>


class iCubMotors
{
public:

    iCubMotors(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const std::string& control_mode, const bool& use_arm, const bool& use_torso);

    virtual ~iCubMotors();

    void close();

    yarp::dev::PolyDriver& driver(const std::string& part);

    yarp::dev::IControlMode& control_mode(const std::string& part);

    bool set_mode(const int& mode, const std::vector<std::string>& considered_joints = std::vector<std::string>()) const;

    virtual bool stop(const std::vector<std::string>& considered_joints = std::vector<std::string>()) const;

    const std::string log_name() const;

protected:
    /*
     * Name to index map.
     */
    std::unordered_map<std::string, std::pair<std::string, int>> name_to_index_ =
    {
        {"torso_yaw", {"torso", 0}},
        {"torso_pitch", {"torso", 2}},
        {"torso_roll", {"torso", 1}},
        {"arm_shoulder_yaw", {"arm", 2}},
        {"arm_shoulder_pitch", {"arm", 0}},
        {"arm_shoulder_roll", {"arm", 1}},
        {"arm_elbow", {"arm", 3}},
        {"arm_wrist_prosup", {"arm", 4}},
        {"arm_wrist_yaw", {"arm", 6}},
        {"arm_wrist_pitch", {"arm", 5}},
        {"hand_hand_finger", {"arm", 7}},
        {"hand_thumb_oppose", {"arm", 8}},
        {"hand_thumb_proximal", {"arm", 9}},
        {"hand_thumb_distal", {"arm", 10}},
        {"hand_index_proximal", {"arm", 11}},
        {"hand_index_distal", {"arm", 12}},
        {"hand_middle_proximal", {"arm", 13}},
        {"hand_middle_distal", {"arm", 14}},
        {"hand_little", {"arm", 15}}
    };

    /*
     * Enabled parts.
     */
    std::vector<std::string> enabled_parts_;

private:
    yarp::os::Network yarp_;

    /**
     * Driver.
     */
    std::unordered_map<std::string, yarp::dev::PolyDriver> drivers_;

    /**
     * View.
     */
    std::unordered_map<std::string, yarp::dev::IControlMode*> modes_;

    /**
     * Log names to be used in messages printed by the class.
     */
    const std::string log_name_ = "iCubMotors";
};

#endif /* ICUBMOTORS_H */
