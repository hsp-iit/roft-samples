/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

service TrackerIDL
{
    string quit();

    string reset();

    string select_object(1:string object_name);

    string start();

    string stop();
}