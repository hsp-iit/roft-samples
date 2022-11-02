/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Utils.h>

#include <iostream>

using namespace yarp::os;
using namespace yarp::sig;


std::pair<bool, Vector> Utils::load_vector_double
(
    const Bottle& rf,
    const std::string& key,
    const int& size
)
{
    if (rf.find(key).isNull())
    {
        std::cout << "load_vector_double. Error: " << key << " cannot be found" << std::endl;
        return std::make_pair(false, Vector());
    }

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
    {
        std::cout << "load_vector_double. Error: null bottle pointing to " << key << std::endl;
        return std::make_pair(false, Vector());
    }

    if ((size != -1) && (b->size() != size))
    {
        std::cout << "load_vector_double. Error: " << key << " does not contain " << size << " items (they are " << b->size() << ")" << std::endl;
        return std::make_pair(false, Vector());
    }

    Vector vector(b->size());
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
        {
            std::cout << "load_vector_double. Error: " << i << "-th value of " << key << " is null" << std::endl;
            return std::make_pair(false, Vector());
        }

        if (!(item_v.isFloat64()))
        {
            std::cout << "load_vector_double. Error: " << i << "-th value of " << key << " is not a double" << std::endl;
            return std::make_pair(false, Vector());
        }

        vector(i) = item_v.asFloat64();
    }

    return std::make_pair(true, vector);
}


std::pair<bool, std::vector<std::string>> Utils::load_vector_string
(
    const Bottle& rf,
    const std::string& key,
    const int& size
)
{
    std::vector<std::string> vector;

    if (rf.find(key).isNull())
    {
        std::cout << "load_vector_string. Error: " << key << " cannot be found" << std::endl;
        return std::make_pair(false, vector);
    }

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
    {
        std::cout << "load_vector_string. Error: null bottle pointing to " << key << std::endl;
        return std::make_pair(false, vector);
    }

    if ((size != -1) && (b->size() != size))
    {
        std::cout << "load_vector_string. Error: " << key << " does not contain " << size << " items (they are " << b->size() << ")" << std::endl;
        return std::make_pair(false, vector);
    }

    vector.resize(b->size());
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
        {
            std::cout << "load_vector_string. Error: " << i << "-th value of " << key << " is null" << std::endl;
            return std::make_pair(false, vector);
        }

        if (!(item_v.isString()))
        {
            std::cout << "load_vector_string. Error: " << i << "-th value of " << key << " is not a double" << std::endl;
            return std::make_pair(false, vector);
        }

        vector.at(i) = item_v.asString();
    }

    return std::make_pair(true, vector);
}
