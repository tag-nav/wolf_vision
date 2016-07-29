/**
 * \file test_map_yaml.cpp
 *
 *  Created on: Jul 27, 2016
 *      \author: jsola
 */




#include "wolf.h"
#include "problem.h"
#include "map_base.h"

#include <iostream>

int main()
{
    using namespace wolf;

    char* w = std::getenv("WOLF_ROOT");
    if (w == NULL)
        throw std::runtime_error("Environment variable WOLF_ROOT not found");

    std::string WOLF_ROOT       = w;
    std::string WOLF_CONFIG     = WOLF_ROOT + "/src/examples";
    std::cout << "\nwolf directory for configuration files: " << WOLF_CONFIG << std::endl;

    Problem problem(FRM_PO_2D);
    std::cout << WOLF_CONFIG + "/map_polyline_example.yaml" << std::endl;
    problem.getMapPtr()->load(WOLF_CONFIG + "/map_polyline_example.yaml");


    return 0;
}
