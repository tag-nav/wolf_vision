/**
 * \file test_map_yaml.cpp
 *
 *  Created on: Jul 27, 2016
 *      \author: jsola
 */




#include "wolf.h"
#include "problem.h"
#include "map_base.h"
#include "landmark_polyline_2D.h"
#include "state_block.h"

#include <iostream>

int main()
{
    using namespace wolf;

    char* w = std::getenv("WOLF_ROOT");
    if (w == NULL)
        throw std::runtime_error("Environment variable WOLF_ROOT not found");

    std::string WOLF_ROOT       = w;
    std::string WOLF_CONFIG     = WOLF_ROOT + "/src/examples";
    std::cout << "\nWolf directory for configuration files: " << WOLF_CONFIG << std::endl;

    Problem problem(FRM_PO_2D);
    std::cout << "Reading map from file: " << WOLF_CONFIG + "/map_polyline_example.yaml" << std::endl;
    problem.getMapPtr()->load(WOLF_CONFIG + "/map_polyline_example.yaml");

    std::cout << "printing map..." << std::endl;

    for (auto lmk_ptr : *(problem.getMapPtr()->getLandmarkListPtr()))
    {
        std::cout << "Lmk ID:    " << lmk_ptr->id();
        LandmarkPolyline2D* poly_ptr = (LandmarkPolyline2D*)lmk_ptr;
        std::cout << "\nn points:  " << poly_ptr->getNPoints();
        std::cout << "\nFirst idx: " << poly_ptr->getFirstId();
        std::cout << "\nFirst def: " << poly_ptr->isFirstDefined();
        std::cout << "\nLast  def: " << poly_ptr->isLastDefined();
        for (int idx = poly_ptr->getFirstId(); idx <= poly_ptr->getLastId(); idx++)
            std::cout << "\n  point: " << idx << ": " << poly_ptr->getPointStateBlockPtr(idx)->getVector().transpose();
        std::cout << std::endl;
    }


    return 0;
}
