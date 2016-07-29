/**
 * \file active_search.cpp
 * \date 10/04/2016
 * \author jsola, dinesh
 */

#include "active_search.h"

#include <iostream>

namespace wolf{

// CLASS ActiveSearchGrid
ActiveSearchGrid::ActiveSearchGrid() : separation_(0), margin_(0) {}

ActiveSearchGrid::ActiveSearchGrid(const int & _img_size_h, const int & _img_size_v, const int & _n_cells_h,
                                   const int & _n_cells_v, const int & _margin, const int & _separation)
{
    setParameters(_img_size_h, _img_size_v, _n_cells_h, _n_cells_v, _margin, _separation);
}

void ActiveSearchGrid::setParameters(const int & _img_size_h, const int & _img_size_v,
                   const int & _n_cells_h, const int & _n_cells_v,
                   const int & _margin, const int & _separation)
    {
    separation_ = _separation;
    margin_ = _margin;

    projections_count_.resize(_n_cells_h + 1, _n_cells_v + 1);
    empty_cells_tile_tmp_.resize(2, (_n_cells_h + 1) * (_n_cells_v + 1));
    img_size_(0) = _img_size_h;
    img_size_(1) = _img_size_v;
    grid_size_(0) = _n_cells_h + 1;
    grid_size_(1) = _n_cells_v + 1;
    cell_size_(0) = _img_size_h / _n_cells_h;
    cell_size_(1) = _img_size_v / _n_cells_v;
    offset_ = -cell_size_ / 2;
    renew();
}

void ActiveSearchGrid::resizeImage(unsigned int _img_size_h, unsigned int _img_size_v)
{
    img_size_(0) = _img_size_h;
    img_size_(1) = _img_size_v;
    cell_size_(0) = _img_size_h / (grid_size_(0) - 1);
    cell_size_(1) = _img_size_v / (grid_size_(1) - 1);
    offset_ = -cell_size_ / 2;
    renew();
}


// Functions to fill in cells
bool ActiveSearchGrid::pickEmptyCell(Eigen::Vector2i & _cell) {
    int k = 0;
    Eigen::Vector2i cell0;
    for (int i = 1; i < grid_size_(0) - 1; i++) {
        for (int j = 1; j < grid_size_(1) - 1; j++) {
            cell0(0) = i;
            cell0(1) = j;
            if (projections_count_(i, j) == 0) {
                empty_cells_tile_tmp_(0,k) = i; //may be done in a better way
                empty_cells_tile_tmp_(1,k) = j;
                k++;
            }
        }
    }
    if (k > 0) { // number of empty inner cells
        int idx = rand() % k; // between 0 and k-1
        _cell(0) = empty_cells_tile_tmp_(0, idx);
        _cell(1) = empty_cells_tile_tmp_(1, idx);
        return true;
    }
    else
        return false;
}

/*
 * Get cell origin (exact pixel)
 */
Eigen::Vector2i ActiveSearchGrid::cellOrigin(const Eigen::Vector2i & _cell) {
    Eigen::Vector2i cell0;
    cell0(0) = offset_(0) + cell_size_(0) * _cell(0);
    cell0(1) = offset_(1) + cell_size_(1) * _cell(1);
    return cell0;
}

void ActiveSearchGrid::cell2roi(const Eigen::Vector2i & _cell, cv::Rect & _roi) {
    roi_coordinates_ = cellOrigin(_cell);
    roi_coordinates_(0) += separation_;
    roi_coordinates_(1) += separation_;
    Eigen::Vector2i roi_size = cell_size_;
    roi_size(0) -= 2 * separation_;
    roi_size(1) -= 2 * separation_;
    _roi.x = roi_coordinates_(0);
    _roi.y = roi_coordinates_(1);
    _roi.width = roi_size(0);
    _roi.height = roi_size(1);
}


bool ActiveSearchGrid::pickRoi(cv::Rect & _roi) {

    Eigen::Vector2i cell;
    if (pickEmptyCell(cell)) {
        cell2roi(cell, _roi);
        return true;
    }
    else
        return false;
}

void ActiveSearchGrid::blockCell(const cv::Rect & _roi)
{
    Eigen::Vector2i pix;
    pix(0) = _roi.x+_roi.width/2;
    pix(1) = _roi.y+_roi.height/2;
    Eigen::Vector2i cell = coords2cell(pix(0), pix(1));
    projections_count_(cell(0), cell(1)) = -1;
}

/*
#if 0
        ////////////////////////////////////////////////////////
        //    ACTIVE SEARCH ALGORITHMS
        ////////////////////////////////////////////////////////

        map<double, observation_ptr_t> ActiveSearch::projectAll(const sensor_ptr_t & senPtr, size_t & numVis) {
            map<double, observation_ptr_t> visObs;
            for (SensorAbstract::DataManagerList::iterator dmaIter = senPtr->dataManagerList().begin(); dmaIter!=senPtr->dataManagerList().end(); dmaIter++ )
              {
                data_manager_ptr_t dmaPtr = *dmaIter;
                for (DataManagerAbstract::ObservationList::iterator obsIter = dmaPtr->observationList().begin(); obsIter
                   != dmaPtr->observationList().end(); obsIter++) {
                  observation_ptr_t obsPtr = *obsIter;
                  obsPtr->project();
                  obsPtr->predictVisibility();
                  if (obsPtr->isVisible()) {
                obsPtr->predictInfoGain();
                visObs[obsPtr->expectation.infoGain] = obsPtr; // this automatically sorts the observations ! ;-)
                  }
                }
              }
            return visObs;
        }

        void ActiveSearch::predictApp(const observation_ptr_t & obsPtr) {

            // Get landmark descriptor
            landmark_ptr_t lmkPtr = obsPtr->landmarkPtr();

            // Get the sensor's current global pose
            vec7 senPose = obsPtr->sensorPtr()->globalPose();
        }

        void ActiveSearch::scanObs(const observation_ptr_t & obsPtr, const image::ConvexRoi & roi) {
        }
#endif
*/

}
