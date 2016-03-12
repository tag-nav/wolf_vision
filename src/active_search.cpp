/**
 * \file activeSearch.cpp
 * \date 10/04/2010
 * \author jsola, dinesh
 */

#include "active_search.h"

// CLASS ActiveSearchGrid
ActiveSearchGrid::ActiveSearchGrid(const int & _img_size_h, const int & _img_size_v, const int & _n_cells_h,
                                   const int & _n_cells_v, const int & _margin, const int & _separation) :
    separation_(_separation), margin_(_margin) {

    projections_count_.resize(_n_cells_v + 1, _n_cells_h +1);
    empty_cells_tile_tmp_.resize(2, (_n_cells_h + 1) * (_n_cells_v + 1));
    img_size_(0) = _img_size_h;
    img_size_(1) = _img_size_v;
    grid_size_(0) = projections_count_.rows();
    grid_size_(1) = projections_count_.cols(), cell_size_(0) = _img_size_v / _n_cells_v;
    cell_size_(1) = _img_size_h / _n_cells_h;
    offset_ = -cell_size_ / 2;
    renew();
}


// Functions to fill in cells
void ActiveSearchGrid::hitCell(const Eigen::Vector2i & p) {
    Eigen::Vector2i cell = pix2cell(p);
    if (cell(0) < 0 || cell(1) < 0 || cell(0) >= grid_size_(0) || cell(1) >= grid_size_(1))
        return;
    if (projections_count_(cell(0), cell(1)) == -1)
        projections_count_(cell(0), cell(1)) = 0;
    projections_count_(cell(0), cell(1))++;
}

bool ActiveSearchGrid::pickEmptyCell(Eigen::Vector2i & cell) {
    int k = 0;
    Eigen::Vector2i cell0;
    for (int i = 1; i < grid_size_(0) - 1; i++) {
        for (int j = 1; j < grid_size_(1) - 1; j++) {
            cell0(0) = i;
            cell0(1) = j;
            if (projections_count_(cell0(0), cell0(1)) == 0) {
                empty_cells_tile_tmp_(0,k) = cell0(0); //may be done in a better way
                empty_cells_tile_tmp_(1,k) = cell0(1);
                k++;
            }
        }
    }
    if (k > 0) { // number of empty inner cells
        int idx = rand() % k; // between 0 and k-1
        cell(0) = empty_cells_tile_tmp_(0, idx);
        cell(1) = empty_cells_tile_tmp_(1, idx);
        return true;
    }
    else
        return false;
}

/*
 * Get cell origin (exact pixel)
 */
Eigen::Vector2i ActiveSearchGrid::cellOrigin(const Eigen::Vector2i & cell) {
    Eigen::Vector2i cell0;
    cell0(0) = offset_(0) + cell_size_(0) * cell(0);
    cell0(1) = offset_(1) + cell_size_(1) * cell(1);
    return cell0;
}

void ActiveSearchGrid::cell2roi(const Eigen::Vector2i & cell, cv::Mat & roi) {
    roi_coordinates_ = cellOrigin(cell);
    roi_coordinates_(0) += separation_;
    roi_coordinates_(1) += separation_;
    Eigen::Vector2i s = cell_size_;
    s(0) -= 2 * separation_;
    s(1) -= 2 * separation_;
    roi = roi(cv::Rect(roi_coordinates_(0),roi_coordinates_(1),s(0),s(1)));
}


bool ActiveSearchGrid::pickRoi(cv::Mat & roi) {
    Eigen::Vector2i cell;
    if (pickEmptyCell(cell)) {
        cell2roi(cell, roi);
        return true;
    }
    else
        return false;
}

void ActiveSearchGrid::blockCell(const cv::Mat & roi)
{
    Eigen::Vector2i p; p(1) = roi_coordinates_(1)+roi.cols/2; p(0) = roi_coordinates_(0)+roi.rows/2;
    Eigen::Vector2i cell = pix2cell(p);
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
