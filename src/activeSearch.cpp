/**
 * \file activeSearch.cpp
 * \date 10/04/2010
 * \author jsola
 * \ingroup rtslam
 */

#include "activeSearch.h"
/*
#include "rtslam/observationAbstract.hpp"
#include "rtslam/sensorAbstract.hpp"
*/

using namespace Eigen;
using namespace std;

///////////////////////////////////////////
// ACTIVE SEARCH TESSELATION GRID
///////////////////////////////////////////

ostream& operator <<(ostream & s, ActiveSearchGrid const & grid) {
    s << "feature count: " << grid.projectionsCount;
    return s;
}


// CLASS ActiveSearchGrid
ActiveSearchGrid::ActiveSearchGrid(const int & imgSize_h, const int & imgSize_v, const int & nCells_h,
                                   const int & nCells_v, const int & margin, const int & separation) :
    separation(separation), margin(margin) {

    projectionsCount.resize(nCells_v + 1, nCells_h +1);
    emptyCellsTile_tmp.resize(2, (nCells_h + 1) * (nCells_v + 1));
    imgSize(0) = imgSize_h;
    imgSize(1) = imgSize_v;
    gridSize(0) = projectionsCount.rows();
    gridSize(1) = projectionsCount.cols(), cellSize(0) = imgSize_v / nCells_v;
    cellSize(1) = imgSize_h / nCells_h;
    offset = -cellSize / 2;
    renew();
}


// Functions to fill in cells
void ActiveSearchGrid::addObs(const vec2 & p) {
    /*veci2 cell = pix2cell(p);
            if (cell(0) < 0 || cell(1) < 0 || cell(0) >= gridSize(0) || cell(1) >= gridSize(1))
                return;
            if (projectionsCount(cell(0), cell(1)) == -1)
                projectionsCount(cell(0), cell(1)) = 0;
            projectionsCount(cell(0), cell(1))++;*/
}

void ActiveSearchGrid::clear() {
    projectionsCount.setZero();
}

void ActiveSearchGrid::renew() {
    offset(0) = - (margin + rand() % (cellSize(0) - 2*margin)); // from -margin to -(cellSize(0)-margin)
            offset(1) = - (margin + rand() % (cellSize(1) - 2*margin)); // from -margin to -(cellSize(0)-margin)
            clear();
}


/*
         * Get one empty cell
         */
bool ActiveSearchGrid::pickEmptyCell(Eigen::Vector2i & cell) {
    int k = 0;
    Eigen::Vector2i cell0;
    for (int i = 1; i < gridSize(0) - 1; i++) {
        for (int j = 1; j < gridSize(1) - 1; j++) {
            cell0(0) = i;
            cell0(1) = j;
            if (projectionsCount(cell0(0), cell0(1)) == 0) {
                emptyCellsTile_tmp(0,k) = cell0(0); //may be done in a better way
                emptyCellsTile_tmp(1,k) = cell0(1);
                k++;
            }
        }
    }
    if (k > 0) { // number of empty inner cells
        //				int idx = (double) rtslam::rand() / RAND_MAX * k;
        int idx = rand() % k; // between 0 and k-1
        cell(0) = emptyCellsTile_tmp(0, idx);
        cell(1) = emptyCellsTile_tmp(1, idx);
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
    cell0(0) = offset(0) + cellSize(0) * cell(0);
    cell0(1) = offset(1) + cellSize(1) * cell(1);
    return cell0;
}


/*
         * Get cell center (can be decimal if size of cell is an odd number of pixels)
         */
vec2 ActiveSearchGrid::cellCenter(const Eigen::Vector2i & cell) {
    return cellOrigin(cell) + cellSize / 2;
}

void ActiveSearchGrid::cell2roi(const veci2 & cell, cv::Mat & roi) {
    Eigen::Vector2i ul = cellOrigin(cell);
    ul(0) += separation;
    ul(1) += separation;
    Eigen::Vector2i s = cellSize;
    s(0) -= 2 * separation;
    s(1) -= 2 * separation;
    roi.init(cv::Rect(ul(0),ul(1),s(0),s(1)));
}


/**
         * Get ROI of one random empty cell
         */
bool ActiveSearchGrid::getRoi(cv::Mat & roi) {
    Eigen::Vector2i cell;
            if (pickEmptyCell(cell)) {
                cell2roi(cell, roi);
                return true;
            }
            else
                return false;
}

void ActiveSearchGrid::setFailed(const cv::Mat & roi)
{
    Eigen::Vector2i p; p(1) = roi.x()+roi.w()/2; p(2) = roi.y()+roi.h()/2;
    Eigen::Vector2i cell = pix2cell(p);
    projectionsCount(cell(0), cell(1)) = -1;
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
