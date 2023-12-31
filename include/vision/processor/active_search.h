//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
/**
 * \file active_search.h
 *
 *  Active search detection and matching for points.
 *
 * \date 10/04/2016
 * \author jsola, dinesh
 */

#ifndef ACTIVESEARCH_H_
#define ACTIVESEARCH_H_

// Wolf includes
#include <core/common/wolf.h>

//OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace wolf{

/**
 * \brief Active search tesselation grid.
 *
 * \author jsola, dinesh
 *
 * This class implements a tesselation grid for achieving active search
 * behavior in landmark initialization.
 *
 * The grid defines a set of cells in the image.
 * The idea is to count the number of projected landmarks, or active tracks, per grid cell,
 * and use one randomly chosen cell that is empty
 * for feature detection and landmark or track initialization.
 * This guarantees that the system will automatically populate all the
 * regions of the image.
 *
 * The feature density can be controlled by adjusting the grid's number of cells.  Important notes:
 * - Typically, use grids of 5x5 to 18x12 cells.
 * - Try to make reasonably square cells.
 * - The final cell sizes are always integers, even if the H and V number of cells are not an exact divisors of the image size.
 *
 * This class implements a few interesting features:
 * - The grid can be randomly re-positioned at each frame to avoid dead zones at the cell edges.
 * - Only the inner cells are activated for feature detection to avoid reaching the image edges.
 * - The region of interest (ROI) associated with a particular cell is shrunk with a parameterizable amount
 *   to guarantee a minimum 'separation' between existing and new features.
 * - The region of interest is ensured to lie at a distance from the image boundaries, defined by the parameter 'margin'.
 *
 * The blue and green grids in the figure below represent the grid
 * at two different offsets, corresponding to two different frames.
 *
 *   \image html tesselationGrid.png "The tesselation grid used for active feature detection and initialization"
 *
 * This second figure shows a typical situation that we use to explain the basic mechanism.
 *
 *   \image html tesselationExample.png "A typical configuration of the tesselation grid"
 *
 * Observe the figure and use the following facts as an operation guide:
 * - The grid is offset by a fraction of a cell size.
 *     - use renew() at each frame to clear the grid and set a new offset.
 * - Projected landmarks are represented by red dots.
 *     - After projection, use hitCell() to add a new dot to the grid.
 * - Cells with projected landmarks inside are 'occupied'.
 * - Only the inner cells (thick blue rectangle) are considered for Region of Interest (ROI) extraction.
 * - One cell is chosen randomly among those that are empty.
 *     - Use pickRoi() to obtain an empty ROI for initialization.
 * - The ROI is smaller than the cell to guarantee a minimum feature separation.
 *     - Use the optional 'separation' parameter at construction time to control this separation.
 * - A new feature is to be be searched inside this ROI.
 * - If there is no feature found in this ROI, call blockCell() function to avoid searching in this area again.
 * - If you need to search more than one feature per frame, proceed like this:
 *     - Call pickRoi().
 *     - Try to detect a Feature in ROI.
 *     - If successful detection
 *         - add the detected pixel with hitCell().
 *     - Else
 *         - block the cell with blockCell().
 *     - Repeat these steps for each feature to be searched.
 *
 * We include here a schematic active-search pseudo-code algorithm to illustrate its operation:
 * \code
 * // Init necessary objects
 * ActiveSearch activeSearch;
 * ActiveSearchGrid grid(640, 480, 4, 4, 10); // Construction with 10 pixels separation.
 *
 * // We start projecting everybody
 * for (obs = begin(); obs != end(); obs++)   // loop observations
 * {
 *   obs->project();
 *   if (obs->isVisible())
 *     grid.hitCell(obs->pixel());   // add only visible landmarks
 * }
 *
 * // Then we process the selected observations
 * activeSearch.selectObs();                  // select interesting features
 * for (activeSearch.selectedObs);            // loop selected obs
 *   obs.process();                           // process observation
 *
 * // Now we go to initialization
 * num_new_detections = 0;
 * while(num_new_detections < max_detections)
 *   grid.pickRoi(roi);                         // roi is now region of interest
 *   if (detectFeature(roi))                    // detect inside ROI
 *     initLandmark();                          // initialize only if successful detection
 *     num_new_detections++;
 *   else
 *     blockCell(roi)
 *
 * \endcode
 *
 */
class ActiveSearchGrid {

    private:
        Eigen::Vector2i img_size_;
        Eigen::Vector2i grid_size_;
        Eigen::Vector2i cell_size_;
        Eigen::Vector2i offset_;
        Eigen::Vector2i roi_coordinates_;
        Eigen::MatrixXi projections_count_;
        Eigen::MatrixXi empty_cells_tile_tmp_;
        int separation_;
        int margin_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)

        /**
         * \brief Void constructor
         *
         * Calling this constructor requires the use of setup() to configure.
         */
        ActiveSearchGrid();

        /**
         * \brief Constructor.
         * \param _img_size_h horizontal image size, in pixels.
         * \param _img_size_v vertical image size.
         * \param _n_cells_h horizontal number of cells per image width.
         * \param _n_cells_v vertical number of cells per image height.
         * \param _margin minimum separation to the edge of the image
         * \param _separation minimum separation between existing and new points.
         */
        ActiveSearchGrid(const int & _img_size_h, const int & _img_size_v,
                         const int & _n_cells_h,  const int & _n_cells_v,
                         const int & _margin = 0,
                         const int & _separation = 0);

        /**
         * \brief Function to set the parameters of the active search grid
         * \param _img_size_h horizontal image size, in pixels.
         * \param _img_size_v vertical image size.
         * \param _n_cells_h horizontal number of cells per image width.
         * \param _n_cells_v vertical number of cells per image height.
         * \param _margin minimum separation to the edge of the image
         * \param _separation minimum separation between existing and new points.
         */
        void setup(const int & _img_size_h, const int & _img_size_v,
                   const int & _n_cells_h,  const int & _n_cells_v,
                   const int & _margin = 0,
                   const int & _separation = 0);

        /**
         * \brief Re-set the image size
         * \param _img_size_h horizontal image size, in pixels.
         * \param _img_size_v vertical image size.
         */
        void resizeImage(unsigned int _img_size_h, unsigned int _img_size_v);

        /** \brief Clear grid.
         *
         * Sets all cell counters to zero.
         */
        void clear();

        /**
         * \brief Clear grid and position it at a new random location.
         *
         * Sets all cell counters to zero and sets a new random grid position.
         */
        void renew();

        /**
         * \brief Add a projected pixel to the grid.
         * If the cell is blocked, unblock and add.
         * \param _x the x-coordinate of the pixel to add.
         * \param _y the y-coordinate of the pixel to add.
         */
        template<typename Scalar>
        void hitCell(const Scalar _x, const Scalar _y);

        /**
         * \brief Add a projected pixel to the grid.
         * If the cell is blocked, unblock and add.
         * \param _pix the pixel to add as an Eigen 2-vector with any Scalar type (can be a non-integer).
         */
        template<typename Scalar>
        void hitCell(const Eigen::Matrix<Scalar, 2, 1>& _pix);

        /**
         * \brief Add a projected pixel to the grid.
         * If the cell is blocked, unblock and add.
         * \param _pix the pixel to add as a cv::KeyPoint.
         */
        void hitCell(const cv::KeyPoint& _pix);

        /**
         * \brief Get ROI of a random empty cell.
         * \param _roi the resulting ROI
         * \return true if ROI exists.
         */
        bool pickRoi(cv::Rect & _roi);

        /**
         * \brief Block a cell known to be empty in order to avoid searching again in it.
         * \param _cell the cell where nothing was found
         */
        void blockCell(const Eigen::Vector2i & _cell);

        /**
         * \brief Call this after pickRoi if no point was found in the roi
         * in order to avoid searching again in it.
         * \param _roi the ROI where nothing was found
         */
        void blockCell(const cv::Rect & _roi);

        /**
         * \brief Get the region of interest, reduced by a margin.
         */
        void cell2roi(const Eigen::Vector2i & _cell, cv::Rect& _roi);

    private:
        /**
         * \brief Get cell corresponding to pixel
         */
        template<typename Scalar>
        Eigen::Vector2i coords2cell(const Scalar _x, const Scalar _y);

        /**
         * \brief Get cell origin (exact pixel)
         */
        Eigen::Vector2i cellOrigin(const Eigen::Vector2i & _cell);

        /**
         * \brief Get cell center (can be decimal if size of cell is an odd number of pixels)
         */
        Eigen::Vector2i cellCenter(const Eigen::Vector2i& _cell);

        /**
         * \brief Get one random empty cell
         */
        bool pickEmptyCell(Eigen::Vector2i & _cell);

        /**
         * \brief True if the cell is blocked
         * \param _cell the queried cell
         */
        bool isCellBlocked(const Eigen::Vector2i& _cell);

};

inline void ActiveSearchGrid::clear()
{
    projections_count_.setZero();
}

inline void ActiveSearchGrid::renew()
{
    offset_(0) = -(margin_ + rand() % (cell_size_(0) - 2 * margin_)); // from -margin to -(cellSize(0)-margin)
    offset_(1) = -(margin_ + rand() % (cell_size_(1) - 2 * margin_)); // from -margin to -(cellSize(0)-margin)
    clear();
}

inline void ActiveSearchGrid::hitCell(const cv::KeyPoint& _pix)
{
    hitCell(_pix.pt.x, _pix.pt.y);
}

template<typename Scalar>
inline void ActiveSearchGrid::hitCell(const Eigen::Matrix<Scalar, 2, 1>& _pix)
{
    hitCell(_pix(0), _pix(1));
}

template<typename Scalar>
inline void ActiveSearchGrid::hitCell(const Scalar _x, const Scalar _y)
{
    Eigen::Vector2i cell = coords2cell(_x, _y);
    if (cell(0) < 0 || cell(1) < 0 || cell(0) >= grid_size_(0) || cell(1) >= grid_size_(1))
        return;

    if (isCellBlocked(cell))
        projections_count_(cell(0), cell(1)) = 0; // unblock cell: it becomes empty

    projections_count_(cell(0), cell(1))++;
}

template<typename Scalar>
inline Eigen::Vector2i ActiveSearchGrid::coords2cell(const Scalar _x, const Scalar _y)
{
    Eigen::Vector2i cell;
    cell(0) = (_x - offset_(0)) / cell_size_(0);
    cell(1) = (_y - offset_(1)) / cell_size_(1);
    return cell;
}

inline Eigen::Vector2i ActiveSearchGrid::cellCenter(const Eigen::Vector2i& _cell)
{
    return cellOrigin(_cell) + cell_size_ / 2; // beware these are all integers, so the result is truncated to the smaller integer
}

}

#endif /* ACTIVESEARCH_H_ */
