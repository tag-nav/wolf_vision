/**
 * \file capture_laser_2D.h
 *
 *  Created on: 06/10/2014
 *     \author: acorominas
 */

#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//wolf includes
#include "capture.h"

//wolf forward declarations
//#include "feature_corner_2D.h"

class CaptureLaser2D : public Capture
{
    public:
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        CaptureLaser2D(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, const NodeLocation _loc = MID);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/        
        virtual ~CaptureLaser2D();
        
        /** \brief Calls the necessary pipeline from raw scan to features.
         * 
         * Calls the necessary pipeline from raw scan to features.
         * 
         **/
        virtual void processCapture();

        /** \brief Extract corners and push-back to Feature down list 
         * 
         * Extract corners and push-back to Feature down list . 
         * 
         **/
        virtual void extractCorners();
};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////


inline CaptureLaser2D::CaptureLaser2D(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, const NodeLocation _loc) :
        Capture(_frm_ptr, _sen_ptr, _loc)
{
    // 
}

inline CaptureLaser2D::~CaptureLaser2D()
{
    //
}

inline void CaptureLaser2D::processCapture()
{
    extractCorners();
}

inline void CaptureLaser2D::extractCorners()
{
    std::cout << "Extracting corners ... " << std::endl;
    //TODO by Andreu: create class FeatureCorner2D + main to test this method.
    //TODO by Juan AC
    // Laser ranges are at data_
    // Scan size is data_.size()
    // Corners should be created as FeatureCorner2D. Corner 3 params to be stored at FeatureBase::measurement_ 
    // After creation, they have to be pushed back to down_node_list_ by means of the method Capture::addFeature(const FeatureShPtr& _f_ptr)  
}

#endif /* CAPTURE_LASER_2D_H_ */
