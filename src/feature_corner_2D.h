/**
 * \file feature_corner_2D.h
 *
 *  Created on: 17/10/2014
 *     \author: acorominas
 */

#ifndef feature_corner_2D_h_
#define feature_corner_2D_h_

//wolf includes
#include "feature_base.h"

class FeatureCorner2D : public FeatureBase
{
    protected:
        
    
    public:
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        FeatureCorner2D(const CaptureShPtr& _capt_ptr, unsigned int _dim_measurement=3, const NodeLocation _loc = MID);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/        
        virtual ~FeatureCorner2D();
        
        /** \brief Find a correspondence to Calls the necessary pipeline from raw scan to features.
         * 
         * Calls the necessary pipeline from raw scan to features.
         * 
         **/
        virtual void checkCorrespondence();

};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////
inline FeatureCorner2D::FeatureCorner2D(const CaptureShPtr& _capt_ptr, unsigned int _dim_measurement, const NodeLocation _loc) :
        FeatureBase(_capt_ptr, _dim_measurement, _loc)
{
    //
}

inline FeatureCorner2D::~FeatureCorner2D()
{
    //
}

void FeatureCorner2D::checkCorrespondence()
{
    //To do 
}

#endif //feature_corner_2D_h_
