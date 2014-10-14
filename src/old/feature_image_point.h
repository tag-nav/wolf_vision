/**
 * \file feature_image_point.h
 *
 *  Created on: 04/08/2014
 *     \author: jsola
 */

#ifndef FEATURE_IMAGE_POINT_H_
#define FEATURE_IMAGE_POINT_H_

#include "feature_base.h"

class FeatureImagePoint : public FeatureBase
{
    public:
        FeatureImagePoint() :
                FeatureBase(2)
        {
        }
        FeatureImagePoint(const NodeShrPtr & _un_ptr) :
                FeatureBase(_un_ptr, 2)
        {
        }
        virtual ~FeatureImagePoint()
        {
            //
        }

        /** \brief Prints node label
         *
         * Prints node label
         *
         **/
        virtual void printLabel(ostream & _ost = cout) const
        {
            _ost <<"FEATURE: image point";
        }

};
#endif /* FEATURE_IMAGE_POINT_H_ */
