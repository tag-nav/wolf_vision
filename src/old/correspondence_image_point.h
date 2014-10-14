/**
 * \file correspondence_image_point.h
 *
 *  Created on: 04/08/2014
 *     \author: jsola
 */

#ifndef CORRESPONDENCE_IMAGE_POINT_H_
#define CORRESPONDENCE_IMAGE_POINT_H_

#include "correspondence_base.h"


//template<class StateT>
class CorrespondenceImagePoint : public CorrespondenceBase//<StateT>
{
    public:

        CorrespondenceImagePoint() :
                CorrespondenceBase(3, 2)//, // Expectation size 3, error size 2.
        {
            //
        }

        CorrespondenceImagePoint(const NodeShrPtr & _un_ptr) :
                CorrespondenceBase(_un_ptr, 3, 2)//, // Expectation size 3, error size 2.
        {
            //
        }

        virtual ~CorrespondenceImagePoint()
        {
            //
        }


        /**
         * \brief Compute expectation.
         */
        virtual void computeExpectation(){

        }
        virtual void computeError(){
            // TODO: Apply the epipolar constraint

//            feature_ptr_.lock()->measurement_ << 1.0, 2.0;
//            error_ = feature_ptr_.lock()->measurement_ - VectorXs::Constant(2,getId()) ;

            error_ = VectorXs::Constant(2,nodeId()) ;
        }

        /** \brief Prints node label
         *
         * Prints node label
         *
         **/
        virtual void printLabel(ostream & _ost = cout) const
        {
            _ost <<"CORRESPONDENCE: image point";
        }


};
#endif /* CORRESPONDENCE_IMAGE_POINT_H_ */
