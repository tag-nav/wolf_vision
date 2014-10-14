/**
 * \file capture_image.h
 *
 *  Created on: 15/08/2014
 *     \author: jsola
 */

#ifndef CAPTURE_IMAGE_H_
#define CAPTURE_IMAGE_H_

// Include base class
#include "capture.h"


/**
 * \brief Image capture, generally from a projective, pin-hole camera.
 */
class CaptureImage : public Capture
{
    public:

        CaptureImage(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, NodeLocation _loc = MID);

        virtual ~CaptureImage();

};


////////////////////////////////
// IMPLEMENTATION
////////////////////////////////

CaptureImage::CaptureImage(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, NodeLocation _loc) :
        Capture(_frm_ptr, _sen_ptr, _loc)
{
    //
}

CaptureImage::~CaptureImage()
{
    //
}


#endif /* CAPTURE_IMAGE_H_ */
