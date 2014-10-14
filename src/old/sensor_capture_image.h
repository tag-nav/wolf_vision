/**
 * \file sensor_capture_image.h
 *
 *  Created on: 04/08/2014
 *     \author: jsola
 */

#ifndef SENSOR_CAPTURE_IMAGE_H_
#define SENSOR_CAPTURE_IMAGE_H_

#include "sensor_capture_base.h"

class SensorCaptureImage : public SensorCaptureBase
{
    public:
        SensorCaptureImage(const NodeShrPtr & _un_ptr) :
            SensorCaptureBase(_un_ptr)
        {
            //
        }
        virtual ~SensorCaptureImage()
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
            _ost <<"CAPTURE: image";
        }

};

#endif /* SENSOR_CAPTURE_IMAGE_H_ */
