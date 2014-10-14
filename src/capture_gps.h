/**
 * \file capture_gps.h
 *
 *  Created on: 17/08/2014
 *     \author: jsola
 */

#ifndef CAPTURE_GPS_H_
#define CAPTURE_GPS_H_

#include "capture.h"

class CaptureGPS : public Capture
{
    public:
        CaptureGPS();
        virtual ~CaptureGPS();
};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////


inline CaptureGPS::CaptureGPS()
{
    // TODO Auto-generated constructor stub
}

inline CaptureGPS::~CaptureGPS()
{
    //
}

#endif /* CAPTURE_GPS_H_ */
