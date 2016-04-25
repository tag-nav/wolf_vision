#ifndef PROCESSORORB_H
#define PROCESSORORB_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"

//#include ORBextractor.h

namespace wolf {

class ProcessorORB : public ProcessorBase
{
    protected:
        ProcessorSensorCamera* sensor_cam_ptr_; //specific pointer to sensor camera object
        CaptureImage* capture_img_ptr_; //specific pointer to capture image object;
        //add an ORBextractor

    public:
        ProcessorORB();
        virtual ~ProcessorORB();

        virtual void extractFeatures(CaptureBase *_capture_ptr);
        virtual void establishConstraints(CaptureBase *_capture_ptr);

    protected:
        virtual void preProcess(){}
        virtual void postProcess(){}


};

} // namespace wolf

#endif // PROCESSORORB_H
