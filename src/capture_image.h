#ifndef CAPTURE_IMAGE_H
#define CAPTURE_IMAGE_H

//std includes
//

//Wolf includes
#include "capture_base.h"
#include "feature_point.h"
#include "sensor_camera.h"

//class CaptureImage
class CaptureImage : public CaptureBase
{
    protected:
        Eigen::MatrixXs data_; ///< Raw data.

    public:
        CaptureImage(const TimeStamp& _ts, SensorCamera* _camera_ptr, const Eigen::MatrixXs& _data, int _img_width, int _img_height);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureImage();

        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const;

        //virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;

        //Eigen::MatrixXs data_; ///< Raw data.
};

#endif // CAPTURE_IMAGE_H
