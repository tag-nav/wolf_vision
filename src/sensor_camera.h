#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

//wolf includes
#include "sensor_base.h"

/**
 *
 * Test for the camera sensor
 *
 **/

class SensorCamera : public SensorBase
{
    public:
    /** \brief Constructor with arguments
     *
     * Constructor with arguments
     * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
     * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
     * \param _intr_ptr contains intrinsic values for the camera
     * \param _disp_noise_factor displacement noise factor
     * \param _rot_noise_factor rotation noise factor
     *
     **/
    SensorCamera(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, int _img_width, int _img_height);

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~SensorCamera();

    int img_width_;
    int img_height_;

};

#endif // SENSOR_CAMERA_H
