#include "sensor_camera.h"


namespace wolf {

/**
 *
 * Test for the camera sensor
 *
 **/

SensorCamera::SensorCamera(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr,
                           int _img_width, int _img_height) :
        SensorBase(SEN_CAMERA, _p_ptr, _o_ptr, _intr_ptr, 2),
        img_width_(_img_width),
        img_height_(_img_height)
{
    setType("CAMERA");
    //
}

SensorCamera::~SensorCamera()
{
    //
}

} // namespace wolf
