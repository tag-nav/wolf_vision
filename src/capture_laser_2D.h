
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//wolf forward declarations
namespace wolf{
class SensorLaser2D;
}

//wolf includes
#include "capture_base.h"

//laserscanutils includes
#include "laser_scan_utils/laser_scan.h"

namespace wolf {

class CaptureLaser2D : public CaptureBase
{
    public:
        /** \brief Constructor with ranges
         **/
        CaptureLaser2D(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const std::vector<float>& _ranges);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         **/        
        virtual ~CaptureLaser2D();
        
        laserscanutils::LaserScan& getScan();

        void setSensorPtr(const SensorBasePtr sensor_ptr);

    private:
        SensorLaser2D* laser_ptr_; //specific pointer to sensor laser 2D object
        laserscanutils::LaserScan scan_;

};

inline laserscanutils::LaserScan& CaptureLaser2D::getScan()
{
    return scan_;
}

inline void CaptureLaser2D::setSensorPtr(const SensorBasePtr sensor_ptr)
{
  CaptureBase::setSensorPtr(sensor_ptr);
  laser_ptr_ = (SensorLaser2D*)(sensor_ptr);
}

} // namespace wolf

#endif /* CAPTURE_LASER_2D_H_ */
