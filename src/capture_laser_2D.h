
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//wolf forward declarations

//wolf includes
#include "capture_base.h"

namespace wolf {

class CaptureLaser2D : public CaptureBase
{
    public:
        /** \brief Constructor with ranges
         **/
        CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const std::vector<float>& _ranges);

        /** \brief Constructor with ranges and intensities
         **/
        CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const std::vector<float>& _ranges, const std::vector<float>& _intensities);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         **/        
        virtual ~CaptureLaser2D();
        
        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const;

        SensorLaser2D const * getLaserPtr() const;
        const std::vector<float>& getRanges() const;
        const std::vector<float>& getIntensities() const;

    private:
        SensorLaser2D* laser_ptr_; //specific pointer to sensor laser 2D object
        std::vector<float> ranges_; // ranges vector. Type float to match ROS LaserScan message
        std::vector<float> intensities_; // intensities vector. Type float to match ROS LaserScan message

};

inline const std::vector<float>& CaptureLaser2D::getIntensities() const
{
    return intensities_;
}

inline SensorLaser2D const * CaptureLaser2D::getLaserPtr() const
{
    return laser_ptr_;
}

inline const std::vector<float>& CaptureLaser2D::getRanges() const
{
    return ranges_;
}

} // namespace wolf

#endif /* CAPTURE_LASER_2D_H_ */
