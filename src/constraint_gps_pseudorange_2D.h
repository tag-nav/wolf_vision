#ifndef CONSTRAINT_GPS_PSEUDORANGE_2D_H_
#define CONSTRAINT_GPS_PSEUDORANGE_2D_H_

#define LIGHT_SPEED 299792458

//Wolf includes
#include "sensor_gps.h"
#include "feature_gps_pseudorange.h"
#include "constraint_sparse.h"
/*
 * NB:
 * FROM THIS CLASS AND ALL THE CLASS INCLUDED, THE LIBRARY RAW_GPS_UTILS
 * MUST NOT BE REACHABLE!
 * OTHERWISE WOLF WON'T COMPILE WITHOUT INSTALLING THIS LIBRARY.
 *
 */



class ConstraintGPSPseudorange2D: public ConstraintSparse<1, 2, 1, 3, 1, 3, 1>
{

public:

    ConstraintGPSPseudorange2D(FeatureBase* _ftr_ptr, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintSparse<1, 2, 1, 3, 1, 3, 1>(_ftr_ptr, CTR_GPS_PR_2D, _status,
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), // position of the vehicle's frame with respect to the initial pos frame
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(), // orientation of the vehicle's frame
                            _ftr_ptr->getCapturePtr()->getSensorPPtr(), // position of the sensor (gps antenna) with respect to the vehicle frame
                                                                        // orientation of antenna is not needed, because omnidirectional
                            _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr(), //intrinsic parameter  = receiver time bias
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehiclePPtr(), // initial vehicle position (ecef)
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehicleOPtr()  // initial vehicle orientation (ecef)
            )
    {
        sat_position_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getSatPosition();
        pseudorange_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getPseudorange();

        //std::cout << "ConstraintGPSPseudorange2D()  pr=" << pseudorange_ << "\tsat_pos=(" << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2] << ")" << std::endl;
    }


    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~ConstraintGPSPseudorange2D()
    {
        //std::cout << "deleting ConstraintGPSPseudorange2D " << nodeId() << std::endl;
    }



    /*
     * TODO improve naming for more coherence.
     *
     * origin = init_vehicle
     * base = vehicle
     */
    template <typename T>
    bool operator()(const T* const _vehicle_p, const T* const _vehicle_o, const T* const _sensor_p, const T* const _bias, const T* const _init_vehicle_p, const T* const _init_vehicle_o, T* _residual) const
    {
//        std::cout << "OPERATOR()\n";
//        std::cout << "_init_vehicle_p: " << _init_vehicle_p[0] << ", " << _init_vehicle_p[1] << ", " << _init_vehicle_p[2] << std::endl;
//        std::cout << "_init_vehicle_o: " << _init_vehicle_o[0] << ", " << _init_vehicle_o[1] << ", " << _init_vehicle_o[2] << ", " << _init_vehicle_o[3] << std::endl;
//        std::cout << "_vehicle_p: " << _vehicle_p[0] << ", " << _vehicle_p[1] << ", " << _vehicle_p[2] << std::endl;
//        std::cout << "_vehicle_o: " << _vehicle_o[0] << ", " << _vehicle_o[1] << ", " << _vehicle_o[2] << ", " << _vehicle_o[3] << std::endl;
//        std::cout << "_sensor_p: " << _sensor_p[0] << ", " << _sensor_p[1] << ", " << _sensor_p[2] << std::endl;

        //TODO
        //TODO
        //TODO
        _residual[0] = T(3);//TODO

        return true;
    }


    /** \brief Returns the jacobians computation method
     *
     * Returns the jacobians computation method
     *
     **/
    virtual JacobianMethod getJacobianMethod() const
    {
        return AUTO;
    }

protected:
    Eigen::Vector3s sat_position_;
    WolfScalar pseudorange_;

};

#endif //CONSTRAINT_GPS_PSEUDORANGE_2D_H_
