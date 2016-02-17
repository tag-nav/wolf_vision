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
//        std::cout << "_vehicle_p: " << _vehicle_p[0] << ", " << _vehicle_p[1] << std::endl;
//        std::cout << "_vehicle_o: " << _vehicle_o[0] << std::endl;
//        std::cout << "_sensor_p: " << _sensor_p[0] << ", " << _sensor_p[1] << ", " << _sensor_p[2] << std::endl;

        Eigen::Matrix<T, 3, 1> sensor_p_base(_sensor_p[0], _sensor_p[1], _sensor_p[2]); //sensor position with respect to the base (the vehicle)
        std::cout << "sensor_p_base: " << sensor_p_base[0] << ", " << sensor_p_base[1] << ", " << sensor_p_base[2] << std::endl;

        Eigen::Matrix<T, 3, 1> vehicle_p_origin(_vehicle_p[0], _vehicle_p[1], T(0));
        std::cout << "vehicle_p_origin: " << vehicle_p_origin[0] << ", " << vehicle_p_origin[1] << ", " << vehicle_p_origin[2] << std::endl;
        /*
         * Base-to-origin transform matrix
         */
        Eigen::Matrix<T, 3, 3> transform_base_to_origin;
        transform_base_to_origin(0, 0) = T(cos(_vehicle_o[0]));
        transform_base_to_origin(0, 1) = T(-sin(_vehicle_o[0]));
        transform_base_to_origin(0, 2) = T(0);
        transform_base_to_origin(1, 0) = T(sin(_vehicle_o[0]));
        transform_base_to_origin(1, 1) = T(cos(_vehicle_o[0]));
        transform_base_to_origin(1, 2) = T(0);
        transform_base_to_origin(2, 0) = T(0);
        transform_base_to_origin(2, 1) = T(0);
        transform_base_to_origin(2, 2) = T(1);

        Eigen::Matrix<T, 3, 1> sensor_p_origin; // sensor position with respect to origin frame (initial frame of the experiment)
        sensor_p_origin = transform_base_to_origin * sensor_p_base + vehicle_p_origin;

        std::cout << "RESULT:  ";
        std::cout << "sensor_p_origin: " << sensor_p_origin[0] << ", " << sensor_p_origin[1] << ", " << sensor_p_origin[2] << std::endl;


        /*
         * Origin-to-ECEF transform matrix
         */
        Eigen::Matrix<T, 4, 4> transform_origin_to_ecef;
        //TODO by andreu


        //result I want to find:
        Eigen::Matrix<T, 3, 1> sensor_p_ecef; //sensor position with respect to ecef coordinate system
        //TODO to be filled
        sensor_p_ecef[0] = sensor_p_ecef[1] = sensor_p_ecef[2] = T(0);
//        sensor_p_ecef = depends on transform_origin_to_ecef and sensor_p_origin



        std::cout << "sensor_p_ecef: " << sensor_p_ecef[0] << ", " << sensor_p_ecef[1] << ", " << sensor_p_ecef[2] << std::endl;



        //il codice qui sotto è quello vecchio, adattato in modo da usare la posizione del sensore rispetto a ecef, calcolata qui sopra
        T square_sum = T(0);
        for (int i = 0; i < 3; ++i)
        {
            square_sum += pow(sensor_p_ecef[i] - T(sat_position_[i]) , 2);
        }
        T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0) ;

        //     error = (expected measurement)       - (actual measurement)
        _residual[0] = (distance + _bias[0]*T(LIGHT_SPEED)) - (pseudorange_);


        /* TODO importante
         * credo che il residuo sia la differenza delle misure, NORMALIZZATA PER LA COVARIANZA
         */

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
