#ifndef CONSTRAINT_GPS_PSEUDORANGE_H_
#define CONSTRAINT_GPS_PSEUDORANGE_H_

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



class ConstraintGPSPseudorange: public ConstraintSparse<1, 3, 4, 3, 1, 3, 4>
{

public:

    ConstraintGPSPseudorange(FeatureBase* _ftr_ptr, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintSparse<1, 3, 4, 3, 1, 3, 4>(_ftr_ptr, CTR_GPS_PR_3D, _status,
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), // position of the vehicle's frame with respect to the initial pos frame
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(), // orientation of the vehicle's frame
                            _ftr_ptr->getCapturePtr()->getSensorPPtr(), // position of the sensor (gps antenna) with respect to the vehicle frame
                                                                        // orientation of antenna is not needed, because omnidirectional
                            _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr(), //intrinsic parameter  = receiver time bias
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehiclePPtr(), // initial vehicle position (ecef)
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehicleOPtr()  // initial vehicle orientation (ecef)
            ) //TODO attention to the last 2 params and the casts
    {
        sat_position_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getSatPosition();
        pseudorange_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getPseudorange();

        //std::cout << "ConstraintGPSPseudorange()  pr=" << pseudorange_ << "\tsat_pos=(" << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2] << ")" << std::endl;
    }


    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~ConstraintGPSPseudorange()
    {
        //std::cout << "deleting ConstraintGPSPseudorange " << nodeId() << std::endl;
    }



    /*
     * TODO improve naming for more coherence.
     *
     * origin = init_vehicle
     * base = vehicle
     */
    template <typename T>
    bool operator()(const T* const _vehicle_p, const T* const _vehicle_q, const T* const _sensor_p, const T* const _bias, const T* const _init_vehicle_p, const T* const _init_vehicle_q, T* _residual) const
    {
        //std::cout << "OPERATOR()\n";

        Eigen::Matrix<T, 4, 1> sensor_p_ecef; //sensor position with respect to ecef coordinate system
        Eigen::Matrix<T, 4, 1> sensor_p_base(_sensor_p[0], _sensor_p[1], _sensor_p[2], T(1)); //sensor position with respect to the base (the vehicle)
        //TODO padding is with 1, check confirm -------------|

        // TODO queste 3 matrici sono sbagliate!! ricontrolla
//        /*
//         * Origin-to-ECEF conversion matrix
//         */
//        Eigen::Matrix<T, 4, 4> conv_origin_to_ecef;
//
//        Eigen::Quaternion<T> vehicle_init_q(_init_vehicle_q[0], _init_vehicle_q[1], _init_vehicle_q[2], _init_vehicle_q[3]);
//        Eigen::Matrix<T, 3, 3> rot_matr_init = vehicle_init_q.toRotationMatrix();
//        for (int i = 0; i < 3; ++i)
//            for (int j = 0; j < 3; ++j)
//                conv_origin_to_ecef(i, j) = rot_matr_init(i, j);
//
//        for (int i = 0; i < 3; ++i)
//            conv_origin_to_ecef(i, 3) = _init_vehicle_p[i];
//
//        conv_origin_to_ecef(3, 0) = conv_origin_to_ecef(3, 1) = conv_origin_to_ecef(3, 2) = T(0);
//        conv_origin_to_ecef(3, 3) = T(1);
//
//
//
//        /*
//         * Base-to-origin conversion matrix
//         */
//        Eigen::Matrix<T, 4, 4> conv_base_to_origin;
//
//        Eigen::Quaternion<T> vehicle_q(_vehicle_q[0], _vehicle_q[1], _vehicle_q[2], _vehicle_q[3]);
//        Eigen::Matrix<T, 3, 3> rot_matr_vehicle = vehicle_q.toRotationMatrix();
//        for (int i = 0; i < 3; ++i)
//            for (int j = 0; j < 3; ++j)
//                conv_base_to_origin(i, j) = rot_matr_vehicle(i, j);
//
//        for (int i = 0; i < 3; ++i)
//            conv_base_to_origin(i, 3) = _vehicle_p[i];
//
//
//        conv_base_to_origin(3, 0) = conv_base_to_origin(3, 1) = conv_base_to_origin(3, 2) = T(0);
//        conv_base_to_origin(3, 3) = T(1);
//
//        /*
//         * transformation from base reference to ecef
//         *
//         * for trying the solver, this two quantities are assumed the same:
//         *      sensor position with respect to ecef coordinate system
//         *      sensor position with respect to the base (the vehicle)
//         *
//         * means that the sensor is located exactly in the origin of the vehicle
//         */
        // TODO queste 3 matrici sono sbagliate!! ricontrolla
        sensor_p_ecef = /* conv_origin_to_ecef * conv_base_to_origin * */ sensor_p_base;



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

#endif //CONSTRAINT_GPS_PSEUDORANGE_H_
