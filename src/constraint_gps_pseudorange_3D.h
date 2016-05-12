#ifndef CONSTRAINT_GPS_PSEUDORANGE_3D_H_
#define CONSTRAINT_GPS_PSEUDORANGE_3D_H_

#define LIGHT_SPEED 299792458

//Wolf includes
#include "sensor_gps.h"
#include "feature_gps_pseudorange.h"
#include "constraint_sparse.h"

namespace wolf {

/*
 * NB:
 * FROM THIS CLASS AND ALL THE CLASS INCLUDED, THE LIBRARY RAW_GPS_UTILS
 * MUST NOT BE REACHABLE!
 * OTHERWISE WOLF WON'T COMPILE WITHOUT INSTALLING THIS LIBRARY.
 *
 * TODO maybe is no more true
 */

class ConstraintGPSPseudorange3D: public ConstraintSparse<1, 3, 4, 3, 1, 3, 4>
{

public:

    ConstraintGPSPseudorange3D(FeatureBase* _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintSparse<1, 3, 4, 3, 1, 3, 4>(_ftr_ptr, CTR_GPS_PR_3D, _apply_loss_function, _status,
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), // position of the vehicle's frame with respect to map frame
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(), // orientation of the vehicle's frame wrt map frame
                            _ftr_ptr->getCapturePtr()->getSensorPPtr(), // position of the sensor (gps antenna) with respect to base frame
                                                                        // orientation of antenna is not needed, because omnidirectional
                            _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr(), //intrinsic parameter  = receiver time bias
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getMapPPtr(), // initial vehicle position wrt ecef frame
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getMapOPtr())  // initial vehicle orientation wrt ecef frame
    {
        setType("GPS PR 3D");
        sat_position_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getSatPosition();
        pseudorange_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getPseudorange();

        //std::cout << "ConstraintGPSPseudorange3D()  pr=" << pseudorange_ << "\tsat_pos=(" << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2] << ")" << std::endl;
    }


    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~ConstraintGPSPseudorange3D()
    {
        //std::cout << "deleting ConstraintGPSPseudorange3D " << nodeId() << std::endl;
    }



    template<typename T>
    bool operator ()(const T* const _vehicle_p, const T* const _vehicle_o, const T* const _sensor_p,
                     const T* const _bias, const T* const _init_vehicle_p, const T* const _init_vehicle_o,
                     T* _residual) const;


    /** \brief Returns the jacobians computation method
     *
     * Returns the jacobians computation method
     *
     **/
    virtual JacobianMethod getJacobianMethod() const
    {
        return JAC_AUTO;
    }

protected:
    Eigen::Vector3s sat_position_;
    Scalar pseudorange_;


public:
    static wolf::ConstraintBase* create(FeatureBase* _feature_ptr, //
                                        NodeBase* _correspondant_ptr = nullptr)
    {
        return new ConstraintGPSPseudorange3D(_feature_ptr);
    }


};

/*
 * naming convention for transformation matrix:
 * T_a_b is "transformation from b to a"
 * To transform a point from b to a:    p_a = T_a_b * P_b
 * T_a_b also means "the pose of b expressed in frame a"
 */

template<typename T>
inline bool ConstraintGPSPseudorange3D::operator ()(const T* const _vehicle_p, const T* const _vehicle_o,
                                                    const T* const _sensor_p, const T* const _bias,
                                                    const T* const _map_p, const T* const _map_o,
                                                    T* _residual) const
{
//    std::cout << "OPERATOR()\n";
//    std::cout << "_map_p: " << _map_p[0] << ", " << _map_p[1] << ", " << _map_p[2] << std::endl;
//    std::cout << "_map_o: " << _map_o[0] << ", " << _map_o[1] << ", " << _map_o[2] << ", " << _map_o[3] << std::endl;
//    std::cout << "_vehicle_p: " << _vehicle_p[0] << ", " << _vehicle_p[1] << ", " << _vehicle_p[2] << std::endl;
//    std::cout << "_vehicle_o: " << _vehicle_o[0] << ", " << _vehicle_o[1] << ", " << _vehicle_o[2] << ", " << _vehicle_o[3] << std::endl;
//    std::cout << "_sensor_p: " << _sensor_p[0] << ", " << _sensor_p[1] << ", " << _sensor_p[2] << std::endl;
    Eigen::Matrix<T, 4, 1> sensor_p_ecef; //sensor position with respect to ecef coordinate system
    Eigen::Matrix<T, 4, 1> sensor_p_base(_sensor_p[0], _sensor_p[1], _sensor_p[2], T(1)); //sensor position with respect to the base (the vehicle)

    /*
     * map-to-ECEF transformation matrix: To transform a point from map to ecef
     */
    Eigen::Matrix<T, 4, 4> T_ecef_map = Eigen::Matrix<T, 4, 4>::Identity();
    Eigen::Quaternion<T> map_o_quat(_map_o[0], _map_o[1], _map_o[2], _map_o[3]);
    Eigen::Matrix<T, 3, 3> rot_matr_init = map_o_quat.toRotationMatrix();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            T_ecef_map(i, j) = rot_matr_init(i, j);
    for (int i = 0; i < 3; ++i)
        T_ecef_map(i, 3) = _map_p[i];

    /*
     * Base-to-map transformation matrix: To transform a point from base to map
     */
    Eigen::Matrix<T, 4, 4> T_map_base = Eigen::Matrix<T, 4, 4>::Identity();
    Eigen::Quaternion<T> vehicle_o_quat(_vehicle_o[0], _vehicle_o[1], _vehicle_o[2], _vehicle_o[3]);
    Eigen::Matrix<T, 3, 3> rot_matr_vehicle = vehicle_o_quat.toRotationMatrix();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            T_map_base(i, j) = rot_matr_vehicle(i, j);
    for (int i = 0; i < 3; ++i)
        T_map_base(i, 3) = _vehicle_p[i];


    /*
     * Compute sensor_p wrt ECEF
     */
    sensor_p_ecef = T_ecef_map * T_map_base * sensor_p_base;
    //std::cout << "sensor_p_ecef: " << sensor_p_ecef[0] << ", " << sensor_p_ecef[1] << ", " << sensor_p_ecef[2] << std::endl;

    /*
     * Compute residual
     */
    T square_sum = T(0);
    for (int i = 0; i < 3; ++i)
        square_sum += pow(sensor_p_ecef[i] - T(sat_position_[i]), 2);

    T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0);
    //     error = (expected measurement)       - (actual measurement)
    _residual[0] = (distance + _bias[0] * T(LIGHT_SPEED)) - (pseudorange_);

    // Normalize by covariance
    _residual[0] = _residual[0] / T(sqrt(getMeasurementCovariance()(0, 0)));

    return true;
}

} // namespace wolf

#endif //CONSTRAINT_GPS_PSEUDORANGE_3D_H_
