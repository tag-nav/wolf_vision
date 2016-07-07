#ifndef CONSTRAINT_IMAGE_H
#define CONSTRAINT_IMAGE_H

//Wolf includes
#include "constraint_sparse.h"
#include "landmark_AHP.h"
#include "sensor_camera.h"
#include "pinholeTools.h"
#include "feature_point_image.h"

namespace wolf {

class ConstraintImage : public ConstraintSparse<2, 3, 4, 3, 4, 4>
{
    protected:
        Eigen::Vector4s intrinsics_;
        Eigen::Vector3s extrinsics_p_;
        Eigen::Vector4s extrinsics_o_;

    public:
        static const unsigned int N_BLOCKS = 5; //TODO: Prueba a comentarlo

        ConstraintImage(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, LandmarkAHP* _landmark_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 3, 4, 3, 4, 4>(_ftr_ptr, CTR_EPIPOLAR, _landmark_ptr, _apply_loss_function, _status,
                                             _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _landmark_ptr->getAnchorFrame()->getPPtr()
                                                   ,_landmark_ptr->getAnchorFrame()->getOPtr(),_landmark_ptr->getPPtr()),
                intrinsics_(_ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr()->getVector()),
                extrinsics_p_(_ftr_ptr->getCapturePtr()->getSensorPPtr()->getVector()),
                extrinsics_o_(_ftr_ptr->getCapturePtr()->getSensorOPtr()->getVector())
        {
            setType("IMAGE");

        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintImage()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _probot, const T* const _orobot, const T* const _lmk, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

//    public:
//        static wolf::ConstraintBase* create(FeatureBase* _feature_ptr, //
//                                            NodeBase* _correspondant_ptr)
//        {
//            return new ConstraintImage(_feature_ptr, (FrameBase*)_correspondant_ptr);
//        }

};
template<typename T>
inline bool ConstraintImage::operator ()(const T* const _probot, const T* const _orobot, const T* const _lmk,
                                         T* _residuals) const
{
    // Do the magic here
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > probotmap(_probot);
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residualsmap(_residuals);

    // error = (expected measurement - getMeasurement() ) * getSquareRootInformation()
    // _residuals = (projectionOfTheLandmark - featurePosition) * getSquareRootInformation()

//    Eigen::Matrix<T,Eigen::Dynamic,1> distortion = ((SensorCamera) *_sensor).getDistortionVector();

    Eigen::Matrix<T,4,1> k_params = intrinsics_.cast<T>();
    Eigen::Matrix<T,3,1> rTc = ((SensorCamera) *_sensor).getPPtr()->getVector();
    Eigen::Matrix<T,4,1> sensor_orientation = ((SensorCamera) *_sensor).getOPtr()->getVector();

    Eigen::Matrix<T,3,1> wTr = _probot;
    Eigen::Matrix<T,4,1> frame_orientation = _orobot; //TODO: Pasar a matriz de rotacion

//    Eigen::Matrix<T,3,1> rTc = ((LandmarkAHP) *_lmk).getAnchorFrame().getPPtr()->getVector();
//    Eigen::Matrix<T,4,1> sensor_ = ((LandmarkAHP) *_lmk).getAnchorFrame().getOPtr()->getVector();  //TODO: Pasar a matriz de rotacion

    Eigen::Matrix<T,3,3> K;
    K(0,0) = k_params(0);
    K(0,1) = 0;
    K(0,2) = k_params(2);
    K(1,0) = 0;
    K(1,1) = k_params(1);
    K(1,2) = k_params(3);
    K(2,0) = 0;
    K(2,1) = 0;
    K(2,2) = 0;

    Eigen::Matrix<T,4,1> vector = ((LandmarkAHP) *_lmk).getPosition()->getVector();
    Eigen::Matrix<T,3,1> m;
    m(0) = vector(0);
    m(1) = vector(1);
    m(2) = vector(2);

    /* making the rotations manually now */
    Eigen::Matrix<T,3,3> wRr;
    wRr(0,0) = pow(frame_orientation(3),2) + pow(frame_orientation(0),2) - pow(frame_orientation(1),2) - pow(frame_orientation(2),2);
    wRr(0,1) = 2*(frame_orientation(0)*frame_orientation(1) + frame_orientation(3)*frame_orientation(2));
    wRr(0,2) = 2*(frame_orientation(0)*frame_orientation(2) - frame_orientation(3)*frame_orientation(1));;
    wRr(1,0) = 2*(frame_orientation(0)*frame_orientation(1) - frame_orientation(3)*frame_orientation(2));;
    wRr(1,1) = pow(frame_orientation(3),2) - pow(frame_orientation(0),2) + pow(frame_orientation(1),2) - pow(frame_orientation(2),2);
    wRr(1,2) = 2*(frame_orientation(1)*frame_orientation(2) + frame_orientation(3)*frame_orientation(0));
    wRr(2,0) = 2*(frame_orientation(0)*frame_orientation(2) + frame_orientation(3)*frame_orientation(1));
    wRr(2,1) = 2*(frame_orientation(1)*frame_orientation(2) - frame_orientation(3)*frame_orientation(0));
    wRr(2,2) = pow(frame_orientation(3),2) - pow(frame_orientation(0),2) - pow(frame_orientation(1),2) + pow(frame_orientation(2),2);

    Eigen::Matrix<T,3,3> rRc;
    rRc(0,0) = pow(sensor_orientation(3),2) + pow(sensor_orientation(0),2) - pow(sensor_orientation(1),2) - pow(sensor_orientation(2),2);
    rRc(0,1) = 2*(sensor_orientation(0)*sensor_orientation(1) + sensor_orientation(3)*sensor_orientation(2));
    rRc(0,2) = 2*(sensor_orientation(0)*sensor_orientation(2) - sensor_orientation(3)*sensor_orientation(1));;
    rRc(1,0) = 2*(sensor_orientation(0)*sensor_orientation(1) - sensor_orientation(3)*sensor_orientation(2));;
    rRc(1,1) = pow(sensor_orientation(3),2) - pow(sensor_orientation(0),2) + pow(sensor_orientation(1),2) - pow(sensor_orientation(2),2);
    rRc(1,2) = 2*(sensor_orientation(1)*sensor_orientation(2) + sensor_orientation(3)*sensor_orientation(0));
    rRc(2,0) = 2*(sensor_orientation(0)*sensor_orientation(2) + sensor_orientation(3)*sensor_orientation(1));
    rRc(2,1) = 2*(sensor_orientation(1)*sensor_orientation(2) - sensor_orientation(3)*sensor_orientation(0));
    rRc(2,2) = pow(sensor_orientation(3),2) - pow(sensor_orientation(0),2) - pow(sensor_orientation(1),2) + pow(sensor_orientation(2),2);
    /* end making the rotations */


    Eigen::Matrix<T,3,1> wTc0 = wRr*rTc + wTr;      // wRr*rTc + wTr*1
    Eigen::Matrix<T,4,1> wRc1 = wRr*rRc;            // wRr*rRc + wTr*0
    Eigen::Matrix<T,3,1> wTc1 = wRr*rTc + wTr;      // wRr*rTc + wTr*1


    Eigen::Matrix<T,3,1> PI = K * wRc1.transpose() * (vector(3) * (wTc0 - wTc1) + m);

    Eigen::Matrix<T,2,1> PI_12;
    PI_12(0) = PI(0);
    PI_12(1) = PI(1);

    Eigen::Matrix<T,2,1> landmark_estimation = PI_12/PI(3);


    Eigen::Matrix<T,2,1> feature_pos = getMeasurement().cast<T>();

    _residuals[0] = (landmark_estimation[0] - feature_pos[0]) * getMeasurementSquareRootInformation()(0, 0);
    _residuals[1] = (landmark_estimation[1] - feature_pos[1]) * getMeasurementSquareRootInformation()(1, 1);

    std::cout << "\n\tRESIDUALS: \n" << _residuals[0] << "\n" << _residuals[1] << std::endl;

    return true;
}
} // namespace wolf

#endif // CONSTRAINT_IMAGE_H
