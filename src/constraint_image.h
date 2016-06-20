#ifndef CONSTRAINT_IMAGE_H
#define CONSTRAINT_IMAGE_H

//Wolf includes
#include "constraint_sparse.h"
#include "landmark_point_3d.h"
#include "pinholeTools.h"
#include "feature_point_image.h"

namespace wolf {

class ConstraintImage : public ConstraintSparse<2, 3, 3, 3>
{
    private:
        Eigen::Vector4s k_parameters_;
        Eigen::Vector2s distortion_;
        Eigen::Vector2s feature_position_;

    public:
        static const unsigned int N_BLOCKS = 3;

        ConstraintImage(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, LandmarkBase* _landmark_ptr, Eigen::Vector4s _k_parameters, Eigen::Vector2s _distortion_,
                        bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 3, 3, 3>(_ftr_ptr, CTR_EPIPOLAR, _landmark_ptr, _apply_loss_function, _status,
                                             _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _landmark_ptr->getPPtr()),
                k_parameters_(_k_parameters), distortion_(_distortion_)
        {
            setType("IMAGE");
            feature_position_[0] = ((FeaturePointImage*)_ftr_ptr)->getKeypoint().pt.x;
            feature_position_[1] = ((FeaturePointImage*)_ftr_ptr)->getKeypoint().pt.y;

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
        bool operator ()(const T* const _probot, const T* const _orobot, const T* const _plmk, T* _residuals) const;

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
inline bool ConstraintImage::operator ()(const T* const _probot, const T* const _orobot, const T* const _plmk,
                                         T* _residuals) const
{
    // Do the magic here

    // error = (expected measurement - getMeasurement() ) * getSquareRootInformation()
    // _residuals = (projectionOfTheLandmark - featurePosition) * getSquareRootInformation()

    Eigen::Matrix<T,3,1> landmark_position = ((LandmarkPoint3D) *_plmk).getPosition();
    Eigen::Matrix<T,4,1> k_params = k_parameters_;
    Eigen::Matrix<T,2,1> distortion = distortion_;

    Eigen::Matrix<T,2,1> projectionOfLandmark;
    projectionOfLandmark = pinhole::projectPoint( k_params, distortion, landmark_position);


    std::cout << "==================== CONSTRAINT IMAGE ========================" << std::endl;
    std::cout << "projected landmark | x: " << projectionOfLandmark[0] << " | y: " << projectionOfLandmark[1] << std::endl;

    Eigen::Matrix<T,2,1> feature_pos = feature_position_;

    _residuals[0] = (projectionOfLandmark[0] - feature_pos[0]) * getMeasurementSquareRootInformation()(0, 0);
    _residuals[1] = (projectionOfLandmark[1] - feature_pos[1]) * getMeasurementSquareRootInformation()(1, 1);

    return true;
}
} // namespace wolf

#endif // CONSTRAINT_IMAGE_H
