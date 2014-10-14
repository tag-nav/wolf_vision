/*
 * observation_image_point.h
 *
 *  Created on: Jun 4, 2014
 *      \author: jsola
 */

#ifndef OBSERVATION_IMAGE_POINT_H_
#define OBSERVATION_IMAGE_POINT_H_

// wolf
#include "observation_base.h"
#include "gaussian.h"
#include "sensor_base.h"

using namespace std;
using namespace Eigen;

/**
 * Observation of a point feature in an image
 */
class ObservationImagePoint : public ObservationBase<Gaussian>
{
    public:
        static const unsigned int SIZE_ = 2;

        Map<Vector4s> intrinsic_;

        ObservationImagePoint(SensorBase & _sensor);
        virtual ~ObservationImagePoint();

        VectorXs& computeExpectation(StateRootBase & _state);
        VectorXs& computeInnovation(StateRootBase & _state);
        scalar_t computeCost(StateRootBase & _state);

    protected:
        Vector2s computeExpectation(StatePose& _state, StatePose& _sensor_pose, Vector4s& _intrinsic, Vector3s& _point);

        Vector2s pinHoleProject(StatePose& _pose, Vector4s _k, Vector3s _point);

};
#endif /* OBSERVATION_IMAGE_POINT_H_ */
