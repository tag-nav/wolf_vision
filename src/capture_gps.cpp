//
// Created by ptirindelli on 4/12/15.
//

#include "capture_gps.h"

using namespace std;

//CaptureGPS(const TimeStamp & _ts, SensorBase* _sensor_ptr, const std::vector<float>& _raw_data);
//
CaptureGPS::CaptureGPS(const TimeStamp & _ts,
                       SensorBase* _sensor_ptr,
                       const std::vector<float>& _raw_data) :
        CaptureBase(_ts, _sensor_ptr),
        raw_data_(_raw_data)
{
    cout << "Data: ";
    for (size_t i = 0; i<_raw_data.size(); ++i)
    {
        cout << _raw_data[i] << " ";
    }
    cout << endl;

}

CaptureGPS::~CaptureGPS()
{
    //std::cout << "deleting CaptureGPS " << nodeId() << std::endl;
}

Eigen::VectorXs CaptureGPS::computePrior(const TimeStamp &_now) const
{
    //TODO implementata a caso, solo perche' e' virtual pura e altrimenti non compila
    return Eigen::Vector3s(1, 2, 3);
}

void CaptureGPS::processCapture() {
    // TODO guarda sto metodo:
    CaptureBase::processCapture();

    //TODO qui dal raw data devi estrarre tutti i satelliti
    //     e aggiungerli come feature separate

    // EXTRACT AND ADD FEATURES

    for(size_t i = 0; i < raw_data_.size(); ++i)
    {
        addFeature(new FeatureGPSPseudorange(raw_data_[i]));
    }

    //TODO ADD CONSTRAINT
    //getFeatureListPtr()->front()->addConstraintFrom(new ConstraintGPS2D(getFeatureListPtr()->front(), getFramePtr()));

}
