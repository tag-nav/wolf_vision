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

void CaptureGPS::processCapture()
{
    cout << "CaptureGPS::processCapture()... processing capture" << std::endl;

    //TODO qui dal raw data devi estrarre tutti i satelliti
    //     e aggiungerli come feature separate

    // EXTRACT AND ADD FEATURES
    for(size_t i = 0; i < raw_data_.size(); ++i)
    {
        addFeature(new FeatureGPSPseudorange(raw_data_[i]));
    }

    // ADD CONSTRAINT
    //TODO sarebbe cosi', usando i frame. ConstraintGPSPseudorange* constr = new ConstraintGPSPseudorange(getFeatureListPtr()->front(), getFramePtr());
    ConstraintGPSPseudorange* constr = new ConstraintGPSPseudorange(getFeatureListPtr()->front());
    //TODO devo aggiungere una constraint per ogni feature!!! vedi se farlo qui o altrove

    // TODO era cosi', ma fallisce un assert perche' non ho fatto partire wolf (credo)
    // getFeatureListPtr()->front()->addConstraintFrom(new ConstraintGPSPseudorange(getFeatureListPtr()->front(), getFramePtr()));
    // ora faccio un costruttore del tutto estraeneo a wolf,
    // con solo le cose di cui ho bisogno


}

//TODO implementata a caso, solo perche' e' virtual pura e altrimenti non compila
Eigen::VectorXs CaptureGPS::computePrior(const TimeStamp &_now) const
{
    return Eigen::Vector3s(1, 2, 3);
}
