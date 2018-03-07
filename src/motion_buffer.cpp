#include "motion_buffer.h"
namespace wolf
{

Motion::Motion(const TimeStamp& _ts,
               const VectorXs& _data,
               const MatrixXs& _data_cov,
               const VectorXs& _delta,
               const MatrixXs& _delta_cov,
               const VectorXs& _delta_integr,
               const MatrixXs& _delta_integr_cov,
               const MatrixXs& _jac_delta,
               const MatrixXs& _jac_delta_int,
               const MatrixXs& _jac_calib) :
        data_size_(_data.size()),
        delta_size_(_delta.size()),
        cov_size_(_delta_cov.cols()),
        calib_size_(_jac_calib.cols()),
        ts_(_ts),
        data_(_data),
        data_cov_(_data_cov),
        delta_(_delta),
        delta_cov_(_delta_cov),
        delta_integr_(_delta_integr),
        delta_integr_cov_(_delta_integr_cov),
        jacobian_delta_(_jac_delta),
        jacobian_delta_integr_(_jac_delta_int),
        jacobian_calib_(_jac_calib)
{
}

Motion::Motion(const TimeStamp& _ts,
               Size _data_size,
               Size _delta_size,
               Size _cov_size,
               Size _calib_size) :
        data_size_(_data_size),
        delta_size_(_delta_size),
        cov_size_(_cov_size),
        calib_size_(_calib_size),
        ts_(_ts)
{
    resize(_data_size, _delta_size, _cov_size, _calib_size);
}

Motion::~Motion()
{
}

void Motion::resize(Size _data_s, Size _delta_s, Size _delta_cov_s, Size _calib_s)
{
    data_.resize(_data_s);
    data_cov_.resize(_data_s, _data_s);
    delta_.resize(_delta_s);
    delta_cov_.resize(_delta_cov_s, _delta_cov_s);
    delta_integr_.resize(_delta_s);
    delta_integr_cov_.resize(_delta_cov_s, _delta_cov_s);
    jacobian_delta_.resize(_delta_cov_s, _delta_cov_s);
    jacobian_delta_integr_.resize(_delta_cov_s, _delta_cov_s);
    jacobian_calib_.resize(_delta_cov_s, _calib_s);
}


////////////////////////////////////////////////////////////////////////////////////////

MotionBuffer::MotionBuffer(Size _data_size,
                           Size _delta_size,
                           Size _cov_size,
                           Size _calib_size) :
        data_size_(_data_size),
        delta_size_(_delta_size),
        cov_size_(_cov_size),
        calib_size_(_calib_size),
        calib_preint_(_calib_size)
{
    container_.clear();
}

const Motion& MotionBuffer::getMotion(const TimeStamp& _ts) const
{
    //assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
        // The time stamp is older than the buffer's oldest data.
        // We could do something here, and throw an error or something, but by now we'll return the first valid data
        previous--;

    return *previous;
}

void MotionBuffer::getMotion(const TimeStamp& _ts, Motion& _motion) const
{
    //assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
        // The time stamp is older than the buffer's oldest data.
        // We could do something here, but by now we'll return the last valid data
        previous--;

    _motion = *previous;
}

void MotionBuffer::split(const TimeStamp& _ts, MotionBuffer& _buffer_part_before_ts)
{
    assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");

    _buffer_part_before_ts.setCalibrationPreint(calib_preint_);

    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
    {
        // The time stamp is more recent than the buffer's most recent data:
        // return an empty buffer as the _oldest_buffer
        _buffer_part_before_ts.get().clear();
    }
    else
    {
        // Transfer part of the buffer
        _buffer_part_before_ts.container_.splice(_buffer_part_before_ts.container_.begin(),
                                                 container_,
                                                 container_.begin(),
                                                 (previous--).base());
    }
}

//MatrixXs MotionBuffer::integrateCovariance() const
//{
//    Eigen::MatrixXs delta_integr_cov(cov_size_, cov_size_);
//    delta_integr_cov.setZero();
//    for (Motion mot : container_)
//    {
//        delta_integr_cov = mot.jacobian_delta_integr_ * delta_integr_cov * mot.jacobian_delta_integr_.transpose()
//                         + mot.jacobian_delta_        * mot.delta_cov_   * mot.jacobian_delta_.transpose();
//    }
//    return delta_integr_cov;
//}
//
//MatrixXs MotionBuffer::integrateCovariance(const TimeStamp& _ts) const
//{
//    Eigen::MatrixXs delta_integr_cov(cov_size_, cov_size_);
//    delta_integr_cov.setZero();
//    for (Motion mot : container_)
//    {
//        if (mot.ts_ > _ts)
//            break;
//
//        delta_integr_cov = mot.jacobian_delta_integr_ * delta_integr_cov * mot.jacobian_delta_integr_.transpose()
//                         + mot.jacobian_delta_        * mot.delta_cov_   * mot.jacobian_delta_.transpose();
//    }
//    return delta_integr_cov;
//}
//
//MatrixXs MotionBuffer::integrateCovariance(const TimeStamp& _ts_1, const TimeStamp _ts_2) const
//{
//    Eigen::MatrixXs cov(cov_size_, cov_size_);
//    cov.setZero();
//    for (Motion mot : container_)
//    {
//        if (mot.ts_ > _ts_2)
//            break;
//
//        if (mot.ts_ >= _ts_1)
//            cov = mot.jacobian_delta_integr_ * cov * mot.jacobian_delta_integr_.transpose()
//                + mot.jacobian_delta_ * mot.delta_cov_ * mot.jacobian_delta_.transpose();
//    }
//    return cov;
//}

void MotionBuffer::print(bool show_data, bool show_delta, bool show_delta_int, bool show_jacs)
{
    using std::cout;
    using std::endl;

    if (!show_data && !show_delta && !show_delta_int && !show_jacs)
    {
        cout << "Buffer state [" << container_.size() << "] : <";
        for (Motion mot : container_)
            cout << " " << mot.ts_;
        cout << " >" << endl;
    }
    else
    {
        print(0,0,0,0);
        for (Motion mot : container_)
        {
            cout << "-- Motion (" << mot.ts_ << ")" << endl;
//            if (show_ts)
//                cout << "   ts: " << mot.ts_ << endl;
            if (show_data)
            {
                cout << "   data: " << mot.data_.transpose() << endl;
                cout << "   data cov: \n" << mot.data_cov_ << endl;
            }
            if (show_delta)
            {
                cout << "   delta: " << mot.delta_.transpose() << endl;
                cout << "   delta cov: \n" << mot.delta_cov_ << endl;
            }
            if (show_delta_int)
            {
                cout << "   delta integrated: " << mot.delta_integr_.transpose() << endl;
                cout << "   delta integrated cov: \n" << mot.delta_integr_cov_ << endl;
            }
            if (show_jacs)
            {
                cout << "   Jac delta: \n" << mot.jacobian_delta_ << endl;
                cout << "   Jac delta integr: \n" << mot.jacobian_delta_integr_ << endl;
                cout << "   Jac calib: \n" << mot.jacobian_calib_ << endl;

            }
        }
    }
}


}

