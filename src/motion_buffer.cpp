#include "motion_buffer.h"
namespace wolf
{

Motion::Motion(const TimeStamp& _ts,
               const VectorXs& _delta,
               const VectorXs& _delta_int,
               const MatrixXs& _jac_delta,
               const MatrixXs& _jac_delta_int,
               const MatrixXs& _delta_cov,
               const MatrixXs& _jac_extra) :
        delta_size_(_delta.size()),
        cov_size_(_delta_cov.size()),
        ts_(_ts),
        delta_(_delta),
        delta_integr_(_delta_int),
        jacobian_delta_(_jac_delta),
        jacobian_delta_integr_(_jac_delta_int),
        delta_cov_(_delta_cov),
        jacobian_calib_(_jac_extra)
{
}

Motion::Motion(const TimeStamp& _ts, Size _delta_size, Size _cov_size) :
        ts_(_ts)
{
    resize(_delta_size, _cov_size == 0 ? _delta_size : _cov_size);
}

Motion::~Motion()
{
}

void Motion::resize(Size ds)
{
    resize(ds, ds);
}

void Motion::resize(Size ds, Size dcs)
{
    delta_.resize(ds);
    delta_integr_.resize(ds);
    jacobian_delta_.resize(dcs, dcs);
    jacobian_delta_integr_.resize(dcs, dcs);
    delta_cov_.resize(dcs, dcs);
//    delta_integr_cov_.resize(dcs, dcs);
}


////////////////////////////////////////////////////////////////////////////////////////

MotionBuffer::MotionBuffer(Size _delta_size, Size _cov_size) :
        delta_size_(_delta_size), cov_size_(_cov_size)
{
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
        _buffer_part_before_ts.container_.splice(_buffer_part_before_ts.container_.begin(), container_, container_.begin(),
                                         (previous--).base());
    }
}

MatrixXs MotionBuffer::integrateCovariance() const
{
    Eigen::MatrixXs cov(cov_size_, cov_size_);
    cov.setZero();
    for (Motion mot : container_)
    {
        cov = mot.jacobian_delta_integr_ * cov * mot.jacobian_delta_integr_.transpose()
                + mot.jacobian_delta_ * mot.delta_cov_ * mot.jacobian_delta_.transpose();
    }
    return cov;
}

MatrixXs MotionBuffer::integrateCovariance(const TimeStamp& _ts) const
{
    Eigen::MatrixXs cov(cov_size_, cov_size_);
    cov.setZero();
    for (Motion mot : container_)
    {
        if (mot.ts_ > _ts)
            break;

        cov = mot.jacobian_delta_integr_ * cov * mot.jacobian_delta_integr_.transpose()
                + mot.jacobian_delta_ * mot.delta_cov_ * mot.jacobian_delta_.transpose();
    }
    return cov;
}

MatrixXs MotionBuffer::integrateCovariance(const TimeStamp& _ts_1, const TimeStamp _ts_2) const
{
    Eigen::MatrixXs cov(cov_size_, cov_size_);
    cov.setZero();
    for (Motion mot : container_)
    {
        if (mot.ts_ > _ts_2)
            break;

        if (mot.ts_ >= _ts_1)
            cov = mot.jacobian_delta_integr_ * cov * mot.jacobian_delta_integr_.transpose()
                + mot.jacobian_delta_ * mot.delta_cov_ * mot.jacobian_delta_.transpose();
    }
    return cov;
}

void MotionBuffer::print(bool show_delta, bool show_delta_cov, bool show_delta_int, bool show_delta_int_cov)
{
    using std::cout;
    using std::endl;

    if (!show_delta && !show_delta_cov && !show_delta_int && !show_delta_int_cov)
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
            if (show_delta)
                cout << "   delta: " << mot.delta_.transpose() << endl;
            if (show_delta_cov)
                cout << "   delta cov: \n" << mot.delta_cov_ << endl;
            if (show_delta_int)
                cout << "   delta integrated: " << mot.delta_integr_.transpose() << endl;
            if (show_delta_int_cov)
                cout << "   delta integrated cov: \n" << integrateCovariance(mot.ts_) << endl;
        }
    }
}


}

