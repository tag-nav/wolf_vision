//
// TEMPORARY FILE
// TEMPORARY FILE
// TEMPORARY FILE
// TEMPORARY FILE
// TEMPORARY FILE
// TEMPORARY FILE
// TEMPORARY FILE
// TEMPORARY FILE
// TEMPORARY FILE
//
/*
 * I've created this source just to use something that maybe can be
 * similar to what probably will be the output of the real receiver.
 * This is a temporary solution, when some real data will be ready
 * i'll decide where/how to put it.
 *
 * [pier]
 */

#ifndef RAW_DATA_SATELLITE_H
#define RAW_DATA_SATELLITE_H




class ObsData
{
public:
    ObsData(const std::string &_sat_id, const TimeStamp &_timestamp, WolfScalar &_pseudorange) :
            sat_id_(_sat_id), timestamp_(_timestamp), pseudorange_(_pseudorange)
    {
        //std::cout << "ObsData constructor: " << toString();
    }

    /** TODO now it's a dummy function
     * Function to calculate the sat position in the time timestamp_
     */
    void calculateSatPosition()
    {
        sat_position_ = Eigen::Vector3s(50000+pseudorange_, 60000+pseudorange_, 70000+pseudorange_);
    }

    std::string toString()
    {
        std::ostringstream s;
        s << "SatID: " << sat_id_ << " -- " << timestamp_.get() << " -- " << pseudorange_ << " -- (" << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2];
        return s.str();
    }

    const std::string &getSatId() const
    {
        return sat_id_;
    }

    const TimeStamp &getTimestamp() const
    {
        return timestamp_;
    }

    WolfScalar getPseudorange() const
    {
        return pseudorange_;
    }

    Eigen::Vector3s getSatPosition() const
    {
        return sat_position_;
    }

    //TODO nel caso voglia calcolarla da fuori. anche se a sto punto devo valutare se tenerla qui o no
    void setSatPosition(const Eigen::Vector3s &sat_position_)
    {
        ObsData::sat_position_ = sat_position_;
    }

protected:
    std::string sat_id_;
    TimeStamp timestamp_;
    WolfScalar pseudorange_;
    Eigen::Vector3s sat_position_;

};



#endif //RAW_DATA_SATELLITE_H
