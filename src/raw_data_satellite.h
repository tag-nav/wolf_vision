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
        std::cout << "Received observation data for satellite " << sat_id_ << std::endl;
    }

    std::string toString()
    {
        std::ostringstream s;
        s << "SatID: " << sat_id_ << " -- il resto TODO -- " << timestamp_.get() << " -- " << pseudorange_ << " --- " << sat_position_ << std::endl;
        return s.str();
    }

    const std::string &getSatId() const {
        return sat_id_;
    }

    const TimeStamp &getTimestamp() const {
        return timestamp_;
    }

    WolfScalar getPseudorange() const {
        return pseudorange_;
    }

    Eigen::Vector3s getSatPosition() const {

        // TODO first calculate sat position!!!!
        // TODO decide if keep sat position here or somewhere else

        return sat_position_;
    }

    void setSatPosition(const Eigen::Vector3s &sat_position_) {//TODO nel caso voglia calcolarla da fuori. anche se a sto punto devo valutare se tenerla qui o no
        ObsData::sat_position_ = sat_position_;
    }

protected:
    std::string sat_id_;
    TimeStamp timestamp_;
    WolfScalar pseudorange_;
    Eigen::Vector3s sat_position_;

};



#endif //RAW_DATA_SATELLITE_H
