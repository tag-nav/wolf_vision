/*
 * feature_track_map.h
 *
 *  Created on: Mar 23, 2018
 *      Author: jsola
 */

#ifndef FEATURE_TRACK_MAP_H_
#define FEATURE_TRACK_MAP_H_

#include "feature_base.h"

#include <map>
#include <deque>

namespace wolf
{

class FeatureTrackMap
{
    public:
        FeatureTrackMap(){}
        virtual ~FeatureTrackMap(){}

        std::deque<FeatureBasePtr> get(size_t _track_id)
        {
            if (container_.count(_track_id) > 0)
                return container_.at(_track_id);
            else
                return std::deque<FeatureBasePtr>();
        }

        void add(FeatureBasePtr _ftr)
        {
            container_[_ftr->trackId()].push_back(_ftr); // will create new track if _id is not present
        }
        void remove(size_t _track_id)
        {
            container_.erase(_track_id);
        }
        size_t size()
        {
            return container_.size();
        }
        size_t trackSize(size_t _track_id)
        {
            if (container_.count(_track_id) > 0)
                return container_.at(_track_id).size();
            else
                return 0;
        }
        FeatureBasePtr getFirstFeature(size_t _track_id)
        {
            if (container_.count(_track_id) > 0)
                return container_.at(_track_id).front();
            else
                return nullptr;
        }

        FeatureBasePtr getLastFeature(size_t _track_id)
        {
            if (container_.count(_track_id) > 0)
                return container_.at(_track_id).back();
            else
                return nullptr;
        }
        FeatureBasePtr getFeature(size_t _track_id, size_t _position)
        {
            if (container_.count(_track_id) > 0 && _position <= trackSize(_track_id))
                return container_.at(_track_id).at(_position);
            else
                return nullptr;
        }

    private:
        std::map<unsigned int, std::deque<FeatureBasePtr> > container_;

};

} /* namespace wolf */

#endif /* FEATURE_TRACK_MAP_H_ */
