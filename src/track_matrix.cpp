/**
 * \file track_matrix.cpp
 *
 *  Created on: Mar 24, 2018
 *      \author: jsola
 */

#include "track_matrix.h"

namespace wolf
{

size_t TrackMatrix::track_id_count_ = 0;

TrackMatrix::TrackMatrix()
{
    //
}

TrackMatrix::~TrackMatrix()
{
    //
}

Track TrackMatrix::track(size_t _track_id)
{
    if (tracks_.count(_track_id) > 0)
        return tracks_.at(_track_id);
    else
        return Track();
}

Snapshot TrackMatrix::snapshot(CaptureBasePtr _capture)
{
    if (_capture && snapshots_.count(_capture->id()) > 0)
        return snapshots_.at(_capture->id());
    else
        return Snapshot();
}

void TrackMatrix::newTrack(CaptureBasePtr _cap, FeatureBasePtr _ftr)
{
    track_id_count_ ++;
    add(track_id_count_, _cap, _ftr);
}

void TrackMatrix::add(size_t _track_id, CaptureBasePtr _cap, FeatureBasePtr _ftr)
{
    assert( (_track_id > 0) && (_track_id <= track_id_count_) && "Provided track ID does not exist. Use newTrack() instead.");

    _ftr->setTrackId(_track_id);
    if (_cap != _ftr->getCapturePtr())
        _ftr->setCapturePtr(_cap);
    tracks_[_track_id].emplace(_cap->getTimeStamp(), _ftr);
    snapshots_[_cap->id()].emplace(_track_id, _ftr);        // will create new snapshot if _cap_id   is not present
}

void TrackMatrix::remove(size_t _track_id)
{
    // Remove track features from all Snapshots
    if (tracks_.count(_track_id))
    {
        for (auto const& pair_time_ftr : tracks_.at(_track_id))
        {
            SizeStd cap_id = pair_time_ftr.second->getCapturePtr()->id();
            snapshots_.at(cap_id).erase(_track_id);
            if (snapshots_.at(cap_id).empty())
                snapshots_.erase(cap_id);
        }

        // Remove track
        tracks_.erase(_track_id);
    }
}

void TrackMatrix::remove(CaptureBasePtr _cap)
{
    // remove snapshot features from all tracks
    if (snapshots_.count(_cap->id()))
    {
        TimeStamp ts = _cap->getTimeStamp();
        for (auto const& pair_trkid_ftr : snapshots_.at(_cap->id()))
        {
            SizeStd trk_id = pair_trkid_ftr.first;
            tracks_.at(trk_id).erase(ts);
            if (tracks_.at(trk_id).empty())
                tracks_.erase(trk_id);
        }

        // remove snapshot
        snapshots_.erase(_cap->id());
    }
}

void TrackMatrix::remove(FeatureBasePtr _ftr)
{
    // assumes _ftr->getCapturePtr() and _ftr->trackId() are valid
    if (_ftr)
    {
        if (auto cap = _ftr->getCapturePtr())
        {
            tracks_   .at(_ftr->trackId()).erase(cap->getTimeStamp());
            if (tracks_.at(_ftr->trackId()).empty())
                tracks_.erase(_ftr->trackId());

            snapshots_.at(cap->id())      .erase(_ftr->trackId());
            if (snapshots_.at(cap->id()).empty())
                snapshots_.erase(cap->id());
        }
    }
}

size_t TrackMatrix::numTracks()
{
    return tracks_.size();
}

size_t TrackMatrix::trackSize(size_t _track_id)
{
    return track(_track_id).size();
}

FeatureBasePtr TrackMatrix::firstFeature(size_t _track_id)
{
    if (tracks_.count(_track_id) > 0)
        return tracks_.at(_track_id).begin()->second;
    else
        return nullptr;
}

FeatureBasePtr TrackMatrix::lastFeature(size_t _track_id)
{
    if (tracks_.count(_track_id) > 0)
        return tracks_.at(_track_id).rbegin()->second;
    else
        return nullptr;
}

vector<FeatureBasePtr> TrackMatrix::trackAsVector(size_t _track_id)
{
    vector<FeatureBasePtr> vec;
    if (tracks_.count(_track_id))
    {
        vec.reserve(trackSize(_track_id));
        for (auto const& pair_time_ftr : tracks_.at(_track_id))
            vec.push_back(pair_time_ftr.second);
    }
    return vec;
}

std::list<FeatureBasePtr> TrackMatrix::snapshotAsList(CaptureBasePtr _cap)
{
    std::list<FeatureBasePtr> lst;
    if (snapshots_.count(_cap->id()))
        for (auto const& pair_trkid_ftr : snapshots_.at(_cap->id()))
            lst.push_back(pair_trkid_ftr.second);
    return lst;
}

TrackMatches TrackMatrix::matches(CaptureBasePtr _cap_1, CaptureBasePtr _cap_2)
{
    TrackMatches pairs;

    Snapshot s_1 = snapshot(_cap_1);
    Snapshot s_2 = snapshot(_cap_2);
    Snapshot s_short = s_1;
    Snapshot s_long  = s_2;
    if (s_1.size() > s_2.size())
    {
        s_long  = s_1;
        s_short = s_2;
    }

    for (auto const & pair_trkid_ftr : s_short)
    {
        SizeStd trk_id = pair_trkid_ftr.first;
        if (s_long.count(trk_id))
            pairs[trk_id] = pair<FeatureBasePtr, FeatureBasePtr>(s_1.at(trk_id), s_2.at(trk_id));
    }

    return pairs;
}

FeatureBasePtr TrackMatrix::feature(size_t _track_id, CaptureBasePtr _cap)
{
    if (snapshot(_cap).count(_track_id))
        return snapshot(_cap).at(_track_id);
    else
        return nullptr;
}

CaptureBasePtr TrackMatrix::firstCapture(size_t _track_id)
{
    return firstFeature(_track_id)->getCapturePtr();
}

}
