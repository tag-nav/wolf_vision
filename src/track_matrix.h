/**
 * \file track_matrix.h
 *
 *  Created on: Mar 24, 2018
 *      \author: jsola
 */

#ifndef TRACK_MATRIX_H_
#define TRACK_MATRIX_H_

#include "feature_base.h"
#include "capture_base.h"

#include <map>
#include <vector>
#include <list>
#include <utility>

namespace wolf
{
using std::map;
using std::vector;
using std::list;
using std::pair;
using std::shared_ptr;

typedef map<TimeStamp, FeatureBasePtr>                      Track;
typedef map<size_t, FeatureBasePtr >                        Snapshot;
typedef map<size_t, pair<FeatureBasePtr, FeatureBasePtr> >  TrackMatches; // matched feature pairs indexed by track_id

/** \brief Matrix of tracked features, by track and by snapshot (Captures or time stamps)
 * This class implements the following data structure:
 *
 * Snapshots at each capture Cx:
 *      C1     C2     C3     C4     C5
 *                                       Tracks for each matched feature:
 *      f1 --- f3 --- f7 --- f11 -- f14    <-- track 1, of corresponding features in different captures
 *      |      |      |      |      |
 *      |      f4 --- f8 --- f12 -- f15    <-- track 2
 *      |      |      |      |
 *      |      f5 --- f9 --- f13           <-- track 3
 *      |      |      |
 *      f2 --- f6 --- f10                  <-- track 4
 *
 *      Each 'f' is a feature pointer of the type FeatureBasePtr
 *
 * This structure allows accessing all the history of matched features with two different accesses:
 *
 *      Track:    track-wise:   a track of features matches along a timely sequence of Captures, indexed by time stamp.
 *
 *                              map<TimeStamp ts, FeatureBasePtr f>
 *
 *      Snapshot: capture-wise: a set of features tracked in a single Capture, indexed by track id:
 *
 *                              map<size_t track_id, FeatureBasePtr f>
 *
 * The class makes sure that both accesses are consistent each time some addition or removal of features is performed.
 *
 * The storage is implemented as two maps of maps, so each addition and removal of single features is accomplished in logarithmic time:
 *
 *      Tracks are identified with the track ID in           f->trackId()
 *      Snapshots are identified with the Capture pointer in f->getCapturePtr()
 *
 * these fields of FeatureBase are initialized each time a feature is added to the track matrix:
 *
 *      add(Cap, track_id, f) will set f.capture_ptr = C and f.traci_id = traci_id.
 *
 * so e.g. given a feature f,
 *
 *      getTrack   (f->trackId()) ;       // returns all the track where feature f is.
 *      getSnapshot(f->getCapturePtr()) ; // returns all the features in the same capture of f.
 *
 */

class TrackMatrix
{
    public:
        TrackMatrix();
        virtual ~TrackMatrix();

        void            newTrack    (CaptureBasePtr _cap, FeatureBasePtr _ftr);
        void            add         (size_t _track_id, CaptureBasePtr _cap, FeatureBasePtr _ftr);
        void            remove      (FeatureBasePtr _ftr);
        void            remove      (size_t _track_id);
        void            remove      (CaptureBasePtr _cap);
        SizeStd          numTracks   ();
        SizeStd          trackSize   (size_t _track_id);
        Track           track       (size_t _track_id);
        Snapshot        snapshot    (CaptureBasePtr _capture);
        vector<FeatureBasePtr>
                        trackAsVector(size_t _track_id);
        list<FeatureBasePtr>
                        snapshotAsList(CaptureBasePtr _cap);
        TrackMatches    matches     (CaptureBasePtr _cap_1, CaptureBasePtr _cap_2);
        FeatureBasePtr  firstFeature(size_t _track_id);
        FeatureBasePtr  lastFeature (size_t _track_id);
        FeatureBasePtr  feature     (size_t _track_id, CaptureBasePtr _cap);
        CaptureBasePtr  firstCapture(size_t _track_id);

    private:

        static SizeStd track_id_count_;

        // Along track: maps of Feature pointers indexed by time stamp.
        map<size_t, Track > tracks_;       // map indexed by track_Id   of ( maps indexed by TimeStamp  of ( features ) )

        // Across track: maps of Feature pointers indexed by track_Id.
        map<size_t, Snapshot > snapshots_; // map indexed by capture_Id of ( maps indexed by track_Id of ( features ) )
};

} /* namespace wolf */

#endif /* TRACK_MATRIX_H_ */
