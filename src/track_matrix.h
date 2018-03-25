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

namespace wolf
{
using std::map;
using std::vector;
using std::shared_ptr;

typedef map<TimeStamp, FeatureBasePtr>          TrackType;
typedef map<unsigned long int, FeatureBasePtr > SnapshotType;

/* This class implements the following data structure:
 *
 * Snapshots at each capture Cx:
 *      C1     C2     C3     C4     C5
 *                                       Tracks for each matched feature:
 *      f ---- f ---- f ---- f ---- f    <-- track 1, of corresponding features in different captures
 *      |      |      |      |      |
 *      |      f ---- f ---- f ---- f    <-- track 2
 *      |      |      |      |
 *      |      f ---- f ---- f           <-- track 3
 *      |      |      |
 *      f ---- f ---- f                  <-- track 4
 *
 *      Each 'f' is a feature pointer of the type FeatureBasePtr
 *
 * This structure allows accessing all the history of matched features with two different accesses:
 *
 *      Track:    track-wise:   a track of features matches along a timely sequence of Captures
 *      Snapshot: capture-wise: a set of features tracked in a single Capture
 *
 * The class makes sure that both accesses are consistent each time some addition or removal of features is performed.
 *
 * The storage is implemented as two maps of maps, so each addition and removal of single features is accomplished in logarithmic time:
 *
 *      Tracks are identified with the track ID in           f->trackId()
 *      Snapshots are identified with the Capture pointer in f->getCapturePtr()
 *
 * so e.g.
 *
 *      getTrack   (f->trackId()) ;       // returns all the track for feature f.
 *      getSnapshot(f->getCapturePtr()) ; // returns all the features in the same capture of f.
 */

class TrackMatrix
{
    public:
        TrackMatrix();
        virtual ~TrackMatrix();

        TrackType       track(size_t _track_id);
        SnapshotType    snapshot(CaptureBasePtr _capture);
        void            add(CaptureBasePtr _cap, FeatureBasePtr _ftr);
        void            remove(FeatureBasePtr _ftr);
        void            remove(size_t _track_id);
        void            remove(CaptureBasePtr _cap);
        size_t          numTracks();
        size_t          trackSize(size_t _track_id);
        FeatureBasePtr  firstFeature(size_t _track_id);
        FeatureBasePtr  lastFeature(size_t _track_id);
        std::vector<FeatureBasePtr>
                        trackAsVector(size_t _track_id);
        FeatureBasePtr  feature(size_t _track_id, size_t _position);
        CaptureBasePtr  firstCapture(size_t _track_id);

    private:

        // Along track: maps of Feature pointers indexed by time stamp.
        std::map<size_t, TrackType > tracks_;       // map indexed by track_Id   of ( maps indexed by TimeStamp  of ( features ) )

        // Across track: maps of Feature pointers indexed by Feature Id.
        std::map<size_t, SnapshotType > snapshots_; // map indexed by capture_Id of ( maps indexed by track_Id of ( features ) )
};

} /* namespace wolf */

#endif /* TRACK_MATRIX_H_ */
