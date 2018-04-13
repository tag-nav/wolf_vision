/**
 * \file gtest_track_matrix.cpp
 *
 *  Created on: Mar 24, 2018
 *      \author: jsola
 */

#include "utils_gtest.h"

#include "track_matrix.h"

using namespace wolf;

class TrackMatrixTest : public testing::Test
{
    public:
        TrackMatrix track_matrix;

        Eigen::Vector2s m;
        Eigen::Matrix2s m_cov;

        CaptureBasePtr C0, C1, C2, C3, C4;
        FeatureBasePtr f0, f1, f2, f3, f4;

        virtual void SetUp()
        {
            C0 = std::make_shared<CaptureBase>("BASE", 0.0);
            C1 = std::make_shared<CaptureBase>("BASE", 1.0);
            C2 = std::make_shared<CaptureBase>("BASE", 2.0);
            C3 = std::make_shared<CaptureBase>("BASE", 3.0);
            C4 = std::make_shared<CaptureBase>("BASE", 4.0);

            f0 = std::make_shared<FeatureBase>("BASE", m, m_cov);
            f1 = std::make_shared<FeatureBase>("BASE", m, m_cov);
            f2 = std::make_shared<FeatureBase>("BASE", m, m_cov);
            f3 = std::make_shared<FeatureBase>("BASE", m, m_cov);
            f4 = std::make_shared<FeatureBase>("BASE", m, m_cov);
        }
};

TEST_F(TrackMatrixTest, newTrack)
{
    track_matrix.newTrack(C0, f0);
    ASSERT_EQ(track_matrix.numTracks(), 1);

    track_matrix.newTrack(C0, f1);
    ASSERT_EQ(track_matrix.numTracks(), 2);
}

TEST_F(TrackMatrixTest, add)
{
    track_matrix.newTrack(C0, f0);

    track_matrix.add(f0->trackId(), C1, f1);
    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     */
    ASSERT_EQ(track_matrix.trackSize(f1->trackId()), 2);
    ASSERT_EQ(f1->trackId(), f0->trackId());

    track_matrix.add(f0->trackId(), C2, f2);
    /*  C0   C1   C2   snapshots
     *
     *  f0---f1---f2   trk 0
     */
    ASSERT_EQ(track_matrix.trackSize(f2->trackId()), 3);
    ASSERT_EQ(f2->trackId(), f0->trackId());
}

TEST_F(TrackMatrixTest, numTracks)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     */

    ASSERT_EQ(track_matrix.numTracks(), 1);

    track_matrix.add(f0->trackId(), C0, f2);

    ASSERT_EQ(track_matrix.numTracks(), 1);
}

TEST_F(TrackMatrixTest, trackSize)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    ASSERT_EQ(track_matrix.trackSize(f0->trackId()), 2);
    ASSERT_EQ(track_matrix.trackSize(f2->trackId()), 1);
}

TEST_F(TrackMatrixTest, first_last_Feature)
{
    /*  C0  C1  C2   snapshots
     *
     *  f0--f1      trk 0
     *      |
     *      f2      trk 1
     */

    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C1, f2);

    ASSERT_EQ(track_matrix.firstFeature(f0->trackId()), f0);
    ASSERT_EQ(track_matrix.lastFeature (f0->trackId()), f1);
    ASSERT_EQ(track_matrix.feature (f0->trackId(), C0 ), f0);
    ASSERT_EQ(track_matrix.feature (f0->trackId(), C1 ), f1);
    ASSERT_EQ(track_matrix.feature (f2->trackId(), C1 ), f2);
}

TEST_F(TrackMatrixTest, remove_ftr)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    ASSERT_EQ(track_matrix.numTracks(), 2);

    track_matrix.remove(f0);
    /*  C0   C1   C2   snapshots
     *
     *       f1        trk 0
     *
     *  f2             trk 1
     */
    ASSERT_EQ(track_matrix.numTracks(), 2);
    ASSERT_EQ(track_matrix.firstFeature(f0->trackId()), f1);
    ASSERT_EQ(track_matrix.firstFeature(f2->trackId()), f2);
    ASSERT_EQ(track_matrix.snapshot(C0).size(), 1);
    ASSERT_EQ(track_matrix.snapshot(C0).at(f2->trackId()), f2);
    ASSERT_EQ(track_matrix.snapshot(C1).size(), 1);
    ASSERT_EQ(track_matrix.snapshot(C1).at(f0->trackId()), f1);

    track_matrix.remove(f1);
    ASSERT_EQ(track_matrix.numTracks(), 1);
    ASSERT_EQ(track_matrix.firstFeature(f2->trackId()), f2);
}

TEST_F(TrackMatrixTest, remove_trk)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    ASSERT_EQ(track_matrix.numTracks(), 2);

    track_matrix.remove(f0->trackId());
    ASSERT_EQ(track_matrix.numTracks(), 1);
    ASSERT_EQ(track_matrix.firstFeature(f2->trackId()), f2);

    track_matrix.remove(f2->trackId());
    ASSERT_EQ(track_matrix.numTracks(), 0);
}

TEST_F(TrackMatrixTest, remove_snapshot)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    ASSERT_EQ(track_matrix.numTracks(), 2);

    track_matrix.remove(C0);
    /*  C1   C2   snapshots
     *
     *  f1        trk 0
     */
    ASSERT_EQ(track_matrix.numTracks(), 1);
    ASSERT_EQ(track_matrix.firstFeature(f1->trackId()), f1);

    track_matrix.remove(C1);
    ASSERT_EQ(track_matrix.numTracks(), 0);
}

TEST_F(TrackMatrixTest, track)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    Track t0 = track_matrix.track(f0->trackId());
    Track t2 = track_matrix.track(f2->trackId());

    ASSERT_EQ(t0.size(), 2);
    ASSERT_EQ(t0.at(C0->getTimeStamp()), f0);
    ASSERT_EQ(t0.at(C1->getTimeStamp()), f1);

    ASSERT_EQ(t2.size(), 1);
    ASSERT_EQ(t2.at(C0->getTimeStamp()), f2);
}

TEST_F(TrackMatrixTest, snapshot)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    Snapshot s0 = track_matrix.snapshot(C0);
    Snapshot s1 = track_matrix.snapshot(C1);

    ASSERT_EQ(s0.size(), 2);
    ASSERT_EQ(s0.at(f0->trackId()), f0);
    ASSERT_EQ(s0.at(f2->trackId()), f2);

    ASSERT_EQ(s1.size(), 1);
    ASSERT_EQ(s1.at(f1->trackId()), f1);
}

TEST_F(TrackMatrixTest, trackAsVector)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    std::vector<FeatureBasePtr> vt0 = track_matrix.trackAsVector(f0->trackId()); // get track 0 as vector

    ASSERT_EQ(vt0.size(), 2);
    ASSERT_EQ(vt0[0], f0);
    ASSERT_EQ(vt0.at(1), f1);
}

TEST_F(TrackMatrixTest, snapshotAsList)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.newTrack(C0, f2);

    /*  C0   C1   C2   snapshots
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    std::list<FeatureBasePtr> lt0 = track_matrix.snapshotAsList(f0->getCapturePtr()); // get track 0 as vector

    ASSERT_EQ(lt0.size() , 2);
    ASSERT_EQ(lt0.front(), f0);
    ASSERT_EQ(lt0.back() , f2);
}

TEST_F(TrackMatrixTest, matches)
{
    track_matrix.newTrack(C0, f0);
    track_matrix.add(f0->trackId(), C1, f1);
    track_matrix.add(f0->trackId(), C2, f2);
    track_matrix.newTrack(C0, f3);
    track_matrix.add(f3->trackId(), C1, f4);

    /*  C0   C1   C2   C3   snapshots
     *
     *  f0---f1---f2        trk 0
     *  |    |
     *  f3---f4             trk 1
     */

    TrackMatches pairs = track_matrix.matches(C0, C2);

    ASSERT_EQ(pairs.size(), 1);
    ASSERT_EQ(pairs.at(f0->trackId()).first, f0);
    ASSERT_EQ(pairs.at(f0->trackId()).second, f2);

    pairs = track_matrix.matches(C2, C3);

    ASSERT_EQ(pairs.size(), 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

