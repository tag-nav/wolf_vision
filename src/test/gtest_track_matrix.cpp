/**
 * \file gtest_track_matrix.cpp
 *
 *  Created on: Mar 24, 2018
 *      \author: jsola
 */

#include "utils_gtest.h"

#include "/Users/jsola/dev/wolf/src/track_matrix.h"

using namespace wolf;

Eigen::Vector2s m;
Eigen::Matrix2s m_cov;

class TrackMatrixTest : public testing::Test
{
    public:
        TrackMatrix track_matrix;

        CaptureBasePtr C0, C1, C2;
        FeatureBasePtr f0, f1, f2;

        virtual void SetUp()
        {
            C0 = std::make_shared<CaptureBase>("BASE", 0.0);
            C1 = std::make_shared<CaptureBase>("BASE", 1.0);
            C2 = std::make_shared<CaptureBase>("BASE", 2.0);

            f0 = std::make_shared<FeatureBase>("BASE", m, m_cov);
            f1 = std::make_shared<FeatureBase>("BASE", m, m_cov);
            f2 = std::make_shared<FeatureBase>("BASE", m, m_cov);

            /*  C0   C1   C2
             *
             *  f0---f1        trk 0
             *  |
             *  f2             trk 1
             */
            f0->setTrackId(0);
            f1->setTrackId(0);
            f2->setTrackId(1);

        }
};

TEST(TrackMatrix, add)
{
    TrackMatrix track_matrix;

    CaptureBasePtr C = std::make_shared<CaptureBase>("BASE", 0.0);

    FeatureBasePtr f = std::make_shared<FeatureBase>("BASE", m, m_cov);
    f->setTrackId(f->id());

    track_matrix.add(C, f);

    ASSERT_EQ(track_matrix.trackSize(f->trackId()), 1);
}

TEST_F(TrackMatrixTest, numTracks)
{

    track_matrix.add(C0, f0);
    track_matrix.add(C1, f1);

    /*  C0   C1   C2
     *
     *  f0---f1        trk 0
     */

    ASSERT_EQ(track_matrix.numTracks(), 1);

    track_matrix.add(C0, f2);

    ASSERT_EQ(track_matrix.numTracks(), 2);
}

TEST_F(TrackMatrixTest, trackSize)
{
    track_matrix.add(C0, f0);
    track_matrix.add(C1, f1);
    track_matrix.add(C0, f2);

    /*  C0   C1   C2
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    ASSERT_EQ(track_matrix.trackSize(f0->trackId()), 2);
    ASSERT_EQ(track_matrix.trackSize(f2->trackId()), 1);
}

TEST_F(TrackMatrixTest, getXxxFeature)
{
    f0->setTrackId(0);
    f1->setTrackId(0);
    f2->setTrackId(0);

    /*  C0  C1  C2
     *
     *  f0--f1--f2  trk 0
     */

    track_matrix.add(C0, f0);
    track_matrix.add(C1, f1);
    track_matrix.add(C2, f2);

    ASSERT_EQ(track_matrix.firstFeature(0), f0);
    ASSERT_EQ(track_matrix.lastFeature (0), f2);
    ASSERT_EQ(track_matrix.feature (0, 0 ), f0);
    ASSERT_EQ(track_matrix.feature (0, 1 ), f1);
    ASSERT_EQ(track_matrix.feature (0, 2 ), f2);
    ASSERT_EQ(track_matrix.feature (0, 99), nullptr);
    ASSERT_EQ(track_matrix.feature (99, 2), nullptr);
}

TEST_F(TrackMatrixTest, remove)
{
    track_matrix.add(C0, f0);
    track_matrix.add(C1, f1);
    track_matrix.add(C0, f2);

    /*  C0   C1   C2
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    ASSERT_EQ(track_matrix.numTracks(), 2);

    track_matrix.remove(0);
    ASSERT_EQ(track_matrix.numTracks(), 1);
    ASSERT_EQ(track_matrix.firstFeature(f2->trackId()), f2);

    track_matrix.remove(1);
    ASSERT_EQ(track_matrix.numTracks(), 0);
}

TEST_F(TrackMatrixTest, trackAsVector)
{
    track_matrix.add(C0, f0);
    track_matrix.add(C1, f1);
    track_matrix.add(C2, f2);

    /*  C0   C1   C2
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    std::vector<FeatureBasePtr> vt0 = track_matrix.trackAsVector(0); // get track 0 as vector

    ASSERT_EQ(vt0.size(), 2);
    ASSERT_EQ(vt0[0], f0);
    ASSERT_EQ(vt0.at(1), f1);
}

TEST_F(TrackMatrixTest, track)
{
    track_matrix.add(C0, f0);
    track_matrix.add(C1, f1);
    track_matrix.add(C0, f2);

    /*  C0   C1   C2
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    TrackType t0 = track_matrix.track(0);
    TrackType t2 = track_matrix.track(1);

    ASSERT_EQ(t0.size(), 2);
    ASSERT_EQ(t0.at(C0->getTimeStamp()), f0);
    ASSERT_EQ(t0.at(C1->getTimeStamp()), f1);

    ASSERT_EQ(t2.size(), 1);
    ASSERT_EQ(t2.at(C0->getTimeStamp()), f2);
}

TEST_F(TrackMatrixTest, snapshot)
{
    track_matrix.add(C0, f0);
    track_matrix.add(C1, f1);
    track_matrix.add(C0, f2);

    /*  C0   C1   C2
     *
     *  f0---f1        trk 0
     *  |
     *  f2             trk 1
     */

    SnapshotType s0 = track_matrix.snapshot(C0);
    SnapshotType s1 = track_matrix.snapshot(C1);

    ASSERT_EQ(s0.size(), 2);
    ASSERT_EQ(s0.at(f0->trackId()), f0);
    ASSERT_EQ(s0.at(f2->trackId()), f2);

    ASSERT_EQ(s1.size(), 1);
    ASSERT_EQ(s1.at(f1->trackId()), f1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

