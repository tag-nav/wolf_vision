/*
 * gtest_tracks.cpp
 *
 *  Created on: Mar 23, 2018
 *      Author: jsola
 */

#include "feature_track_map.h"

#include "utils_gtest.h"

using namespace wolf;

Eigen::Vector2s m;
Eigen::Matrix2s m_cov;

TEST(FeatureTrackMap, add)
{
    FeatureTrackMap tracks;

    FeatureBasePtr f = std::make_shared<FeatureBase>("BASE", m, m_cov);
    f->setTrackId(f->id());

    tracks.add(f);

    ASSERT_EQ(tracks.get(f->trackId()).size(), 1);
}

TEST(FeatureTrackMap, size)
{
    FeatureTrackMap tracks;

    FeatureBasePtr f0 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f1 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f2 = std::make_shared<FeatureBase>("BASE", m, m_cov);

    size_t track_id = f0->id();
    f0->setTrackId(track_id);
    f1->setTrackId(track_id);
    f2->setTrackId(f2->id());

    tracks.add(f0);
    tracks.add(f1);

    ASSERT_EQ(tracks.size(), 1);

    tracks.add(f2);

    ASSERT_EQ(tracks.size(), 2);
}

TEST(FeatureTrackMap, trackSize)
{
    FeatureTrackMap tracks;

    FeatureBasePtr f0 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f1 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f2 = std::make_shared<FeatureBase>("BASE", m, m_cov);

    size_t track_id = f0->id();
    f0->setTrackId(track_id);
    f1->setTrackId(track_id);
    f2->setTrackId(f2->id());

    tracks.add(f0);
    tracks.add(f1);
    tracks.add(f2);

    ASSERT_EQ(tracks.trackSize(track_id), 2);
    ASSERT_EQ(tracks.trackSize(f2->id()), 1);
}

TEST(FeatureTrackMap, getXxxFeature)
{
    FeatureTrackMap tracks;

    FeatureBasePtr f0 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f1 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f2 = std::make_shared<FeatureBase>("BASE", m, m_cov);

    // 3 features on the same track
    size_t track_id = f0->id();
    f0->setTrackId(track_id);
    f1->setTrackId(track_id);
    f2->setTrackId(track_id);

    tracks.add(f0);
    tracks.add(f1);
    tracks.add(f2);

    ASSERT_EQ(tracks.getFirstFeature(track_id), f0);
    ASSERT_EQ(tracks.getLastFeature (track_id), f2);
    ASSERT_EQ(tracks.getFeature(track_id, 0), f0);
    ASSERT_EQ(tracks.getFeature(track_id, 1), f1);
    ASSERT_EQ(tracks.getFeature(track_id, 2), f2);
    ASSERT_EQ(tracks.getFeature(track_id, 99), nullptr);
    ASSERT_EQ(tracks.getFeature(99, 2), nullptr);
}

TEST(FeatureTrackMap, remove)
{
    FeatureTrackMap tracks;

    FeatureBasePtr f0 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f1 = std::make_shared<FeatureBase>("BASE", m, m_cov);
    FeatureBasePtr f2 = std::make_shared<FeatureBase>("BASE", m, m_cov);

    size_t track_id = f0->id();
    f0->setTrackId(track_id);
    f1->setTrackId(track_id);
    f2->setTrackId(f2->id());

    tracks.add(f0);
    tracks.add(f1);

    ASSERT_EQ(tracks.size(), 1);

    tracks.add(f2);

    ASSERT_EQ(tracks.size(), 2);

    tracks.remove(track_id);
    ASSERT_EQ(tracks.size(), 1);
    ASSERT_EQ(tracks.getFirstFeature(f2->id()), f2);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
