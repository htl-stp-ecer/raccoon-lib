//
// Created by eternalRose on 1/29/26.
//

#include <gtest/gtest.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include "Kmeans.hpp"

using namespace libstp::kmeans;

class KMeansTest : public ::testing::Test {
protected:
    KMeans km{10};
};

TEST_F(KMeansTest, TwoDistinctClusters) {
    std::vector<double> data = {100.0, 110.0, 105.0, 1000.0, 1010.0, 1020.0};

    auto result = km.fit(data);
    double centroid1 = result.centroid1;
    double centroid2 = result.centroid2;

    // Ensure centroids are separated
    EXPECT_LT(centroid1, centroid2);
    EXPECT_NEAR(centroid1, 105.0, 5.0);
    EXPECT_NEAR(centroid2, 1010.0, 5.0);
}

TEST_F(KMeansTest, SingleValueReturnsSameCentroids) {
    std::vector<double> data = {42.0};
    auto result = km.fit(data);

    EXPECT_DOUBLE_EQ(result.centroid1, 42.0);
    EXPECT_DOUBLE_EQ(result.centroid2, 42.0);
}

TEST_F(KMeansTest, SortedClusters) {
    std::vector<double> data = {1.0, 2.0, 3.0, 100.0, 110.0, 120.0};

    auto result = km.fit(data);

    EXPECT_LT(result.centroid1, result.centroid2);
    EXPECT_NEAR(result.centroid1, 2.0, 1.0);
    EXPECT_NEAR(result.centroid2, 110.0, 5.0);
}

TEST_F(KMeansTest, OverlappingClusters) {
    std::vector<double> data = {50.0, 55.0, 60.0, 58.0, 62.0};

    auto result = km.fit(data);

    EXPECT_GE(result.centroid1, *std::min_element(data.begin(), data.end()));
    EXPECT_LE(result.centroid2, *std::max_element(data.begin(), data.end()));
    EXPECT_LT(result.centroid1, result.centroid2);
}

TEST_F(KMeansTest, EmptyDataReturnsZeroCentroids) {
    std::vector<double> data{};
    auto result = km.fit(data);

    EXPECT_DOUBLE_EQ(result.centroid1, 0.0);
    EXPECT_DOUBLE_EQ(result.centroid2, 0.0);
}

TEST_F(KMeansTest, SymmetricClusters) {
    std::vector<double> data = {10.0, 12.0, 11.0, 100.0, 102.0, 101.0};

    auto result = km.fit(data);

    EXPECT_NEAR(result.centroid1, 11.0, 1.0);
    EXPECT_NEAR(result.centroid2, 101.0, 1.0);
    EXPECT_LT(result.centroid1, result.centroid2);
}
