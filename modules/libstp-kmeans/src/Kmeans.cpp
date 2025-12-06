//
// Created by eternalRose on 12/6/25.
//

#include "../include/Kmeans.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

libstp::kmeans::KMeansResult libstp::kmeans::KMeans::fit(const std::vector<double>& data) const {
    if (data.empty()) {
        return {0.0, 0.0};
    }

    double c1 = *std::ranges::min_element(data);
    double c2 = *std::ranges::max_element(data);

    for (int iter = 0; iter < max_iters; ++iter) {
        std::vector<double> cluster1, cluster2;

        for (double x : data) {
            if (std::abs(x - c1) < std::abs(x - c2))
                cluster1.push_back(x);
            else
                cluster2.push_back(x);
        }

        if (!cluster1.empty())
            c1 = std::accumulate(cluster1.begin(), cluster1.end(), 0.0) / cluster1.size();
        if (!cluster2.empty())
            c2 = std::accumulate(cluster2.begin(), cluster2.end(), 0.0) / cluster2.size();
    }

    if (c1 > c2) std::swap(c1, c2);

    return {c1, c2};
}
