#ifndef KMEANS_HPP
#define KMEANS_HPP

#include <vector>

namespace libstp::kmeans {
    /// Result of the current two-centroid 1D clustering pass.
    struct KMeansResult {
        double centroid1;
        double centroid2;
    };

    /// Fixed-iteration k-means for one-dimensional data with exactly two clusters.
    class KMeans {
    public:
        KMeans(int maxIterations = 10) : max_iters(maxIterations) {}

        /// Cluster `data` and return the two final centroids in ascending order.
        KMeansResult fit(const std::vector<double>& data) const;

    private:
        int max_iters;
    };
}


#endif
