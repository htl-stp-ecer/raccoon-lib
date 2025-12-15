#ifndef KMEANS_HPP
#define KMEANS_HPP

#include <vector>

namespace libstp::kmeans {
    struct KMeansResult {
        double centroid1;
        double centroid2;
    };

    class KMeans {
    public:
        KMeans(int maxIterations = 10) : max_iters(maxIterations) {}

        KMeansResult fit(const std::vector<double>& data) const;

    private:
        int max_iters;
    };
}


#endif
