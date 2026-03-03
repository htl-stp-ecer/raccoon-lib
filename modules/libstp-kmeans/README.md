# libstp-kmeans

Contributor reference for the small one-dimensional clustering helper used by higher-level sensor
code.

## Purpose

`libstp-kmeans` implements a fixed two-cluster k-means routine over `std::vector<double>`. It is
currently a narrow utility module rather than a general clustering framework.

This module is suitable when a caller wants to split a one-dimensional sample set into two groups
and only needs the final centroids.

## Architecture

The implementation is intentionally compact:

- [`include/Kmeans.hpp`](include/Kmeans.hpp) declares the result struct and `KMeans` class
- [`src/Kmeans.cpp`](src/Kmeans.cpp) implements the clustering loop
- [`bindings/bindings.cpp`](bindings/bindings.cpp) exposes the same API to Python

Algorithm details from the current implementation:

- input is one-dimensional (`double` samples only)
- the initial centroids are the minimum and maximum sample values
- cluster assignment is based on absolute distance to each centroid
- ties fall into the second cluster because the comparison uses `<` instead of `<=`
- the loop runs for a fixed number of iterations; there is no convergence threshold
- the returned centroids are sorted in ascending order before returning
- empty input returns `{0.0, 0.0}`

## Public API

The public C++ API is:

- `struct libstp::kmeans::KMeansResult`
- `class libstp::kmeans::KMeans`
- `KMeans::KMeans(int maxIterations = 10)`
- `KMeans::fit(const std::vector<double>& data) const`

`KMeansResult` currently reports only `centroid1` and `centroid2`. Cluster membership counts and
assignments are not retained.

## Dependencies

This module has no extra module dependencies in its `CMakeLists.txt`. The implementation uses only
the C++ standard library.

The Python extension depends on `pybind11` through the shared module registration helper.

## Python Bindings

`bindings/bindings.cpp` builds a Python extension module named `kmeans`.

Currently exposed bindings:

- `KMeansResult.centroid1`
- `KMeansResult.centroid2`
- `KMeans(max_iterations=10)`
- `KMeans.fit(data)`

The Python binding mirrors the C++ surface directly; there is no NumPy-specific overload or vector
validation layer in this module.

## Testing

No module-local tests exist in this directory.

If you modify the algorithm, verify at least:

- empty input
- identical-value input
- clearly separated clusters
- borderline cases where points are equidistant from both centroids

## Extension Points

Natural extensions here would be:

- configurable initialization strategies
- convergence-based stopping criteria
- support for more than two clusters
- richer result reporting such as assignments or sample counts
- Python conveniences for array-like inputs

If the module grows beyond the current 1D/two-centroid use case, update this README to explain the
tradeoff between keeping it lightweight and turning it into a more general clustering utility.
