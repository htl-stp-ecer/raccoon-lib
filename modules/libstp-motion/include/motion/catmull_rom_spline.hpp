#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

namespace libstp::motion
{
    /**
     * Centripetal Catmull-Rom spline (alpha = 0.5) with arc-length reparameterization.
     *
     * Stores waypoints in 2D and provides arc-length-parameterized evaluation
     * of position and tangent. Centripetal parameterization avoids cusps and
     * self-intersections that uniform Catmull-Rom produces on unevenly-spaced points.
     *
     * Endpoint handling: the first and last control points are duplicated so the
     * curve passes through all user-supplied waypoints.
     */
    class CatmullRomSpline
    {
    public:
        /**
         * Construct a spline through the given waypoints.
         *
         * @param waypoints  At least 2 points in (x, y) meters.
         * @param samples_per_segment  LUT density for arc-length reparameterization.
         * @throws std::invalid_argument if fewer than 2 waypoints.
         */
        explicit CatmullRomSpline(std::vector<Eigen::Vector2d> waypoints,
                                  int samples_per_segment = 200)
        {
            if (waypoints.size() < 2)
                throw std::invalid_argument("CatmullRomSpline requires at least 2 waypoints");

            // Virtual control points via reflection for natural endpoint behavior.
            // Duplicating the same point would cause zero knot intervals in
            // centripetal parameterization (division by zero).
            // Instead: P_virtual[-1] = 2*P[0] - P[1], P_virtual[N] = 2*P[N-1] - P[N-2]
            const auto& first = waypoints.front();
            const auto& second = waypoints[1];
            const auto& last = waypoints.back();
            const auto& second_last = waypoints[waypoints.size() - 2];

            points_.reserve(waypoints.size() + 2);
            points_.push_back(2.0 * first - second);          // virtual start
            for (auto& p : waypoints)
                points_.push_back(std::move(p));
            points_.push_back(2.0 * last - second_last);      // virtual end

            num_segments_ = static_cast<int>(points_.size()) - 3;
            buildArcLengthLUT(samples_per_segment);
        }

        /** Total arc length of the spline in meters. */
        [[nodiscard]] double totalLength() const { return arc_length_lut_.back(); }

        /** Number of user-supplied waypoints. */
        [[nodiscard]] int numWaypoints() const { return num_segments_ + 1; }

        /**
         * Evaluate position at arc-length s (clamped to [0, totalLength]).
         */
        [[nodiscard]] Eigen::Vector2d positionAt(double s) const
        {
            auto [seg, t] = arcLengthToParam(s);
            return evaluateSegment(seg, t);
        }

        /**
         * Evaluate unit tangent at arc-length s.
         */
        [[nodiscard]] Eigen::Vector2d tangentAt(double s) const
        {
            auto [seg, t] = arcLengthToParam(s);
            Eigen::Vector2d d = evaluateSegmentDerivative(seg, t);
            double norm = d.norm();
            if (norm < 1e-12)
            {
                // Degenerate: return direction toward next distinct point
                return Eigen::Vector2d(1.0, 0.0);
            }
            return d / norm;
        }

        /**
         * Find the arc-length of the closest point on the spline to a query point.
         *
         * Uses coarse LUT scan near s_hint then Newton refinement.
         *
         * @param point   Query point in spline frame (meters).
         * @param s_hint  Approximate arc-length (e.g., from previous cycle).
         * @return Arc-length of the nearest point on the spline.
         */
        [[nodiscard]] double findNearestArcLength(const Eigen::Vector2d& point,
                                                   double s_hint) const
        {
            const double total = totalLength();
            if (total < 1e-12) return 0.0;

            // Scan a window around s_hint in the LUT
            const double window = total * 0.25; // 25% of path
            const double s_lo = std::max(0.0, s_hint - window);
            const double s_hi = std::min(total, s_hint + window);

            // Find closest LUT entry in window
            double best_s = s_hint;
            double best_dist_sq = (positionAt(s_hint) - point).squaredNorm();

            const double ds = total / static_cast<double>(arc_length_lut_.size() - 1);
            int i_lo = std::max(0, static_cast<int>(s_lo / ds));
            int i_hi = std::min(static_cast<int>(arc_length_lut_.size()) - 1,
                                static_cast<int>(s_hi / ds) + 1);

            for (int i = i_lo; i <= i_hi; ++i)
            {
                double si = arc_length_lut_[i];
                double d2 = (positionAt(si) - point).squaredNorm();
                if (d2 < best_dist_sq)
                {
                    best_dist_sq = d2;
                    best_s = si;
                }
            }

            // Newton refinement: minimize f(s) = ||P(s) - point||^2
            // f'(s) = 2 * (P(s) - point) . P'(s)
            // Clamp to a neighborhood around best_s to avoid jumping to distant
            // local minima on curved paths.
            const double newton_radius = window;
            const double newton_lo = std::max(0.0, best_s - newton_radius);
            const double newton_hi = std::min(total, best_s + newton_radius);

            double s = best_s;
            for (int iter = 0; iter < 5; ++iter)
            {
                auto [seg, t] = arcLengthToParam(s);
                Eigen::Vector2d p = evaluateSegment(seg, t);
                Eigen::Vector2d dp = evaluateSegmentDerivative(seg, t);
                double dp_norm = dp.norm();
                if (dp_norm < 1e-12) break;

                Eigen::Vector2d diff = p - point;
                double f_prime = 2.0 * diff.dot(dp);
                // Approximate f'' ≈ 2 * dp . dp (ignore curvature term)
                double f_double_prime = 2.0 * dp.squaredNorm();
                if (std::abs(f_double_prime) < 1e-12) break;

                double step = f_prime / f_double_prime;
                s = std::clamp(s - step, newton_lo, newton_hi);
            }

            // Accept Newton result only if it's actually closer
            double newton_dist_sq = (positionAt(s) - point).squaredNorm();
            return (newton_dist_sq <= best_dist_sq) ? s : best_s;
        }

    private:
        // Catmull-Rom centripetal parameterization helpers
        static double knotValue(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
                                double t_prev)
        {
            // Centripetal: alpha = 0.5 → t_i = t_{i-1} + |p_i - p_{i-1}|^0.5
            double d = (p1 - p0).norm();
            return t_prev + std::sqrt(std::max(d, 1e-12));
        }

        /**
         * Evaluate position on segment `seg` at local parameter `t` in [0, 1].
         *
         * Uses the Barry-Goldman algorithm for Catmull-Rom evaluation:
         * three lerps on knot intervals produce the final point.
         */
        [[nodiscard]] Eigen::Vector2d evaluateSegment(int seg, double t) const
        {
            const Eigen::Vector2d& P0 = points_[seg];
            const Eigen::Vector2d& P1 = points_[seg + 1];
            const Eigen::Vector2d& P2 = points_[seg + 2];
            const Eigen::Vector2d& P3 = points_[seg + 3];

            // Centripetal knot values
            double t0 = 0.0;
            double t1 = knotValue(P0, P1, t0);
            double t2 = knotValue(P1, P2, t1);
            double t3 = knotValue(P2, P3, t2);

            // Map local t ∈ [0,1] to knot space [t1, t2]
            double u = t1 + t * (t2 - t1);

            // Barry-Goldman recursive evaluation
            auto lerp = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                           double ta, double tb, double u_) -> Eigen::Vector2d
            {
                double denom = tb - ta;
                if (std::abs(denom) < 1e-12) return a;
                double alpha = (u_ - ta) / denom;
                return (1.0 - alpha) * a + alpha * b;
            };

            Eigen::Vector2d A1 = lerp(P0, P1, t0, t1, u);
            Eigen::Vector2d A2 = lerp(P1, P2, t1, t2, u);
            Eigen::Vector2d A3 = lerp(P2, P3, t2, t3, u);

            Eigen::Vector2d B1 = lerp(A1, A2, t0, t2, u);
            Eigen::Vector2d B2 = lerp(A2, A3, t1, t3, u);

            return lerp(B1, B2, t1, t2, u);
        }

        /**
         * Evaluate the derivative (dp/dt in local parameter) on segment `seg`.
         *
         * Computed via central finite difference for numerical robustness.
         */
        [[nodiscard]] Eigen::Vector2d evaluateSegmentDerivative(int seg, double t) const
        {
            constexpr double h = 1e-5;
            double t_lo = std::max(0.0, t - h);
            double t_hi = std::min(1.0, t + h);
            double dt = t_hi - t_lo;
            if (dt < 1e-12) return Eigen::Vector2d(0.0, 0.0);
            return (evaluateSegment(seg, t_hi) - evaluateSegment(seg, t_lo)) / dt;
        }

        /**
         * Build cumulative arc-length lookup table by densely sampling the spline.
         */
        void buildArcLengthLUT(int samples_per_segment)
        {
            int total_samples = num_segments_ * samples_per_segment + 1;
            arc_length_lut_.resize(total_samples);
            param_lut_.resize(total_samples);

            arc_length_lut_[0] = 0.0;
            param_lut_[0] = {0, 0.0};

            Eigen::Vector2d prev = evaluateSegment(0, 0.0);
            int idx = 1;

            for (int seg = 0; seg < num_segments_; ++seg)
            {
                for (int i = 1; i <= samples_per_segment; ++i)
                {
                    double t = static_cast<double>(i) / samples_per_segment;
                    Eigen::Vector2d p = evaluateSegment(seg, t);
                    double chord = (p - prev).norm();
                    arc_length_lut_[idx] = arc_length_lut_[idx - 1] + chord;
                    param_lut_[idx] = {seg, t};
                    prev = p;
                    ++idx;
                }
            }
        }

        /**
         * Map arc-length s to (segment, local_t) via binary search in the LUT.
         */
        [[nodiscard]] std::pair<int, double> arcLengthToParam(double s) const
        {
            s = std::clamp(s, 0.0, totalLength());

            // Binary search for the LUT interval containing s
            auto it = std::lower_bound(arc_length_lut_.begin(), arc_length_lut_.end(), s);
            int idx = static_cast<int>(it - arc_length_lut_.begin());

            if (idx == 0) return param_lut_[0];
            if (idx >= static_cast<int>(arc_length_lut_.size()))
                return param_lut_.back();

            // Linearly interpolate between adjacent LUT entries
            double s0 = arc_length_lut_[idx - 1];
            double s1 = arc_length_lut_[idx];
            double frac = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;

            auto [seg0, t0] = param_lut_[idx - 1];
            auto [seg1, t1] = param_lut_[idx];

            if (seg0 == seg1)
            {
                return {seg0, t0 + frac * (t1 - t0)};
            }
            else
            {
                // Cross-segment boundary: snap to closer entry
                return (frac < 0.5) ? param_lut_[idx - 1] : param_lut_[idx];
            }
        }

        std::vector<Eigen::Vector2d> points_;  // [duplicate_first, wp0, wp1, ..., wpN, duplicate_last]
        int num_segments_{0};                   // number of spline segments = waypoints - 1

        // Arc-length LUT: arc_length_lut_[i] = cumulative arc length at param_lut_[i]
        std::vector<double> arc_length_lut_;
        std::vector<std::pair<int, double>> param_lut_;  // (segment_index, local_t)
    };
}
