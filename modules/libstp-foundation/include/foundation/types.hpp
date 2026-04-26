#pragma once

#include <numbers>

#include "pch.hpp"

namespace libstp::foundation {

    using Vector3f = Eigen::Vector3f;

    /// Position plus planar heading used by several motion-related modules.
    struct Pose {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector3f position{Vector3f::Zero()};
        float heading{0.0f}; // radians, 0 = +X, positive = CCW
    };

    /**
     * Unified chassis velocity type for all robot motion.
     * Uses consistent naming: vx (forward), vy (lateral), wz (angular around z-axis).
     */
    struct ChassisVelocity {
        double vx{0.0};  ///< Forward velocity (m/s), positive = forward
        double vy{0.0};  ///< Lateral velocity (m/s), positive = right
        double wz{0.0};  ///< Angular velocity (rad/s), positive = counter-clockwise

        ChassisVelocity() = default;
        ChassisVelocity(double vx_, double vy_, double wz_) : vx(vx_), vy(vy_), wz(wz_) {}
    };

    // ============================================================================
    // Strongly-typed unit wrappers to prevent unit conversion errors
    // ============================================================================

    /**
     * Strongly-typed distance in meters.
     * Use Meters::from_cm() or Meters::from_mm() to convert from other units.
     */
    struct Meters {
        double value{0.0};

        constexpr Meters() = default;
        constexpr explicit Meters(double m) : value(m) {}

        static constexpr Meters from_cm(double cm) { return Meters{cm * 0.01}; }
        static constexpr Meters from_mm(double mm) { return Meters{mm * 0.001}; }

        constexpr double to_cm() const { return value * 100.0; }
        constexpr double to_mm() const { return value * 1000.0; }

        constexpr operator double() const { return value; }

        constexpr Meters operator-() const { return Meters{-value}; }
        constexpr Meters operator+(Meters other) const { return Meters{value + other.value}; }
        constexpr Meters operator-(Meters other) const { return Meters{value - other.value}; }
        constexpr Meters operator*(double scale) const { return Meters{value * scale}; }
        constexpr Meters operator/(double scale) const { return Meters{value / scale}; }
    };

    /**
     * Strongly-typed velocity in meters per second.
     */
    struct MetersPerSecond {
        double value{0.0};

        constexpr MetersPerSecond() = default;
        constexpr explicit MetersPerSecond(double mps) : value(mps) {}

        static constexpr MetersPerSecond from_cm_per_sec(double cmps) {
            return MetersPerSecond{cmps * 0.01};
        }

        constexpr operator double() const { return value; }

        constexpr MetersPerSecond operator-() const { return MetersPerSecond{-value}; }
        constexpr MetersPerSecond operator*(double scale) const { return MetersPerSecond{value * scale}; }
    };

    /**
     * Strongly-typed angle in radians.
     * Use Radians::from_deg() to convert from degrees.
     */
    struct Radians {
        double value{0.0};

        constexpr Radians() = default;
        constexpr explicit Radians(double rad) : value(rad) {}

        static constexpr Radians from_deg(double deg) {
            return Radians{deg * std::numbers::pi / 180.0};
        }

        constexpr double to_deg() const {
            return value * 180.0 / std::numbers::pi;
        }

        constexpr operator double() const { return value; }

        constexpr Radians operator-() const { return Radians{-value}; }
        constexpr Radians operator+(Radians other) const { return Radians{value + other.value}; }
        constexpr Radians operator-(Radians other) const { return Radians{value - other.value}; }
    };

    /**
     * Strongly-typed angular velocity in radians per second.
     */
    struct RadiansPerSecond {
        double value{0.0};

        constexpr RadiansPerSecond() = default;
        constexpr explicit RadiansPerSecond(double rps) : value(rps) {}

        static constexpr RadiansPerSecond from_deg_per_sec(double dps) {
            return RadiansPerSecond{dps * std::numbers::pi / 180.0};
        }

        constexpr operator double() const { return value; }

        constexpr RadiansPerSecond operator-() const { return RadiansPerSecond{-value}; }
        constexpr RadiansPerSecond operator*(double scale) const { return RadiansPerSecond{value * scale}; }
    };

    // User-defined literals for convenient construction.
    namespace literals {
        constexpr Meters operator""_m(long double value) { return Meters{static_cast<double>(value)}; }
        constexpr Meters operator""_cm(long double value) { return Meters::from_cm(static_cast<double>(value)); }
        constexpr Meters operator""_mm(long double value) { return Meters::from_mm(static_cast<double>(value)); }
        constexpr MetersPerSecond operator""_mps(long double value) { return MetersPerSecond{static_cast<double>(value)}; }
        constexpr Radians operator""_rad(long double value) { return Radians{static_cast<double>(value)}; }
        constexpr Radians operator""_deg(long double value) { return Radians::from_deg(static_cast<double>(value)); }
        constexpr RadiansPerSecond operator""_rad_s(long double value) { return RadiansPerSecond{static_cast<double>(value)}; }
    }


} // namespace libstp::foundation
