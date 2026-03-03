//
// Created by eternalRose on 1/15/26.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif
#include <string>
#include <vector>

#include "CalibrationType.hpp"

namespace libstp::calibration_store {
    /// Process-wide YAML-backed storage for simple threshold calibrations.
    class CalibrationStore final {
        CalibrationStore() = default;
        ~CalibrationStore();
        /// Backing file path relative to the current working directory.
        std::string FILE_PATH = "./racoon.calibration.yml";
    public:
        CalibrationStore(const CalibrationStore&) = delete;
        CalibrationStore& operator=(const CalibrationStore&) = delete;
        CalibrationStore(CalibrationStore&&) = delete;
        CalibrationStore& operator=(CalibrationStore&&) = delete;

        /// Check whether the backing YAML file exists.
        [[nodiscard]] bool doesFileExist() const;

        /// Return the singleton store instance.
        static CalibrationStore& instance();

        /// Persist one black/white threshold pair for the given calibration type and set.
        void storeReading(float black_tresh, float white_tresh, CalibrationType type,
                          const std::string& set_name = "default") const;

        /// Report whether both expected values are present for the given type and set.
        [[nodiscard]] bool hasReadings(CalibrationType type,
                                       const std::string& set_name = "default") const;

        /// Return `{white_tresh, black_tresh}` or `{0.0, 0.0}` when no reading exists.
        [[nodiscard]] std::vector<float> getReadings(CalibrationType type,
                                                      const std::string& set_name = "default") const;

        /// Return the names of all calibration sets stored for the given type.
        [[nodiscard]] std::vector<std::string> getSetNames(CalibrationType type) const;
    };
}
