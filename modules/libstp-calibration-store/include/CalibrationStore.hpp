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
    class CalibrationStore final {
        CalibrationStore() = default;
        ~CalibrationStore();
        std::string FILE_PATH = "./racoon.calibration.yml";
    public:
        CalibrationStore(const CalibrationStore&) = delete;
        CalibrationStore& operator=(const CalibrationStore&) = delete;
        CalibrationStore(CalibrationStore&&) = delete;
        CalibrationStore& operator=(CalibrationStore&&) = delete;
        [[nodiscard]] bool doesFileExist() const;
        static CalibrationStore& instance();
        void storeReading(float black_tresh, float white_tresh, CalibrationType type) const;
        [[nodiscard]] bool hasReadings(CalibrationType type) const;

        [[nodiscard]] std::vector<float> getReadings(CalibrationType type) const;
    };
}
