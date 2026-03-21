//
// CamSensor - subscribes to raccoon cam_detections_t and provides query API.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace libstp::cam {

    /**
     * Receives colour-blob detections published by raccoon-cam over LCM
     * and exposes a simple query interface.
     *
     * The constructor starts a background receive thread that keeps an
     * internal cache of the latest cam_detections_t message.  All public
     * query methods are thread-safe.
     */
    class CamSensor {
    public:
        CamSensor();
        ~CamSensor();

        CamSensor(const CamSensor&) = delete;
        CamSensor& operator=(const CamSensor&) = delete;

        // ---- Detection queries (from latest cam_detections_t) ----

        /** Return true if at least one blob with the given label was detected. */
        bool isDetected(const std::string& label) const;

        /** Centre-x of the first blob matching \p label (normalised 0-1). */
        float getBlobX(const std::string& label) const;
        /** Centre-y of the first blob matching \p label (normalised 0-1). */
        float getBlobY(const std::string& label) const;
        /** Width of the first blob matching \p label (normalised 0-1). */
        float getBlobWidth(const std::string& label) const;
        /** Height of the first blob matching \p label (normalised 0-1). */
        float getBlobHeight(const std::string& label) const;
        /** Area in pixels of the first blob matching \p label. */
        int getBlobArea(const std::string& label) const;
        /** Detection confidence of the first blob matching \p label (0-1). */
        float getConfidence(const std::string& label) const;

        /** Return all labels present in the latest detection frame. */
        std::vector<std::string> getDetectedLabels() const;

        // ---- Frame dimensions ----

        int getFrameWidth() const;
        int getFrameHeight() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> pImpl;
    };

} // namespace libstp::cam
