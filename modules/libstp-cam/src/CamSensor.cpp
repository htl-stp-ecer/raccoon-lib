//
// CamSensor implementation - background LCM subscriber + cached blob map.
//

#include "CamSensor.hpp"

#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <raccoon/Transport.h>
#include <raccoon/Channels.h>
#include <raccoon/cam_detections_t.hpp>
#include <raccoon/cam_blob_t.hpp>

#include "foundation/logging.hpp"

namespace libstp::cam {

// ---- Internal blob cache entry ----
struct BlobData {
    float x          = 0.0f;
    float y          = 0.0f;
    float width      = 0.0f;
    float height     = 0.0f;
    float area       = 0.0f;
    float confidence = 0.0f;
};

// ---- Impl (PIMPL) ----
struct CamSensor::Impl {
    raccoon::Transport transport;
    std::thread        spinThread;
    std::atomic<bool>  running{false};

    mutable std::mutex                            mtx;
    std::unordered_map<std::string, BlobData>     blobs;
    int                                           frameWidth  = 0;
    int                                           frameHeight = 0;

    Impl() : transport(raccoon::Transport::create()) {}

    void start() {
        transport.subscribe<raccoon::cam_detections_t>(
            raccoon::Channels::CAM_DETECTIONS,
            [this](const raccoon::cam_detections_t& msg) {
                std::lock_guard<std::mutex> lock(mtx);
                blobs.clear();
                frameWidth  = msg.frame_width;
                frameHeight = msg.frame_height;
                for (int i = 0; i < msg.num_detections; ++i) {
                    const auto& d = msg.detections[i];
                    BlobData bd;
                    bd.x          = d.x;
                    bd.y          = d.y;
                    bd.width      = d.width;
                    bd.height     = d.height;
                    bd.area       = d.area;
                    bd.confidence = d.confidence;
                    blobs[d.label] = bd;
                }
            });

        running = true;
        spinThread = std::thread([this]() {
            while (running) {
                transport.spinOnce(50);
            }
        });
    }

    void stop() {
        running = false;
        transport.stop();
        if (spinThread.joinable()) {
            spinThread.join();
        }
    }

    const BlobData* findBlob(const std::string& label) const {
        auto it = blobs.find(label);
        return (it != blobs.end()) ? &it->second : nullptr;
    }
};

// ---- Public API ----

CamSensor::CamSensor() : pImpl(std::make_unique<Impl>()) {
    LIBSTP_LOG_INFO("CamSensor: subscribing to " + std::string(raccoon::Channels::CAM_DETECTIONS));
    pImpl->start();
}

CamSensor::~CamSensor() {
    pImpl->stop();
}

bool CamSensor::isDetected(const std::string& label) const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->findBlob(label) != nullptr;
}

float CamSensor::getBlobX(const std::string& label) const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    auto* b = pImpl->findBlob(label);
    return b ? b->x : 0.0f;
}

float CamSensor::getBlobY(const std::string& label) const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    auto* b = pImpl->findBlob(label);
    return b ? b->y : 0.0f;
}

float CamSensor::getBlobWidth(const std::string& label) const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    auto* b = pImpl->findBlob(label);
    return b ? b->width : 0.0f;
}

float CamSensor::getBlobHeight(const std::string& label) const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    auto* b = pImpl->findBlob(label);
    return b ? b->height : 0.0f;
}

int CamSensor::getBlobArea(const std::string& label) const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    auto* b = pImpl->findBlob(label);
    return b ? static_cast<int>(b->area) : 0;
}

float CamSensor::getConfidence(const std::string& label) const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    auto* b = pImpl->findBlob(label);
    return b ? b->confidence : 0.0f;
}

std::vector<std::string> CamSensor::getDetectedLabels() const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    std::vector<std::string> labels;
    labels.reserve(pImpl->blobs.size());
    for (const auto& [label, _] : pImpl->blobs) {
        labels.push_back(label);
    }
    return labels;
}

int CamSensor::getFrameWidth() const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->frameWidth;
}

int CamSensor::getFrameHeight() const {
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->frameHeight;
}

} // namespace libstp::cam
