#include "localization/recorder.hpp"

// JSONL writer for offline visualization. The schema lives in
// modules/libstp-localization/docs/recording-format.md — bump format_version
// before changing field semantics.
//
// Pose::position is in METERS (the same units used by the filter's
// process_translation_noise_m and by the cm-conversion in
// computeParticleLogWeight, which multiplies by 100). Units in the JSON
// follow the format-spec contract: position=m, heading=rad,
// sensor_offset=cm.

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

namespace libstp::localization {

namespace {

constexpr int kFormatVersion = 1;

// Stringify a double with full precision. Handles non-finite values by
// emitting JSON-legal substitutes (we use 0 for NaN/Inf; JSON has no native
// NaN/Inf representation and consumers prefer numbers).
inline void appendNumber(std::string& out, double value) {
    if (!std::isfinite(value)) {
        out += "0";
        return;
    }
    char buf[32];
    // 17 significant digits round-trips a double; 9 is enough for floats.
    int n = std::snprintf(buf, sizeof(buf), "%.17g", value);
    if (n > 0) {
        out.append(buf, static_cast<size_t>(n));
    } else {
        out += "0";
    }
}

inline void appendNumber(std::string& out, float value) {
    appendNumber(out, static_cast<double>(value));
}

inline std::uint64_t systemClockUnixNs(std::chrono::system_clock::time_point tp) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count());
}

// Emit a JSON string literal. Used only for the fixed-vocabulary "notes" /
// surface_kind values — the rest of the header is pre-formatted JSON passed
// verbatim from Python.
inline void appendJsonString(std::string& out, std::string_view s) {
    out += '"';
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

}  // namespace

LocalizationRecorder::LocalizationRecorder(RecorderConfig cfg) : m_cfg(std::move(cfg)) {
    if (m_cfg.queue_capacity == 0) {
        m_cfg.queue_capacity = 1024;
    }
    if (m_cfg.flush_every == 0) {
        m_cfg.flush_every = 32;
    }
    if (!std::isfinite(m_cfg.record_hz) || m_cfg.record_hz <= 0.0) {
        m_cfg.record_hz = 20.0;
    }
    if (!std::isfinite(m_cfg.tick_hz) || m_cfg.tick_hz <= 0.0) {
        m_cfg.tick_hz = 100.0;
    }
    const double ratio = m_cfg.tick_hz / m_cfg.record_hz;
    m_downsampleStride = ratio <= 1.0
                             ? 1u
                             : static_cast<std::uint32_t>(std::ceil(ratio));

    // Best-effort parent-directory creation. Failure here only matters if
    // the ofstream below also fails; we report through m_ok in that case.
    try {
        const std::filesystem::path p(m_cfg.path);
        if (p.has_parent_path()) {
            std::error_code ec;
            std::filesystem::create_directories(p.parent_path(), ec);
            // Ignore ec — the ofstream open will surface real failures.
        }
    } catch (...) {
        // Defensive: any pathological path is handled by ofstream::open below.
    }

    m_out.open(m_cfg.path, std::ios::out | std::ios::trunc);
    if (!m_out.is_open()) {
        // One-shot warning; recordFrame() is now a no-op.
        std::cerr << "[localization::Recorder] failed to open '" << m_cfg.path
                  << "': " << std::strerror(errno) << "\n";
        m_ok = false;
        return;
    }

    m_startWall = std::chrono::system_clock::now();
    m_startedAtUnixNs = systemClockUnixNs(m_startWall);
    writeHeader();
    if (!m_out.good()) {
        std::cerr << "[localization::Recorder] failed to write header to '"
                  << m_cfg.path << "'\n";
        m_out.close();
        m_ok = false;
        return;
    }

    m_ok = true;
    m_writer = libstp::threading::jthread(
        [this](libstp::threading::stop_token stop) { writerLoop(stop); });
}

LocalizationRecorder::~LocalizationRecorder() {
    stop();
}

bool LocalizationRecorder::recordFrame(FrameInput frame, bool force) {
    if (!m_ok.load(std::memory_order_relaxed) || m_stopRequested.load(std::memory_order_relaxed)) {
        return false;
    }

    // Downsample: keep every Nth tick OR any tick the caller marks as
    // observation/resample-bearing (force=true). Resync points must not
    // be lost to downsampling.
    const std::uint64_t tick = m_tickCounter++;
    if (!force && (tick % m_downsampleStride) != 0) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(m_queueMutex);
        if (m_queue.size() >= m_cfg.queue_capacity) {
            // Drop oldest. Filter must never stall on writer backpressure.
            m_queue.pop_front();
            ++m_droppedSinceLastReport;
        }
        m_queue.push_back(std::move(frame));
    }
    m_queueCv.notify_one();
    return true;
}

void LocalizationRecorder::stop() {
    bool expected = false;
    if (!m_stopRequested.compare_exchange_strong(expected, true)) {
        return;  // already stopped
    }
    // Wake the writer so it can observe the stop_token.
    m_queueCv.notify_all();
    if (m_writer.joinable()) {
        m_writer.request_stop();
        m_queueCv.notify_all();
        m_writer.join();
    }
    if (m_out.is_open()) {
        m_out.flush();
        m_out.close();
    }
    m_ok = false;
}

void LocalizationRecorder::writeHeader() {
    std::string line;
    line.reserve(512);
    line += "{\"kind\":\"header\",\"format_version\":";
    line += std::to_string(kFormatVersion);
    line += ",\"started_at_unix_ns\":";
    line += std::to_string(m_startedAtUnixNs);
    line += ",\"tick_hz\":";
    appendNumber(line, m_cfg.tick_hz);
    line += ",\"record_hz\":";
    appendNumber(line, m_cfg.record_hz);
    line += ",\"particle_count\":";
    line += std::to_string(m_cfg.particle_count_hint);
    line += ",\"units\":{\"position\":\"m\",\"heading\":\"rad\",\"sensor_offset\":\"cm\"}";
    line += ",\"robot\":";
    line += m_cfg.robot_json.empty() ? "null" : m_cfg.robot_json;
    line += ",\"sensors\":";
    line += m_cfg.sensors_json.empty() ? "[]" : m_cfg.sensors_json;
    line += ",\"table_map\":";
    line += m_cfg.table_map_json.empty() ? "null" : m_cfg.table_map_json;
    line += ",\"notes\":";
    appendJsonString(line, m_cfg.notes);
    line += "}\n";
    m_out.write(line.data(), static_cast<std::streamsize>(line.size()));
}

void LocalizationRecorder::serializeFrame(const FrameInput& frame, std::uint64_t t_ns) {
    std::string line;
    // Rough sizing: header + 128 particles × ~80 chars.
    line.reserve(128 + frame.particles.size() * 96);
    line += "{\"kind\":\"frame\",\"t_ns\":";
    line += std::to_string(t_ns);

    line += ",\"pose\":[";
    appendNumber(line, frame.estimate_pose.position.x());
    line += ',';
    appendNumber(line, frame.estimate_pose.position.y());
    line += ',';
    appendNumber(line, frame.estimate_pose.heading);
    line += ']';

    line += ",\"sigma\":[";
    appendNumber(line, frame.estimate_sigma.x());
    line += ',';
    appendNumber(line, frame.estimate_sigma.y());
    line += ',';
    appendNumber(line, frame.estimate_sigma.z());
    line += ']';

    line += ",\"odom_delta\":";
    if (frame.odom_delta.has_value()) {
        line += '[';
        appendNumber(line, frame.odom_delta->position.x());
        line += ',';
        appendNumber(line, frame.odom_delta->position.y());
        line += ',';
        appendNumber(line, frame.odom_delta->heading);
        line += ']';
    } else {
        line += "[]";
    }

    line += ",\"particles\":[";
    for (size_t i = 0; i < frame.particles.size(); ++i) {
        if (i != 0) line += ',';
        const auto& [pose, w] = frame.particles[i];
        line += '[';
        appendNumber(line, pose.position.x());
        line += ',';
        appendNumber(line, pose.position.y());
        line += ',';
        appendNumber(line, pose.heading);
        line += ',';
        appendNumber(line, w);
        line += ']';
    }
    line += ']';

    line += ",\"observations\":[";
    if (!frame.observations.empty()) {
        const double pose_x = frame.observation_pose ? frame.observation_pose->position.x() : 0.0;
        const double pose_y = frame.observation_pose ? frame.observation_pose->position.y() : 0.0;
        const double pose_h = frame.observation_pose ? frame.observation_pose->heading : 0.0;
        const bool has_pose = frame.observation_pose.has_value();
        const double sig_x = frame.observation_sigma ? frame.observation_sigma->x() : 0.0;
        const double sig_y = frame.observation_sigma ? frame.observation_sigma->y() : 0.0;
        const double sig_h = frame.observation_sigma ? frame.observation_sigma->z() : 0.0;
        const bool has_sig = frame.observation_sigma.has_value();

        for (size_t i = 0; i < frame.observations.size(); ++i) {
            if (i != 0) line += ',';
            const auto& m = frame.observations[i];
            line += "{\"surface_kind\":";
            line += (m.kind == Observation::SurfaceKind::Line ? "\"line\"" : "\"wall\"");
            line += ",\"detected\":";
            line += (m.detected ? "true" : "false");
            line += ",\"sensor_offset_cm\":[";
            appendNumber(line, m.sensor.forwardCm);
            line += ',';
            appendNumber(line, m.sensor.strafeCm);
            line += "],\"sigma_cm\":";
            appendNumber(line, m.sigma_cm);
            line += ",\"measured_distance_cm\":null";
            line += ",\"pose\":";
            if (has_pose) {
                line += '[';
                appendNumber(line, pose_x);
                line += ',';
                appendNumber(line, pose_y);
                line += ',';
                appendNumber(line, pose_h);
                line += ']';
            } else {
                line += "null";
            }
            line += ",\"pose_sigma\":";
            if (has_sig) {
                line += '[';
                appendNumber(line, sig_x);
                line += ',';
                appendNumber(line, sig_y);
                line += ',';
                appendNumber(line, sig_h);
                line += ']';
            } else {
                line += "null";
            }
            line += '}';
        }
    }
    line += ']';

    line += ",\"resampled\":";
    line += (frame.resampled ? "true" : "false");

    line += "}\n";
    m_out.write(line.data(), static_cast<std::streamsize>(line.size()));
}

void LocalizationRecorder::writerLoop(libstp::threading::stop_token stop) {
    std::size_t writtenSinceFlush = 0;
    while (true) {
        FrameInput frame;
        bool haveFrame = false;
        std::uint64_t droppedReport = 0;
        {
            std::unique_lock<std::mutex> lk(m_queueMutex);
            m_queueCv.wait(lk, [&]() {
                return !m_queue.empty() || stop.stop_requested() ||
                       m_stopRequested.load(std::memory_order_relaxed);
            });
            if (!m_queue.empty()) {
                frame = std::move(m_queue.front());
                m_queue.pop_front();
                haveFrame = true;
            }
            if (m_droppedSinceLastReport > 0) {
                droppedReport = m_droppedSinceLastReport;
                m_droppedSinceLastReport = 0;
            }
        }

        if (droppedReport > 0) {
            std::cerr << "[localization::Recorder] queue overflow: dropped "
                      << droppedReport << " frame(s)\n";
        }

        if (haveFrame) {
            const auto now_ns = systemClockUnixNs(std::chrono::system_clock::now());
            const std::uint64_t t_ns = now_ns >= m_startedAtUnixNs ? now_ns - m_startedAtUnixNs : 0;
            serializeFrame(frame, t_ns);
            if (++writtenSinceFlush >= m_cfg.flush_every) {
                m_out.flush();
                writtenSinceFlush = 0;
            }
        }

        // On stop, drain everything that's left, then exit.
        if (stop.stop_requested() || m_stopRequested.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> lk(m_queueMutex);
            while (!m_queue.empty()) {
                FrameInput f = std::move(m_queue.front());
                m_queue.pop_front();
                const auto now_ns = systemClockUnixNs(std::chrono::system_clock::now());
                const std::uint64_t t_ns =
                    now_ns >= m_startedAtUnixNs ? now_ns - m_startedAtUnixNs : 0;
                serializeFrame(f, t_ns);
            }
            m_out.flush();
            return;
        }
    }
}

}  // namespace libstp::localization
