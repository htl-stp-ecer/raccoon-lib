#pragma once

// Post-run localization recorder.
//
// Captures particle-filter state to a JSONL file (one object per line)
// for offline visualization in the Web-IDE. Format contract:
//   modules/libstp-localization/docs/recording-format.md
//
// Threading contract (see recording-format.md §"Recorder Threading Contract"):
//   * recordFrame() runs on the filter's tick thread, *while holding the
//     filter mutex*. It MUST be cheap — snapshot copy + queue push, no I/O,
//     no logging from the hot path.
//   * A dedicated writer std::jthread pops frames, serializes JSON, writes
//     lines, and flushes periodically. Joined on stop()/dtor.
//   * Overflow policy: drop OLDEST. The filter is never stalled by writer
//     backpressure — debug recording is best-effort.

#include "foundation/types.hpp"
#include "localization/localization.hpp"
#include "threading/jthread_compat.hpp"

#include <Eigen/Core>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace libstp::localization {

struct RecorderConfig {
    std::string path;                 ///< Absolute target file path.
    double record_hz{20.0};           ///< Target rate after downsampling.
    double tick_hz{100.0};            ///< For the header line.
    int particle_count_hint{128};     ///< For the header line.
    // Pre-formatted JSON fragments embedded verbatim into the header. The
    // caller (Python binding / CLI) is responsible for valid JSON — the
    // recorder does not escape these. Use "null" / "[]" for "unset".
    std::string robot_json{"null"};
    std::string sensors_json{"[]"};
    std::string table_map_json{"null"};
    std::string notes{};
    // Bounded queue size. Default ~50 s headroom at 20 Hz.
    std::size_t queue_capacity{1024};
    // How often (in frames) the writer fsyncs/flushes the stream.
    std::size_t flush_every{32};
};

class LocalizationRecorder {
public:
    // One snapshot worth of filter state. Constructed under the filter mutex.
    struct FrameInput {
        libstp::foundation::Pose estimate_pose{};
        Eigen::Vector3d estimate_sigma{0.0, 0.0, 0.0};
        std::optional<libstp::foundation::Pose> odom_delta;
        // (pose, weight) for each particle. Caller copies in.
        std::vector<std::pair<libstp::foundation::Pose, double>> particles;
        // Observation snapshot collected since last recorded frame. Empty
        // on most ticks; populated only at resync points.
        std::vector<Observation::SurfaceMeasurement> observations;
        std::optional<libstp::foundation::Pose> observation_pose;
        std::optional<Eigen::Vector3d> observation_sigma;
        bool resampled{false};
    };

    explicit LocalizationRecorder(RecorderConfig cfg);
    ~LocalizationRecorder();

    LocalizationRecorder(const LocalizationRecorder&) = delete;
    LocalizationRecorder& operator=(const LocalizationRecorder&) = delete;
    LocalizationRecorder(LocalizationRecorder&&) = delete;
    LocalizationRecorder& operator=(LocalizationRecorder&&) = delete;

    // Submit a frame. Non-blocking. Drops oldest on overflow. Downsamples
    // internally based on record_hz/tick_hz, EXCEPT when ``force`` is true
    // (set this when observations or a resample happened — these must not
    // be dropped). Returns true if the frame was queued.
    bool recordFrame(FrameInput frame, bool force);

    // Idempotent. Flushes and joins the writer thread.
    void stop();

    // False if the file could not be opened. recordFrame becomes a no-op.
    [[nodiscard]] bool ok() const noexcept { return m_ok.load(); }

private:
    void writerLoop(libstp::threading::stop_token stop);
    void writeHeader();
    void serializeFrame(const FrameInput& frame, std::uint64_t t_ns);

    RecorderConfig m_cfg;
    std::uint64_t m_startedAtUnixNs{0};
    std::chrono::system_clock::time_point m_startWall{};

    // Downsampling — recordFrame() side (filter thread).
    std::uint64_t m_tickCounter{0};
    std::uint32_t m_downsampleStride{1};

    // Bounded SPSC-ish queue. We keep it simple: mutex + condvar. The hot
    // path (recordFrame) holds the queue mutex for ≤ one push / one drop.
    std::mutex m_queueMutex;
    std::condition_variable m_queueCv;
    std::deque<FrameInput> m_queue;
    std::uint64_t m_droppedSinceLastReport{0};

    std::atomic<bool> m_ok{false};
    std::atomic<bool> m_stopRequested{false};

    std::ofstream m_out;
    libstp::threading::jthread m_writer;
};

}  // namespace libstp::localization
