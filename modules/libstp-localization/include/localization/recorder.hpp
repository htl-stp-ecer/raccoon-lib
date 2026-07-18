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
//   * A dedicated writer std::jthread pops frames and serializes JSON. Joined
//     on stop()/dtor.
//   * Overflow policy: drop OLDEST. The filter is never stalled by writer
//     backpressure — debug recording is best-effort.
//
// Disk contract (buffer_in_ram, default true):
//   * The writer serializes into a chunked in-RAM buffer and performs NO disk
//     I/O during the run. The whole recording is written to the file once, at
//     stop(). This is deliberate: streaming 100+ MB to the SD card mid-run
//     saturates the card's I/O and starves *other* processes' real-time work
//     (e.g. the STM32 SPI reader's control loop) even though this thread never
//     blocks the filter. Set buffer_in_ram=false to stream+flush periodically
//     instead (crash-resilient, but reintroduces mid-run disk pressure) — use
//     only when debugging a crash that truncates the recording.

#include "foundation/types.hpp"
#include "localization/localization.hpp"
#include "threading/jthread_compat.hpp"

#include <Eigen/Core>
#include <atomic>
#include <chrono>
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
    // How often (in frames) the writer fsyncs/flushes the stream. Only used
    // when buffer_in_ram is false (streaming mode).
    std::size_t flush_every{32};
    // Buffer the entire recording in RAM and write it to disk only at stop(),
    // instead of streaming to the SD card during the run. See "Disk contract"
    // above. Default true (never touch the disk mid-run).
    bool buffer_in_ram{true};
    // Retention: keep only the N most recent localization recordings (this one
    // plus the N-1 previous) and delete older ones at start, so ~150 MB-per-run
    // files don't fill the SD card. Matches sibling recordings by filename
    // across run directories: <runs_root>/*/<this file's name>. 0 or negative
    // disables pruning (keep everything).
    int retain_recordings{3};
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
        // Monotonic capture instant, stamped inside recordFrame() when the frame
        // is accepted. The writer derives the recording's relative time axis from
        // this (not from a re-sampled clock at serialize time), so the axis is both
        // jitter-free and immune to wall-clock steps. Callers leave it default.
        std::chrono::steady_clock::time_point capture_time{};
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

    // Route serialized bytes to the RAM buffer (buffer_in_ram) or straight to
    // the ofstream (streaming). Called only from the writer thread / ctor.
    void emit(const char* data, std::size_t n);
    // Write the RAM buffer to the ofstream. Called once, from stop(), after the
    // writer thread has been joined (so no concurrent access to the buffer).
    void flushRamToDisk();

    // Delete older sibling recordings, keeping only the newest
    // m_cfg.retain_recordings (this run included). Best-effort; never throws.
    // Called once from the constructor after the new file is opened.
    void pruneOldRecordings();

    RecorderConfig m_cfg;
    std::uint64_t m_startedAtUnixNs{0};
    std::chrono::system_clock::time_point m_startWall{};
    // Monotonic t=0 for the relative per-frame time axis. Captured together with
    // m_startWall so the two share an origin, but the frame timeline is measured
    // against this steady anchor and never jumps if the system clock is stepped.
    std::chrono::steady_clock::time_point m_startSteady{};

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

    // RAM buffer (buffer_in_ram mode). Chunked into ~1 MiB blocks so the buffer
    // grows without giant contiguous reallocations / 2× peak memory. Touched
    // only by the writer thread during the run and by stop() after the join.
    static constexpr std::size_t kRamChunkBytes = std::size_t{1} << 20;  // 1 MiB
    std::deque<std::string> m_ramChunks;
    std::string m_ramCurrent;

    libstp::threading::jthread m_writer;
};

}  // namespace libstp::localization
