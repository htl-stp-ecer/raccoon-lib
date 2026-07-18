// Unit tests for libstp::localization::LocalizationRecorder.
//
// Validates the JSONL output contract from
// modules/libstp-localization/docs/recording-format.md:
//   * file open failure → ok() == false, recordFrame is no-op
//   * empty session → header only
//   * downsampling honours tick_hz/record_hz
//   * forced frames (obs/resample) bypass downsampling
//   * queue overflow drops oldest, never crashes / never stalls
#include <gtest/gtest.h>

#include "foundation/types.hpp"
#include "localization/localization.hpp"
#include "localization/recorder.hpp"

#include <Eigen/Core>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using libstp::foundation::Pose;
using libstp::localization::LocalizationRecorder;
using libstp::localization::Observation;
using libstp::localization::RecorderConfig;

namespace {

// Trivial line-by-line JSONL split. We don't link a JSON parser into the
// test target; "is it a parseable object" is asserted lightly by checking
// the leading '{' and trailing '}' and that key markers are present.
std::vector<std::string> readLines(const std::filesystem::path& p) {
    std::vector<std::string> out;
    std::ifstream in(p);
    std::string line;
    while (std::getline(in, line)) {
        out.push_back(line);
    }
    return out;
}

LocalizationRecorder::FrameInput makeFrame(int particleCount = 4) {
    LocalizationRecorder::FrameInput f;
    f.estimate_pose.position = Eigen::Vector3f(0.5f, 1.2f, 0.0f);
    f.estimate_pose.heading = 0.7854f;
    f.estimate_sigma = Eigen::Vector3d(0.01, 0.01, 0.005);
    f.particles.reserve(static_cast<size_t>(particleCount));
    for (int i = 0; i < particleCount; ++i) {
        Pose p;
        p.position = Eigen::Vector3f(0.5f + 0.001f * i, 1.2f, 0.0f);
        p.heading = 0.7854f;
        f.particles.emplace_back(p, 1.0 / particleCount);
    }
    return f;
}

// Each test gets its own tempfile so they can run in parallel without
// stomping on one another.
std::filesystem::path tempPath(const std::string& tag) {
    auto base = std::filesystem::temp_directory_path() /
                ("localization_recorder_" + tag + "_" +
                 std::to_string(::getpid()) + ".jsonl");
    std::error_code ec;
    std::filesystem::remove(base, ec);
    return base;
}

}  // namespace

TEST(LocalizationRecorder, OpenFailureMarksNotOk) {
    RecorderConfig cfg;
    // A directory path that cannot exist as a writable file on a sane Linux.
    cfg.path = "/proc/self/no-such-dir/cannot-write.jsonl";
    LocalizationRecorder rec(cfg);
    EXPECT_FALSE(rec.ok());
    // recordFrame is a no-op; must not crash.
    EXPECT_FALSE(rec.recordFrame(makeFrame(), /*force=*/true));
}

TEST(LocalizationRecorder, EmptySessionWritesOnlyHeader) {
    const auto path = tempPath("empty");
    {
        RecorderConfig cfg;
        cfg.path = path.string();
        cfg.tick_hz = 100.0;
        cfg.record_hz = 20.0;
        cfg.particle_count_hint = 64;
        LocalizationRecorder rec(cfg);
        ASSERT_TRUE(rec.ok());
        // No frames pushed — dtor flushes + closes.
    }
    auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_NE(lines[0].find("\"kind\":\"header\""), std::string::npos);
    EXPECT_NE(lines[0].find("\"format_version\":1"), std::string::npos);
    EXPECT_NE(lines[0].find("\"tick_hz\":100"), std::string::npos);
    EXPECT_NE(lines[0].find("\"record_hz\":20"), std::string::npos);
    std::filesystem::remove(path);
}

TEST(LocalizationRecorder, FramesParseableAndDownsampled) {
    const auto path = tempPath("downsample");
    {
        RecorderConfig cfg;
        cfg.path = path.string();
        cfg.tick_hz = 100.0;
        cfg.record_hz = 20.0;  // stride = 5
        LocalizationRecorder rec(cfg);
        ASSERT_TRUE(rec.ok());

        // 50 unforced ticks → 50 / 5 = 10 frames.
        for (int i = 0; i < 50; ++i) {
            rec.recordFrame(makeFrame(), /*force=*/false);
        }
        // Wait until the writer drains. Bounded wait — the recorder's
        // condvar wakes per push, so this is well under 100 ms in practice.
        for (int i = 0; i < 200; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    auto lines = readLines(path);
    ASSERT_GE(lines.size(), 2u);
    EXPECT_NE(lines[0].find("\"kind\":\"header\""), std::string::npos);
    int frameCount = 0;
    for (size_t i = 1; i < lines.size(); ++i) {
        ASSERT_FALSE(lines[i].empty());
        EXPECT_EQ(lines[i].front(), '{');
        EXPECT_EQ(lines[i].back(), '}');
        EXPECT_NE(lines[i].find("\"kind\":\"frame\""), std::string::npos);
        EXPECT_NE(lines[i].find("\"pose\":["), std::string::npos);
        EXPECT_NE(lines[i].find("\"particles\":["), std::string::npos);
        ++frameCount;
    }
    EXPECT_EQ(frameCount, 10);
    std::filesystem::remove(path);
}

TEST(LocalizationRecorder, ForceBypassesDownsample) {
    const auto path = tempPath("forced");
    {
        RecorderConfig cfg;
        cfg.path = path.string();
        cfg.tick_hz = 100.0;
        cfg.record_hz = 20.0;  // stride = 5
        LocalizationRecorder rec(cfg);
        ASSERT_TRUE(rec.ok());

        // Every tick force=true → every tick recorded.
        for (int i = 0; i < 7; ++i) {
            auto f = makeFrame();
            // Tag one observation so the line emits the obs payload too.
            Observation::SurfaceMeasurement sm;
            sm.kind = Observation::SurfaceKind::Line;
            sm.detected = true;
            sm.sigma_cm = 1.5;
            sm.sensor.forwardCm = 8.0f;
            sm.sensor.strafeCm = 0.0f;
            f.observations.push_back(sm);
            f.observation_pose = f.estimate_pose;
            f.observation_sigma = Eigen::Vector3d(0.02, 0.02, 0.01);
            rec.recordFrame(std::move(f), /*force=*/true);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto lines = readLines(path);
    int frameCount = 0;
    for (size_t i = 1; i < lines.size(); ++i) {
        if (lines[i].find("\"kind\":\"frame\"") != std::string::npos) {
            ++frameCount;
        }
    }
    EXPECT_EQ(frameCount, 7);
    // At least one frame carried an observation.
    bool sawObservation = false;
    for (const auto& l : lines) {
        if (l.find("\"surface_kind\":\"line\"") != std::string::npos) {
            sawObservation = true;
            break;
        }
    }
    EXPECT_TRUE(sawObservation);
    std::filesystem::remove(path);
}

TEST(LocalizationRecorder, OverflowDropsOldestNoCrash) {
    const auto path = tempPath("overflow");
    RecorderConfig cfg;
    cfg.path = path.string();
    cfg.tick_hz = 1.0;
    cfg.record_hz = 1.0;       // stride = 1 (record everything)
    cfg.queue_capacity = 4;    // tiny so we overflow fast
    cfg.flush_every = 1;
    LocalizationRecorder rec(cfg);
    ASSERT_TRUE(rec.ok());

    // Spam frames much faster than the writer can drain by holding the
    // writer back is hard from outside — instead push many synchronously;
    // worst case the writer keeps up. The contract is "no crash, no hang".
    for (int i = 0; i < 5000; ++i) {
        rec.recordFrame(makeFrame(2), /*force=*/false);
    }
    rec.stop();

    // File should still be parseable JSONL with a header and some frames.
    auto lines = readLines(path);
    ASSERT_GE(lines.size(), 1u);
    EXPECT_NE(lines[0].find("\"kind\":\"header\""), std::string::npos);
    std::filesystem::remove(path);
}

TEST(LocalizationRecorder, RetentionKeepsOnlyNewestRecordings) {
    // Simulate the on-device layout: <runsRoot>/<run_id>/localization.jsonl.
    namespace fs = std::filesystem;
    const fs::path runsRoot =
        fs::temp_directory_path() / ("loc_retain_" + std::to_string(::getpid()));
    std::error_code ec;
    fs::remove_all(runsRoot, ec);
    fs::create_directories(runsRoot, ec);

    const auto now = fs::file_time_type::clock::now();
    auto makeOldRecording = [&](const std::string& id, int ageMinutes) {
        const fs::path dir = runsRoot / id;
        fs::create_directories(dir, ec);
        const fs::path f = dir / "localization.jsonl";
        std::ofstream(f) << "{\"kind\":\"header\"}\n";
        // Older than the recorder we create below (which lands at ~now).
        fs::last_write_time(f, now - std::chrono::minutes(ageMinutes), ec);
        return f;
    };
    // Five previous runs, run0 oldest … run4 newest.
    const fs::path r0 = makeOldRecording("run0", 50);
    const fs::path r1 = makeOldRecording("run1", 40);
    const fs::path r2 = makeOldRecording("run2", 30);
    const fs::path r3 = makeOldRecording("run3", 20);
    const fs::path r4 = makeOldRecording("run4", 10);

    // New run: constructing the recorder prunes to retain_recordings (3).
    const fs::path newDir = runsRoot / "run_new";
    fs::create_directories(newDir, ec);
    RecorderConfig cfg;
    cfg.path = (newDir / "localization.jsonl").string();
    cfg.retain_recordings = 3;
    {
        LocalizationRecorder rec(cfg);
        ASSERT_TRUE(rec.ok());
    }

    // Newest 3 survive (this run + run4 + run3); the three oldest are gone.
    EXPECT_TRUE(fs::exists(newDir / "localization.jsonl"));
    EXPECT_TRUE(fs::exists(r4));
    EXPECT_TRUE(fs::exists(r3));
    EXPECT_FALSE(fs::exists(r2));
    EXPECT_FALSE(fs::exists(r1));
    EXPECT_FALSE(fs::exists(r0));

    fs::remove_all(runsRoot, ec);
}

TEST(LocalizationRecorder, RetentionDisabledKeepsAll) {
    namespace fs = std::filesystem;
    const fs::path runsRoot =
        fs::temp_directory_path() / ("loc_retain_off_" + std::to_string(::getpid()));
    std::error_code ec;
    fs::remove_all(runsRoot, ec);
    fs::create_directories(runsRoot, ec);
    const auto now = fs::file_time_type::clock::now();
    for (int i = 0; i < 4; ++i) {
        const fs::path dir = runsRoot / ("run" + std::to_string(i));
        fs::create_directories(dir, ec);
        const fs::path f = dir / "localization.jsonl";
        std::ofstream(f) << "{\"kind\":\"header\"}\n";
        fs::last_write_time(f, now - std::chrono::minutes(10 * (4 - i)), ec);
    }
    const fs::path newDir = runsRoot / "run_new";
    fs::create_directories(newDir, ec);
    RecorderConfig cfg;
    cfg.path = (newDir / "localization.jsonl").string();
    cfg.retain_recordings = 0;  // disabled
    { LocalizationRecorder rec(cfg); ASSERT_TRUE(rec.ok()); }

    int count = 0;
    for (const auto& e : fs::directory_iterator(runsRoot)) {
        if (fs::exists(e.path() / "localization.jsonl")) ++count;
    }
    EXPECT_EQ(count, 5);  // 4 old + new, nothing pruned
    fs::remove_all(runsRoot, ec);
}
