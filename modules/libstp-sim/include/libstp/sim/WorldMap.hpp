#pragma once

// libstp-sim retired its WorldMap copy in Phase 1 commit B; the canonical
// implementation now lives in libstp-map. This shim keeps existing
// `libstp::sim::WorldMap` / `MapSegment` / `FtmapParseError` callers and
// the `raccoon.sim` bindings compiling without churn.
#include "libstp/map/MapSegment.hpp"
#include "libstp/map/WorldMap.hpp"

namespace libstp::sim
{
    using WorldMap = libstp::map::WorldMap;
    using MapSegment = libstp::map::MapSegment;
    using FtmapParseError = libstp::map::FtmapParseError;
}
