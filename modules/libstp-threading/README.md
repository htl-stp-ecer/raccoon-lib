# libstp-threading

Threading primitives for raccoon-lib.

## ThreadManager — long-running daemons

Process-wide registry for threads that live for the lifetime of the library.
Each daemon receives a `std::stop_token`; the manager's destructor requests
stop and joins all daemons on library unload.

```cpp
#include "threading/thread_manager.hpp"

libstp::threading::ThreadManager::instance().add_daemon(
    "heartbeat",
    [](std::stop_token stop) {
        using namespace std::chrono_literals;
        while (!stop.stop_requested()) {
            send_heartbeat();
            std::this_thread::sleep_for(100ms);
        }
    });
```

Typical start point: a static guard object in a translation unit linked into
the pybind11 extension module, so the daemon starts when the .so is imported.

## ThreadPool — short-lived tasks

Fixed-size worker pool with futures. Use for fan-out compute work (parallel
motion-controller ticks, sensor preprocessing). Do not use for blocking I/O
— the pool has a fixed worker count and will starve.

```cpp
#include "threading/thread_pool.hpp"

libstp::threading::ThreadPool pool(4);
auto fut = pool.submit([](int x) { return x * 2; }, 21);
int result = fut.get();
```

Nothing in raccoon-lib uses the pool yet; the class is provided as the
canonical way to do task-based parallelism once we need it.
