# libstp-async

Contributor reference for the small coroutine wrapper used to bridge C++ async-style producers into
Python-facing code.

## Purpose

`libstp-async` currently provides one abstraction: `libstp::async::AsyncAlgorithm<T>`. It wraps a
C++20 coroutine handle and exposes a minimal interface for:

- stepping a coroutine manually with `advance()`
- reading the last yielded or returned value with `current()`
- awaiting the wrapper from another coroutine through `await_*`

The current Python binding only exposes the `int` specialization.

## Architecture

The module is intentionally small:

- [`include/async/algorithm.hpp`](include/async/algorithm.hpp) declares the coroutine wrapper and
  its `promise_type`
- [`src/algorithm.cpp`](src/algorithm.cpp) contains the template method definitions
- [`bindings/bindings.cpp`](bindings/bindings.cpp) exports `AsyncAlgorithm<int>` as the Python
  module `async`

Behavior to keep in mind while reading the implementation:

- `initial_suspend()` and `final_suspend()` both return `std::suspend_always`
- `advance()` resumes the coroutine once and returns `false` when the coroutine is already done or
  becomes done after resuming
- `current()` returns the last value written through `yield_value()` or `return_value()`
- `await_suspend()` resumes the wrapped coroutine and then resumes the awaiting coroutine

## Public API

The entire public C++ surface today is:

- `template <typename T> class libstp::async::AsyncAlgorithm`
- `AsyncAlgorithm::advance()`
- `AsyncAlgorithm::current()`
- `AsyncAlgorithm::await_ready()`
- `AsyncAlgorithm::await_suspend()`
- `AsyncAlgorithm::await_resume()`

There are no ready-made coroutine factory functions in this module. Callers are expected to define
their own coroutine-returning functions using `AsyncAlgorithm<T>` as the return type.

## Dependencies

Direct code dependencies are minimal:

- the C++ standard library coroutine support (`<coroutine>`)

At the build-system level, this module is registered with `hal` as an additional dependency through
its `CMakeLists.txt`, even though the current source in this directory does not include HAL headers.

## Python Bindings

`bindings/bindings.cpp` builds a Python extension module named `async`.

Currently exposed Python API:

- `AsyncAlgorithmInt.advance()`
- `AsyncAlgorithmInt.current()`

Not currently exposed:

- other template specializations
- the awaitable interface
- helper functions to construct coroutine instances from Python

If you extend the binding surface, document the ownership and lifetime model clearly. The wrapper
owns the coroutine handle and destroys it in the destructor.

## Testing

No module-local tests exist in this directory. There is also no sample coroutine producer checked in
alongside the wrapper.

If you change this module, verify at least:

- the C++ target still builds with the intended coroutine-capable toolchain
- the Python extension imports successfully
- a small coroutine producer yields and returns values in the order you expect

## Extension Points

Likely future work in this module includes:

- additional bound specializations besides `AsyncAlgorithm<int>`
- helpers that adapt the wrapper to Python `asyncio` or project-specific event loops
- explicit examples or tests for coroutine producers and consumers

Keep the implementation notes in sync if you change suspension behavior, because downstream users
need to reason about exactly when a wrapped coroutine runs.
