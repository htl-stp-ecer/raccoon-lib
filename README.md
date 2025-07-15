# LibStp

[![Gitter][gitter-badge]][gitter-link]

|      CI              | status |
|----------------------|--------|
| conda.recipe         | [![Conda Actions Status][actions-conda-badge]][actions-conda-link] |
| pip builds           | [![Pip Actions Status][actions-pip-badge]][actions-pip-link] |


A robotics library built with [pybind11][] and [scikit-build-core][]. Python
3.9+ is required.

[gitter-badge]:            https://badges.gitter.im/pybind/Lobby.svg
[gitter-link]:             https://gitter.im/pybind/Lobby
[actions-badge]:           https://github.com/pybind/scikit_build_example/workflows/Tests/badge.svg
[actions-conda-link]:      https://github.com/pybind/scikit_build_example/actions?query=workflow%3AConda
[actions-conda-badge]:     https://github.com/pybind/scikit_build_example/workflows/Conda/badge.svg
[actions-pip-link]:        https://github.com/pybind/scikit_build_example/actions?query=workflow%3APip
[actions-pip-badge]:       https://github.com/pybind/scikit_build_example/workflows/Pip/badge.svg
[actions-wheels-link]:     https://github.com/pybind/scikit_build_example/actions?query=workflow%3AWheels
[actions-wheels-badge]:    https://github.com/pybind/scikit_build_example/workflows/Wheels/badge.svg

## Getting Started

### Prerequisites

- C++20 compatible compiler
- CMake >= 3.15
- Python >= 3.9
- Pip

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://gitlab.com/da253/library.git
   cd library
   ```

2. **Install dependencies and build the library:**
   ```bash
   python3 -m venv .venv // Create virtual env to not bloat own installation
   source .venv/bin/activate
   pip install .
   ```

## Usage

```python
import libstp

# result = libstp.some_function(arg1, arg2)
# print(result)
```

## Project Structure

The project is organized into the following directories:

- `modules/`: Contains the C++ source code for the library, organized into submodules.
- `python/`: Contains the Python bindings and additional Python modules.
- `tests/`: Contains tests for the library.
- `docs/`: Contains the documentation.
- `.github/`: Contains CI/CD workflows.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

[cibuildwheel]: https://cibuildwheel.readthedocs.io
[scientific-python development guide]: https://learn.scientific-python.org/development
[dependabot]: https://docs.github.com/en/code-security/dependabot
[github actions]: https://docs.github.com/en/actions
[pre-commit]: https://pre-commit.com
[nox]: https://nox.thea.codes
[pybind11]: https://pybind11.readthedocs.io
[scikit-build-core]: https://scikit-build-core.readthedocs.io
[scikit-build (classic)]: https://scikit-build.readthedocs.io
