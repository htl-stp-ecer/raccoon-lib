# Cross-Compilation für ARM64 (Raspberry Pi 3)

Diese Anleitung erklärt, wie du das raccoon-Wheel direkt auf deinem x86_64-Entwicklungsrechner für den Raspberry Pi 3 (ARM64) kompilierst — ohne Docker, ohne QEMU-Emulation.

Inkrementelle Builds sind dabei typischerweise **5–10× schneller** als der QEMU-Docker-Build.

## Warum Cross-Compilation statt Docker/QEMU?

Der Standard-Build (`build.sh`) startet einen ARM64-Docker-Container und führt darin den ganzen Build aus. Auf x86_64-Hosts läuft dieser Container via QEMU-Emulation — jede kompilierte Instruktion wird emuliert, was einen erheblichen Overhead erzeugt. Mit einer nativen Cross-Toolchain läuft der Compiler nativ auf dem Host und erzeugt direkt ARM64-Binaries.

## Voraussetzungen

### Einmalig: Toolchain einrichten

```bash
bash scripts/setup-cross-toolchain.sh
```

Das Skript installiert (benötigt `sudo`):

| Paket | Zweck |
|---|---|
| `gcc-aarch64-linux-gnu` | C-Cross-Compiler |
| `g++-aarch64-linux-gnu` | C++-Cross-Compiler |
| `binutils-aarch64-linux-gnu` | Linker, strip, etc. für ARM64 |
| `python3.13-dev:arm64` | ARM64 Python-Header (`pyconfig.h`) |
| `ccache` | Compiler-Cache für schnelle Wiederholungsbuilds |

Außerdem registriert es `arm64` als Foreign-Architektur (`dpkg --add-architecture arm64`), damit apt ARM64-Pakete parallel zu amd64 installieren kann.

**Unterstützte Host-Betriebssysteme:** Ubuntu 24.10+ / 25.x (Python 3.13 in den Standard-Repos). Auf Ubuntu 24.04 LTS (noble) ist Python 3.13 nicht im Standard-Repo enthalten — verwende dort den Docker-Build oder trage das deadsnakes-PPA ein.

### Python-Pakete (wenn nicht vorhanden)

```bash
pip install scikit-build-core build pybind11
```

## Build ausführen

### Option A: direkt auf dem Host (empfohlen, nach Setup)

```bash
bash build-cross.sh
```

Das Wheel landet in `build-cross/raccoon-*.whl` und trägt den Platform-Tag `linux_aarch64`.

### Option B: via Docker (kein Host-Setup nötig)

Wer `setup-cross-toolchain.sh` nicht lokal ausführen will oder auf einem anderen Ubuntu-Release arbeitet, kann stattdessen den vorkonfigurierten Cross-Builder-Container nutzen:

```bash
bash build-cross-docker.sh
```

`Dockerfile.cross` enthält den vollständigen Cross-Toolchain-Setup (Compiler, ARM64-Header, LCM). Der Container läuft nativ als x86_64 — **keine QEMU-Emulation**. Der erste Aufruf baut das Docker-Image (~2 Min.); danach wird es wiederverwendet. ccache läuft in einem Docker-Volume, inkrementelle Builds sind damit genauso schnell wie beim Host-Build.

Optionen für `build-cross-docker.sh`:

| Variable | Default | Beschreibung |
|---|---|---|
| `REBUILD_IMAGE=1` | — | Baut das Docker-Image neu |
| `FORCE_REBUILD=1` | — | Löscht den scikit-build-Cache |
| `BUILD_NUMBER` | `0` | CI-Versions-Stempel |
| `CCACHE_VOL` | `raccoon-ccache-cross` | Name des Docker-Volumes für ccache |

Beispiel:

```bash
REBUILD_IMAGE=1 bash build-cross-docker.sh   # Image neu bauen
FORCE_REBUILD=1 bash build-cross-docker.sh   # sauberer Wheel-Build
```

### Optionen

| Variable | Default | Beschreibung |
|---|---|---|
| `BUILD_DIR` | `build-cross` | Ausgabeverzeichnis für das Wheel |
| `BUILD_TYPE` | `Release` | `Release` oder `Debug` |
| `BUILD_JOBS` | Anzahl CPU-Kerne | Parallele Compiler-Jobs |
| `FORCE_REBUILD=1` | — | Löscht den scikit-build-Cache und baut von vorn |
| `SKIP_DSL_GEN=1` | — | Überspringt `generate_step_builders.py` |
| `LIBSTP_RUN_MYPY=ON` | `OFF` | Aktiviert mypy-Type-Check als Build-Gate |

Beispiel:

```bash
FORCE_REBUILD=1 BUILD_JOBS=8 bash build-cross.sh
```

## Deployment

Das erzeugte Wheel kann direkt mit `deploy.sh` auf den Pi übertragen werden. Setze `BUILD_DIR=build-cross`:

```bash
BUILD_DIR=build-cross RPI_HOST=10.101.156.206 bash deploy.sh
```

`deploy.sh` nimmt automatisch das neueste `.whl` aus `BUILD_DIR`.

## Warum C++-Tests nicht im Cross-Build laufen

ARM64-Binaries können nicht direkt auf einem x86_64-Host ausgeführt werden (ohne QEMU). Deshalb ist `LIBSTP_BUILD_TESTS=OFF` im Cross-Build gesetzt. Tests laufen weiterhin in zwei Kontexten:

- **Lokal, nativ (x86_64):** `cmake -B build-test -DLIBSTP_BUILD_TESTS=ON && ninja -C build-test && ctest --test-dir build-test`
- **CI/Docker (ARM64):** über `build.sh` in der GitHub Actions Pipeline

## Wie es funktioniert (technische Details)

### CMake-Toolchain-Datei

`cmake/toolchains/aarch64-linux-gnu.cmake` konfiguriert CMake für Cross-Compilation:

- Setzt `CMAKE_SYSTEM_NAME=Linux` und `CMAKE_SYSTEM_PROCESSOR=aarch64` → CMake erkennt Cross-Compile-Modus
- Gibt `aarch64-linux-gnu-g++` als Compiler an
- Setzt `CMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER` → Programme (Python, ruff, …) werden auf dem Host gesucht, nicht im ARM64-Sysroot
- Pinnt `Python_EXECUTABLE` auf den Host-Python3.13 (für Build-Zeit-Skripte: Syntax-Check, Ruff, DSL-Codegen)
- Pinnt `Python_INCLUDE_DIR` auf `/usr/include/python3.13` (arch-unabhängige Headers); der Cross-Compiler findet `pyconfig.h` automatisch in `/usr/include/aarch64-linux-gnu/python3.13/`

### SOABI-Überschreibung in CMakeLists.txt

pybind11-Extension-Module müssen als `_core.cpython-313-aarch64-linux-gnu.so` benannt sein (nicht `...-x86_64-...`). CMake leitet den SOABI-String aus dem Host-Python-Interpreter ab, was auf x86_64 den falschen Architektur-Teil ergibt.

`CMakeLists.txt` ersetzt nach `python_add_library()` den Architektur-Teil des SOABI-Strings:

```cmake
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  string(REGEX REPLACE "(cpython-[0-9]+-)([^-]+)(-linux-gnu)"
    "\\1aarch64\\3" _aarch64_soabi "${Python_SOABI}")
  set_target_properties(_core PROPERTIES SUFFIX ".${_aarch64_soabi}.so")
endif()
```

### Wheel-Platform-Tag

scikit-build-core liest den Platform-Tag aus `sysconfig.get_platform()`, was auf dem Host `linux-x86_64` zurückgibt. Die Umgebungsvariable `_PYTHON_HOST_PLATFORM=linux-aarch64` überschreibt diese Funktion, sodass das Wheel korrekt als `linux_aarch64` getaggt wird.

## Bekannte Fallstricke (aus der Implementierung)

**`aarch64-linux-gnu-g++ not found`**
→ Toolchain nicht eingerichtet: `bash scripts/setup-cross-toolchain.sh`

**`ARM64 Python 3.13 headers not found`**
→ `libpython3.13-dev:arm64` fehlt. Das Setup-Skript extrahiert die Headers direkt aus dem Debian-Paket (kein vollständiges ARM64-Python-Runtime nötig).

**`GLib2 not found` / `Could NOT find GLib2`**
→ LCM braucht GLib2 auch für ARM64. Das Setup-Skript installiert `libglib2.0-dev:arm64`. Ubuntu ARM64-Pakete kommen von `ports.ubuntu.com` — das Setup-Skript richtet die nötige APT-Quelle ein.

**`add_executable cannot create target 'lcm-gen' because an imported target exists`**
→ `build-cross.sh` patcht LCM's CMakeLists.txt im Cross-Build-Cache (`.cmake-cache-cross/lcm-src/`) einmalig: `lcmgen/` wird bei Cross-Compilation nicht gebaut. Stattdessen wird das System-`/usr/bin/lcm-gen` (x86_64) als IMPORTED CMake-Target registriert. Der Docker-Build (`.cmake-cache-docker/`) ist davon nicht betroffen.

**`python3: command not found`**
→ Ubuntu 25.04 hat keinen `python3`-Symlink standardmäßig. `build-cross.sh` verwendet `python3.13` explizit. Andere Versions-Symlinks müssen ggf. manuell gesetzt werden.

**Das Wheel hat den Platform-Tag `linux_x86_64` statt `linux_aarch64`**
→ `_PYTHON_HOST_PLATFORM` wurde nicht gesetzt. `build-cross.sh` setzt diese Variable automatisch; beim manuellen `python -m build`-Aufruf muss sie explizit gesetzt werden: `_PYTHON_HOST_PLATFORM=linux-aarch64 python3.13 -m build ...`

**`error: externally-managed-environment` beim pip install**
→ Ubuntu 25.04 schützt das System-Python (PEP 668). `build-cross.sh` prüft nur ob die Pakete vorhanden sind; Installation via `uv pip install build scikit-build-core pybind11` oder in einem venv.
