#!/usr/bin/env bash

# One-command local build for this CentOS/RHEL 8 development host.  The actual
# build remains delegated to OpenROAD's maintained etc/Build.sh entry point.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
build_script="${script_dir}/etc/Build.sh"
minimum_cmake_version="3.27"

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

version_at_least() {
  printf '%s\n%s\n' "${minimum_cmake_version}" "$1" | sort -V -C
}

cmake_version() {
  "$1" --version 2>/dev/null | awk 'NR == 1 { print $3 }'
}

select_cmake() {
  local candidate version
  local -a candidates=()

  [[ -n "${OPENROAD_CMAKE:-}" ]] && candidates+=("${OPENROAD_CMAKE}")
  [[ -n "${CONDA_PREFIX:-}" ]] && candidates+=("${CONDA_PREFIX}/bin/cmake")
  candidates+=(
    "$(command -v cmake 2>/dev/null || true)"
    "/data_2t/tangzj/installs/anaconda3/envs/torch-cpu/bin/cmake"
    "/usr/local/bin/cmake"
    "/usr/bin/cmake"
  )

  for candidate in "${candidates[@]}"; do
    [[ -n "${candidate}" && -x "${candidate}" ]] || continue
    version="$(cmake_version "${candidate}")"
    if [[ -n "${version}" ]] && version_at_least "${version}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: ./build-oa.sh [etc/Build.sh options]

Build OpenROAD in one command with defaults selected for this host:
  - CMake build (Bazel is not installed)
  - GCC Toolset 13 for the C++20 standard library
  - release OpenROAD with Qt GUI support
  - tests disabled (GoogleTest is not installed)
  - output in build/
  - install into ~/.local/ (already on this host's PATH)

Examples:
  ./build-oa.sh
  ./build-oa.sh -clean
  OPENROAD_BUILD_JOBS=16 ./build-oa.sh
  OPENROAD_BUILD_DIR=/tmp/openroad-build ./build-oa.sh
  OPENROAD_INSTALL_PREFIX=/path/to/install ./build-oa.sh
  OPENROAD_CMAKE=/path/to/cmake ./build-oa.sh
  OPENROAD_CUDD_ROOT=/path/to/cudd ./build-oa.sh
  OPENROAD_LEMON_ROOT=/path/to/lemon ./build-oa.sh
  OPENROAD_ABSL_ROOT=/path/to/abseil ./build-oa.sh
  OPENROAD_YAML_ROOT=/path/to/yaml-cpp ./build-oa.sh
  OPENROAD_QT_CHARTS_ROOT=/path/to/qtcharts ./build-oa.sh
  OPENROAD_DEPS_DIR=/path/to/cache ./build-oa.sh

Additional arguments are passed to etc/Build.sh after these defaults.
EOF
  exit 0
fi

[[ -x "${build_script}" ]] \
  || die "Build script not found or not executable: ${build_script}"
[[ -d "${script_dir}/.git" || -f "${script_dir}/.git" ]] \
  || die "This script must be run from an OpenROAD checkout."

selected_cmake="$(select_cmake)" \
  || die "CMake ${minimum_cmake_version}+ is required. Set OPENROAD_CMAKE to a suitable executable."

build_dir="${OPENROAD_BUILD_DIR:-build}"
jobs="${OPENROAD_BUILD_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}"
[[ "${jobs}" =~ ^[1-9][0-9]*$ ]] \
  || die "OPENROAD_BUILD_JOBS must be a positive integer (got '${jobs}')."
install_prefix="${OPENROAD_INSTALL_PREFIX:-${HOME}/.local}"
for argument in "$@"; do
  case "${argument}" in
    -local)
      install_prefix="${HOME}/.local"
      ;;
    -prefix=*)
      install_prefix="${argument#*=}"
      [[ "${install_prefix}" == /* ]] || install_prefix="${PWD}/${install_prefix}"
      ;;
  esac
done

# etc/Build.sh invokes `cmake` by name.  A one-command shim selects the newer
# CMake without putting the rest of its Conda environment ahead of system tools.
cmake_shim_dir="$(mktemp -d "${TMPDIR:-/tmp}/openroad-cmake.XXXXXX")"
trap 'rm -f "${cmake_shim_dir}/cmake"; rmdir "${cmake_shim_dir}"' EXIT
ln -s "${selected_cmake}" "${cmake_shim_dir}/cmake"
export PATH="${cmake_shim_dir}:${PATH}"

gcc_toolset_enable="${OPENROAD_GCC_TOOLSET_ENABLE:-/opt/rh/gcc-toolset-13/enable}"
if [[ -f "${gcc_toolset_enable}" ]]; then
  # Red Hat's enable script references optional unset environment variables.
  set +u
  # shellcheck source=/dev/null
  source "${gcc_toolset_enable}"
  set -u
else
  die "GCC Toolset 13 was not found. Set OPENROAD_GCC_TOOLSET_ENABLE."
fi

boost_version="1.89.0"
boost_version_underscore="${boost_version//./_}"
deps_dir="${OPENROAD_DEPS_DIR:-${script_dir}/.openroad-local}"
boost_root="${OPENROAD_BOOST_ROOT:-${deps_dir}/boost-${boost_version}}"
boost_header="${boost_root}/include/boost/unordered/unordered_flat_map.hpp"
boost_cmake_dir="${boost_root}/lib/cmake/Boost-${boost_version}"
boost_iostreams_library="${boost_root}/lib/libboost_iostreams.a"
if [[ ! -f "${boost_header}" \
      || ! -f "${boost_cmake_dir}/BoostConfig.cmake" \
      || ! -f "${boost_iostreams_library}" ]]; then
  command -v wget >/dev/null 2>&1 || die "wget is required to download Boost."
  command -v md5sum >/dev/null 2>&1 || die "md5sum is required to verify Boost."

  boost_cache_dir="${deps_dir}/cache"
  boost_source_dir="${deps_dir}/src/boost_${boost_version_underscore}"
  boost_archive="${boost_cache_dir}/boost_${boost_version_underscore}.tar.gz"
  boost_archive_md5="187b577ce9f485314fcf17bcba2fb542"
  mkdir -p "${boost_cache_dir}" "${deps_dir}/src" "${boost_root}"

  if [[ ! -f "${boost_archive}" ]] \
    || ! echo "${boost_archive_md5}  ${boost_archive}" | md5sum --quiet -c -; then
    echo "[INFO] Downloading Boost ${boost_version}..."
    wget -O "${boost_archive}.download" \
      "https://archives.boost.io/release/${boost_version}/source/boost_${boost_version_underscore}.tar.gz"
    echo "${boost_archive_md5}  ${boost_archive}.download" | md5sum --quiet -c - \
      || die "Boost ${boost_version} download checksum verification failed."
    mv "${boost_archive}.download" "${boost_archive}"
  fi
  echo "${boost_archive_md5}  ${boost_archive}" | md5sum --quiet -c - \
    || die "Boost ${boost_version} archive checksum verification failed."

  if [[ ! -x "${boost_source_dir}/bootstrap.sh" ]]; then
    echo "[INFO] Extracting Boost ${boost_version}..."
    tar -xf "${boost_archive}" -C "${deps_dir}/src"
  fi
  if [[ ! -x "${boost_source_dir}/b2" ]]; then
    echo "[INFO] Bootstrapping Boost ${boost_version}..."
    (cd "${boost_source_dir}" && ./bootstrap.sh --prefix="${boost_root}")
  fi
  echo "[INFO] Building Boost ${boost_version}..."
  (
    cd "${boost_source_dir}"
    ./b2 install \
      --prefix="${boost_root}" \
      --with-iostreams \
      --with-serialization \
      --with-system \
      --with-thread \
      -j "${jobs}"
  )
fi
if [[ ! -f "${boost_header}" \
      || ! -f "${boost_cmake_dir}/BoostConfig.cmake" \
      || ! -f "${boost_iostreams_library}" ]]; then
  die "Boost ${boost_version} installation failed."
fi

absl_version="20250512.0"
absl_root="${OPENROAD_ABSL_ROOT:-${deps_dir}/abseil-${absl_version}}"
absl_header="${absl_root}/include/absl/base/no_destructor.h"
absl_config_present=false
for candidate in "${absl_root}/lib/cmake/absl" "${absl_root}/lib64/cmake/absl"; do
  if [[ -f "${candidate}/abslConfig.cmake" ]]; then
    absl_config_present=true
    break
  fi
done
if [[ ! -f "${absl_header}" || "${absl_config_present}" != true ]]; then
  command -v wget >/dev/null 2>&1 || die "wget is required to download Abseil."
  command -v md5sum >/dev/null 2>&1 || die "md5sum is required to verify Abseil."

  absl_archive_md5="ecd64c3c38b20335c48e1ede28a8db90"
  absl_archive="${deps_dir}/cache/abseil-cpp-${absl_version}.tar.gz"
  absl_source_dir="${deps_dir}/src/abseil-cpp-${absl_version}"
  absl_build_dir="${deps_dir}/build/abseil-${absl_version}"
  mkdir -p "${deps_dir}/cache" "${deps_dir}/src" "${deps_dir}/build" "${absl_root}"

  if [[ ! -f "${absl_archive}" ]] \
    || ! echo "${absl_archive_md5}  ${absl_archive}" | md5sum --quiet -c -; then
    echo "[INFO] Downloading Abseil ${absl_version}..."
    wget -O "${absl_archive}.download" \
      "https://github.com/abseil/abseil-cpp/releases/download/${absl_version}/abseil-cpp-${absl_version}.tar.gz"
    echo "${absl_archive_md5}  ${absl_archive}.download" | md5sum --quiet -c - \
      || die "Abseil ${absl_version} download checksum verification failed."
    mv "${absl_archive}.download" "${absl_archive}"
  fi
  echo "${absl_archive_md5}  ${absl_archive}" | md5sum --quiet -c - \
    || die "Abseil ${absl_version} archive checksum verification failed."

  if [[ ! -f "${absl_source_dir}/CMakeLists.txt" ]]; then
    echo "[INFO] Extracting Abseil ${absl_version}..."
    tar -xf "${absl_archive}" -C "${deps_dir}/src"
  fi
  echo "[INFO] Building Abseil ${absl_version}..."
  "${selected_cmake}" \
    -S "${absl_source_dir}" \
    -B "${absl_build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_PREFIX="${absl_root}" \
    -DABSL_BUILD_TESTING=OFF \
    -DABSL_ENABLE_INSTALL=ON
  "${selected_cmake}" --build "${absl_build_dir}" --target install --parallel "${jobs}"
fi
[[ -f "${absl_header}" ]] || die "Abseil ${absl_version} installation failed."

absl_cmake_dir=""
for candidate in "${absl_root}/lib/cmake/absl" "${absl_root}/lib64/cmake/absl"; do
  if [[ -f "${candidate}/abslConfig.cmake" ]]; then
    absl_cmake_dir="${candidate}"
    break
  fi
done
[[ -n "${absl_cmake_dir}" ]] || die "Abseil CMake package was not found under '${absl_root}'."

yaml_root="${OPENROAD_YAML_ROOT:-/home/zjtang/local}"
yaml_cmake_dir="${yaml_root}/lib64/cmake/yaml-cpp"
yaml_library_dir="${yaml_root}/lib64"
if [[ ! -f "${yaml_root}/include/yaml-cpp/yaml.h" \
      || ! -f "${yaml_cmake_dir}/yaml-cpp-config.cmake" \
      || ! -e "${yaml_library_dir}/libyaml-cpp.so" ]]; then
  die "yaml-cpp was not found under '${yaml_root}'. Set OPENROAD_YAML_ROOT."
fi

# Qt Core and Widgets are installed system-wide, but this host lacks Charts.
# Build the matching Qt Charts release locally so GUI support does not require
# root privileges or mix the system Qt/Tcl runtime with an Anaconda runtime.
qmake="${OPENROAD_QMAKE:-$(command -v qmake-qt5 2>/dev/null || true)}"
[[ -x "${qmake}" ]] || die "qmake-qt5 was not found. Set OPENROAD_QMAKE."
qt_version="$("${qmake}" -query QT_VERSION)"
[[ "${qt_version}" == 5.* ]] || die "Qt 5 is required (found '${qt_version}')."
qt_cmake_dir="/usr/lib64/cmake/Qt5"
qt_core_cmake_dir="/usr/lib64/cmake/Qt5Core"
qt_gui_cmake_dir="/usr/lib64/cmake/Qt5Gui"
qt_widgets_cmake_dir="/usr/lib64/cmake/Qt5Widgets"
for candidate in \
  "${qt_cmake_dir}/Qt5Config.cmake" \
  "${qt_core_cmake_dir}/Qt5CoreConfig.cmake" \
  "${qt_gui_cmake_dir}/Qt5GuiConfig.cmake" \
  "${qt_widgets_cmake_dir}/Qt5WidgetsConfig.cmake"; do
  [[ -f "${candidate}" ]] || die "System Qt 5 development files are incomplete: ${candidate}"
done

qt_charts_stage="${OPENROAD_QT_CHARTS_ROOT:-${deps_dir}/qtcharts-${qt_version}}"
qt_charts_root="${qt_charts_stage}/usr"
qt_charts_library_dir="${qt_charts_root}/lib64"
qt_charts_cmake_dir="${qt_charts_library_dir}/cmake/Qt5Charts"
qt_charts_library="${qt_charts_library_dir}/libQt5Charts.so"
qt_charts_header="${qt_charts_root}/include/qt5/QtCharts/QChart"
if [[ ! -f "${qt_charts_cmake_dir}/Qt5ChartsConfig.cmake" \
      || ! -e "${qt_charts_library}" \
      || ! -f "${qt_charts_header}" ]]; then
  command -v wget >/dev/null 2>&1 || die "wget is required to download Qt Charts."
  command -v md5sum >/dev/null 2>&1 || die "md5sum is required to verify Qt Charts."
  command -v make >/dev/null 2>&1 || die "make is required to build Qt Charts."

  qt_charts_archive_md5="0f173cac231bb06cb1ba335b116db106"
  qt_charts_archive="${deps_dir}/cache/qtcharts-everywhere-opensource-src-${qt_version}.tar.xz"
  qt_charts_source_dir="${deps_dir}/src/qtcharts-everywhere-src-${qt_version}"
  qt_charts_build_dir="${deps_dir}/build/qtcharts-${qt_version}"
  mkdir -p "${deps_dir}/cache" "${deps_dir}/src" "${qt_charts_build_dir}" "${qt_charts_stage}"

  if [[ ! -f "${qt_charts_archive}" ]] \
    || ! echo "${qt_charts_archive_md5}  ${qt_charts_archive}" | md5sum --quiet -c -; then
    echo "[INFO] Downloading Qt Charts ${qt_version}..."
    wget -O "${qt_charts_archive}.download" \
      "https://download.qt.io/archive/qt/5.15/${qt_version}/submodules/qtcharts-everywhere-opensource-src-${qt_version}.tar.xz"
    echo "${qt_charts_archive_md5}  ${qt_charts_archive}.download" | md5sum --quiet -c - \
      || die "Qt Charts ${qt_version} download checksum verification failed."
    mv "${qt_charts_archive}.download" "${qt_charts_archive}"
  fi
  echo "${qt_charts_archive_md5}  ${qt_charts_archive}" | md5sum --quiet -c - \
    || die "Qt Charts ${qt_version} archive checksum verification failed."

  if [[ ! -f "${qt_charts_source_dir}/qtcharts.pro" ]]; then
    echo "[INFO] Extracting Qt Charts ${qt_version}..."
    tar -xf "${qt_charts_archive}" -C "${deps_dir}/src"
  fi
  echo "[INFO] Building Qt Charts ${qt_version}..."
  (
    cd "${qt_charts_build_dir}"
    # qmake recursively regenerates submodule Makefiles with bare gcc/g++
    # command names. Keep those on the system toolchain used to build Qt.
    export PATH="/usr/bin:/bin"
    "${qmake}" \
      "${qt_charts_source_dir}/qtcharts.pro" \
      CONFIG+=release \
      QT_BUILD_PARTS=libs \
      QMAKE_CC=/usr/bin/gcc \
      QMAKE_CXX=/usr/bin/g++ \
      QMAKE_LINK=/usr/bin/g++
    make clean
    make -j "${jobs}"
    make install INSTALL_ROOT="${qt_charts_stage}"
  )
fi
if [[ ! -f "${qt_charts_cmake_dir}/Qt5ChartsConfig.cmake" \
      || ! -e "${qt_charts_library}" \
      || ! -f "${qt_charts_header}" ]]; then
  die "Qt Charts ${qt_version} installation failed."
fi

echo "[INFO] Host: $(. /etc/os-release 2>/dev/null; echo "${PRETTY_NAME:-unknown Linux}")"
echo "[INFO] CMake: ${selected_cmake} ($(cmake_version "${selected_cmake}"))"
echo "[INFO] Compiler: $(command -v g++) ($(g++ -dumpfullversion -dumpversion))"
echo "[INFO] Build directory: ${build_dir}"
echo "[INFO] Install prefix: ${install_prefix}"
echo "[INFO] Boost: ${boost_root}"
echo "[INFO] Abseil: ${absl_root}"
echo "[INFO] yaml-cpp: ${yaml_root}"
echo "[INFO] Qt 5: /usr (${qt_version}), Charts: ${qt_charts_root}"

default_cmake_options="-DBUILD_GUI=ON -DBUILD_TESTS=OFF -DLINK_TIME_OPTIMIZATION=OFF"
# GCC Toolset on EL8 supplements the older system libstdc++ at link time.  Use
# static GCC runtimes for final binaries/modules to avoid unresolved C++20 ABI
# symbols when OpenROAD links its many static component libraries.
default_cmake_options+=" -DCMAKE_EXE_LINKER_FLAGS=\"-L${yaml_library_dir} -Wl,-rpath,${yaml_library_dir} -static-libstdc++ -static-libgcc\""
default_cmake_options+=" -DCMAKE_SHARED_LINKER_FLAGS=\"-L${yaml_library_dir} -Wl,-rpath,${yaml_library_dir} -static-libstdc++ -static-libgcc\""
default_cmake_options+=" -DCMAKE_INSTALL_RPATH=${install_prefix}/lib64:${yaml_library_dir}:/opt/or-tools/lib64"
default_cmake_options+=" -DBoost_ROOT=${boost_root}"
default_cmake_options+=" -DBoost_DIR=${boost_cmake_dir}"
default_cmake_options+=" -Dabsl_DIR=${absl_cmake_dir}"
default_cmake_options+=" -Dyaml-cpp_DIR=${yaml_cmake_dir}"
default_cmake_options+=" -DQt5_DIR=${qt_cmake_dir}"
default_cmake_options+=" -DQt5Charts_DIR=${qt_charts_cmake_dir}"
default_cmake_options+=" -DQt5Core_DIR=${qt_core_cmake_dir}"
default_cmake_options+=" -DQt5Gui_DIR=${qt_gui_cmake_dir}"
default_cmake_options+=" -DQt5Widgets_DIR=${qt_widgets_cmake_dir}"
cudd_root="${OPENROAD_CUDD_ROOT:-/data_2t/tangzj/installs/cudd}"
if [[ -f "${cudd_root}/lib/libcudd.a" && -f "${cudd_root}/include/cudd.h" ]]; then
  default_cmake_options+=" -DCUDD_LIB=${cudd_root}/lib/libcudd.a"
  default_cmake_options+=" -DCUDD_INCLUDE=${cudd_root}/include"
  echo "[INFO] CUDD: ${cudd_root}"
else
  die "CUDD was not found under '${cudd_root}'. Set OPENROAD_CUDD_ROOT."
fi

# The /usr/local LEMON headers predate C++20 and use allocator member
# functions removed from the standard. This maintained local fork uses
# std::allocator_traits and supplies the generated lemon/config.h separately.
lemon_root="${OPENROAD_LEMON_ROOT:-/data_2t/tangzj/installs/lemon-graph}"
lemon_library="${lemon_root}/build/lemon/libemon.a"
if [[ -f "${lemon_root}/lemon/bits/array_map.h" \
      && -f "${lemon_root}/build/lemon/config.h" \
      && -f "${lemon_library}" ]]; then
  default_cmake_options+=" -DLEMON_INCLUDE_DIR=${lemon_root};${lemon_root}/build"
  default_cmake_options+=" -DLEMON_LIBRARY=${lemon_library}"
  echo "[INFO] LEMON: ${lemon_root}"
else
  die "C++20-compatible LEMON was not found under '${lemon_root}'. Set OPENROAD_LEMON_ROOT."
fi

"${build_script}" \
  -cmake-build \
  "-dir=${build_dir}" \
  "-threads=${jobs}" \
  "-prefix=${install_prefix}" \
  -no-tests \
  "-cmake=${default_cmake_options}" \
  "$@"

install -d "${install_prefix}/lib64"
qt_charts_real_library="$(readlink -f "${qt_charts_library}")"
qt_charts_real_name="$(basename "${qt_charts_real_library}")"
install -m 0755 "${qt_charts_real_library}" "${install_prefix}/lib64/${qt_charts_real_name}"
ln -sfn "${qt_charts_real_name}" "${install_prefix}/lib64/libQt5Charts.so.5.15"
ln -sfn "${qt_charts_real_name}" "${install_prefix}/lib64/libQt5Charts.so.5"
ln -sfn "${qt_charts_real_name}" "${install_prefix}/lib64/libQt5Charts.so"
echo "[INFO] Installed OpenROAD: ${install_prefix}/bin/openroad"
