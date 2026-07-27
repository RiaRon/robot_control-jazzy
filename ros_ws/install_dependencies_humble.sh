#!/usr/bin/env bash
# Host prerequisites for building this branch's vendor tree on Ubuntu 22.04.
#
# Two of them are not reachable through rosdep: openarm_can links CLI11 without
# declaring it in package.xml, and dg_sdk_ros2_bridge links a libDGSDK.so that
# upstream ships only as versioned files. Both are silent until colcon fails.
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    echo "error: source a ROS 2 Humble environment first" >&2
    exit 2
fi

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
CLI11_VERSION="${CLI11_VERSION:-v2.4.2}"
CLI11_PREFIX="${CLI11_PREFIX:-$HOME/opt/cli11}"
DGSDK_VERSION="${DGSDK_VERSION:-171}"

echo "== rosdep =="
# The three Gazebo simulation packages and realsense2_description are the only
# unresolved keys; none of them is needed to build the drivers.
rosdep install --from-paths "$WORKSPACE/src" --ignore-src -r -y || {
    echo "warning: rosdep left keys unresolved; see docs/humble-verification.md" >&2
}

echo "== CLI11 (openarm_can) =="
if [[ ! -f "$CLI11_PREFIX/lib/cmake/CLI11/CLI11Config.cmake" ]]; then
    SRC="$(mktemp -d)"
    git clone --quiet --depth 1 -b "$CLI11_VERSION" \
        https://github.com/CLIUtils/CLI11.git "$SRC"
    cmake -S "$SRC" -B "$SRC/build" \
        -DCLI11_BUILD_TESTS=OFF -DCLI11_BUILD_EXAMPLES=OFF \
        -DCLI11_BUILD_DOCS=OFF -DCMAKE_INSTALL_PREFIX="$CLI11_PREFIX" >/dev/null
    cmake --build "$SRC/build" --target install >/dev/null
    rm -rf "$SRC"
fi
echo "CLI11 at $CLI11_PREFIX"

echo "== libDGSDK.so (dg_sdk_ros2_bridge) =="
# Upstream vendors libDGSDK_{140,160,171}.so and its CMakeLists links the
# unversioned name. Firmware B>=3.6 and M>=2.8 need 1.6.0 or newer.
LIBS="$WORKSPACE/src/delto_m_ros2/dg_sdk_ros2_bridge/libs"
if [[ ! -f "$LIBS/libDGSDK.so" ]]; then
    cp "$LIBS/libDGSDK_${DGSDK_VERSION}.so" "$LIBS/libDGSDK.so"
fi
echo "libDGSDK.so -> libDGSDK_${DGSDK_VERSION}.so"

cat <<EOF

Build with CLI11 on the search path:

  export CMAKE_PREFIX_PATH=$CLI11_PREFIX:\$CMAKE_PREFIX_PATH
  $WORKSPACE/build.sh
EOF
