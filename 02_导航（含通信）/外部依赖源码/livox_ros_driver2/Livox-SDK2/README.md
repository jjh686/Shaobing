# Bundled Livox SDK2

The headers and `lib/amd64/liblivox_lidar_sdk_shared.so` come from the official
[Livox-SDK2 repository](https://github.com/Livox-SDK/Livox-SDK2), commit
`c0796f04c143143899c87a773d9f6b7136453c0b`. The x86_64 shared library was built
with CMake in Release mode using the `livox_lidar_sdk_shared` target and
`-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-Bsymbolic`. Internal symbol binding prevents
the SDK's bundled spdlog implementation from mixing with ROS 2's system
spdlog implementation during shutdown.

This revision supports MID-360S (device type 35). Its configuration section is
named `Mid360s`; ordinary MID-360 uses `MID360`. The ROS driver treats both
models as four-line sensors.

The ARM64 binary retains the previously bundled version. Rebuild the pinned
official SDK on ARM64 before using MID-360S on that architecture.

See `LICENSE.txt` for the upstream license.
