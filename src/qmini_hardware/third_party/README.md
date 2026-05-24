# Vendored third-party libraries

## `unitree_actuator_sdk`

Required for `qmini_hardware` to compile and to talk to the GO-M8010-6 motors
over RS-485. **Not committed to this repo.** Pull it in as a git submodule:

```bash
# From the workspace root (qmini_ros2/):
git submodule add https://github.com/unitreerobotics/unitree_actuator_sdk \
  src/qmini_hardware/third_party/unitree_actuator_sdk
git submodule update --init --recursive
```

After cloning fresh on a new machine, run:

```bash
git submodule update --init --recursive
```

License: BSD-3-Clause (compatible with the rest of this workspace).

## Why a submodule and not vendored copy?

- The SDK has its own update cadence; pinning to a specific commit via the
  submodule lets you update on your schedule rather than diffing thousands of
  files in this repo's history.
- Keeps the main repo lean (~MB vs ~hundreds of MB if vendored).
- Trivial to re-clone on the Pi 5 deployment target.

## What the build expects

`src/qmini_hardware/CMakeLists.txt` checks for
`third_party/unitree_actuator_sdk/CMakeLists.txt` and fails with a clear
error message if it's missing. Once present, the SDK will be added as a
subdirectory and linked into the motor-bus node (see M1).
