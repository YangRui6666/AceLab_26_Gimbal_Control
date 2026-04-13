# Repository Guidelines

## Project Structure & Module Organization
`User/` contains project-owned firmware logic: `Bsp/` for peripheral wrappers, `Device/` for hardware-facing classes such as `GM6020`, `Module/` for control algorithms like `PID` and IMU fusion, `Task/` for FreeRTOS tasks, and `Library/` for local supporting code. `Core/`, `USB_DEVICE/`, `Drivers/`, and `Middlewares/` are STM32CubeMX or ST-provided trees; prefer edits inside `/* USER CODE BEGIN */` blocks or regenerate from `Gimbal_um.ioc`. `cmake/` holds the ARM GCC toolchain and CubeMX CMake glue. `build/` is generated output and not source of truth.

## Build, Test, and Development Commands
Prefix shell commands with `rtk` as described in `RTK.md`.

- `rtk cmake --list-presets` lists the supported presets.
- `rtk cmake --preset Debug` configures a Ninja Debug build in `build/Debug`.
- `rtk cmake --build --preset Debug` builds the `Gimbal_um` ELF with `arm-none-eabi-*` tools from `PATH`.
- `rtk cmake --build --preset Release` builds the optimized firmware image.
- `rtk ctest --test-dir build/Debug -N` currently reports `Total Tests: 0`; use it to confirm whether any local tests were added.

## Coding Style & Naming Conventions
Follow `ace代码规范.md`: use 4-space indentation, put braces on their own lines, keep spaces around operators, and avoid multiple blank lines. Prefer lower_snake_case for C files, functions, and structs, with `_t` suffixes for struct typedefs. Keep existing C++ class names, such as `MotorManage` and `GM6020`, consistent with the current codebase. Declare public APIs in headers and add short Doxygen comments when behavior is not obvious from the implementation.

## Testing Guidelines
No automated unit tests are committed in the source tree today. At minimum, require a clean Debug build before merging. For changes in control, CAN, USB, or IMU paths, record bench validation on STM32F405 hardware, including task timing, motor response, and peripheral bring-up observations.

## Commit & Pull Request Guidelines
Recent history uses short Chinese commit subjects that describe one concrete firmware change, sometimes prefixed with `(ai)`. Keep commits focused and descriptive, for example `修改电机管理器，修复构造问题`. Pull requests should summarize affected modules, note whether `Gimbal_um.ioc` or generated files were regenerated, link the related task or issue, and include hardware evidence such as logs, captures, or calibration notes when behavior changes.
