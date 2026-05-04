# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

```bash
rtk cmake --list-presets              # List presets
rtk cmake --preset Debug               # Configure Ninja Debug build
rtk cmake --build --preset Debug       # Build ELF (arm-none-eabi-gcc)
rtk cmake --build --preset Release     # Build optimized
rtk cmake --build --preset Debug --target flash  # Flash via bundled OpenOCD (DAPLink)
```

Prefix all shell commands with `rtk` (see RTK.md). There are no automated unit tests; a clean Debug build is the minimum gate.

## Architecture

This is a dual-axis gimbal firmware for STM32F405RGT6 + two GM6020 motors (yaw/pitch), with BMI088 IMU, CAN2 motor bus, USB CDC virtual COM for host communication, and FreeRTOS.

### Layer stack (top → bottom)

```
Task / Ctrl     communicate_task, ctrl_task (1kHz), debug_tune debug_task rtt_task
Module          MotorManage (C++), PID (C++), imu_fusion (C/MahonyAHRS)
Device          GM6020 (C++)
BSP             bsp_can, bsp_imu (BMI088 SPI), bsp_usb (CDC protocol)
```

**Language boundary**: only `GM6020` and `MotorManage` use C++ classes. Everything else (BSP, imu_fusion, tasks) is C.

### FreeRTOS model

Two tasks + ISRs:
- `ctrl_task` — 1ms fixed-period control loop. Runs attitude fusion, mode state machine, motor setpoint generation.
- `communicate_task` — woken by USB RX ISR semaphore. Parses protocol frames, pushes to control message queue.

### Control flow

```
USB host command → bsp_usb parses SOF/CRC/EOF frame
  → communicate_task decodes CMD+DATA
  → ctrl_msg_queue → ctrl_task reads queue
  → state machine (mode switch / target update)
  → imu_fusion (MahonyAHRS → world-frame attitude)
  → MotorManage.set(joint yaw, joint pitch)
  → PID position+velocity dual-loop → CAN2 → GM6020 motors
```

Status feedback (`0x03`) and lock feedback (`0x08`) are sent ~10ms via USB CDC.

### Key types (see `User/Task/state.h`)

- `WorkMode_e`: `STABLE(0)`, `SEARCH(1)`, `AUTO_AIM(2)`
- `ProtectState_e`: `PROTECT_NONE(0)`, `PROTECT_LOCK(1)`, `PROTECT_DISABLE(2)`
- `AimTrackMode_e`: `IDLE`, `LARGE_MOVE`, `SMALL_TRACK`, `SPIN_TRACK`, `TRACK_LOST`
- Global control context is the file-scope static `ctrl_ctx` in `ctrl_task.cpp`

### Communication protocol (USB CDC)

Frame: `SOF(0xAA 0x55) + LEN + CMD + DATA + CRC16(MODBUS) + EOF(0x5A 0xA5)`

| CMD | Direction | Purpose |
|-----|-----------|---------|
| `0x82` | Host→MCU | Enable data streaming |
| `0x83` | Host→MCU | Enter search mode |
| `0x84` | Host→MCU | Auto-aim incremental target (yaw/pitch x100, yaw_rate_dps, timestamp) |
| `0x87` | Host→MCU | Enter lock protect |
| `0x88` | Host→MCU | Exit lock protect → STABLE |
| `0x03` | MCU→Host | Attitude feedback (yaw/pitch/roll target, timestamp, mode) |
| `0x08` | MCU→Host | Lock feedback (timestamp, reason) |

`0x84` yaw/pitch values are in 0.01° units; yaw_rate in °/s. Host should send `0x84` continuously during auto-aim — gaps trigger lost-target search fallback.

### Generated vs. hand-written code

- `Core/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/` — STM32CubeMX generated from `Gimbal_um.ioc`. When editing, work inside `/* USER CODE BEGIN */` blocks or regenerate from the `.ioc` file.
- `User/` — all hand-written project code (BSP, Device, Module, Task, Library).
- `cmake/stm32cubemx/` — auto-generated CMake from CubeMX.
- `cmake/gcc-arm-none-eabi.cmake` — ARM GCC toolchain file.

## Coding Conventions

From `ace代码规范.md` (ACE lab embedded C coding standard):
- 4-space indent, braces on their own line, spaces around operators
- Lower snake_case for C files, functions, structs; `_t` suffix for struct typedefs
- Keep existing C++ class names (`MotorManage`, `GM6020`, `PID`) consistent
- No consecutive blank lines; functions should be short and focused
- Doxygen-style `@brief`/`@param`/`@retval` comments on public APIs when behavior is non-obvious

## Important Constraints

- **Only modify what was asked**. Do not expand a local fix into broader refactors or adjacent logic changes.
- The gimbal has software joint limits — do not remove or widen them without explicit approval.
- `ctrl_task` must complete within 1ms. Avoid adding blocking calls or heavy computation in the control path.
- CAN2 is the motor bus; CAN1 is reserved. USB CDC is the only host link.
- IMU offline triggers lock protect output path automatically.
- No host message timeout auto-locks; only explicit `0x87` or IMU loss does.
