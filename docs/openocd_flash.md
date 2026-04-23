# OpenOCD 一键烧录

仓库内置了 OpenOCD，可直接通过 CMake 目标烧录当前构建出的固件。

## 一键命令

Debug:

```bash
cmake --build --preset Debug --target flash
```

Release:

```bash
cmake --build --preset Release --target flash
```

## 目录约定

- `tools/openocd/windows/bin/openocd.exe`
- `tools/openocd/windows/openocd/scripts/...`
- `tools/openocd/linux/bin/openocd`
- `tools/openocd/linux/openocd/scripts/...`

项目在 CMake 配置阶段按主机平台自动选择内置 OpenOCD：

- Windows 主机使用 `tools/openocd/windows`
- Ubuntu 24 等 Linux 主机使用 `tools/openocd/linux`

如果当前平台缺少对应二进制或脚本目录，CMake 会在配置阶段直接报错。

## 可覆盖参数

默认配置适用于 `ST-Link + STM32F405xx`：

```bash
-DOPENOCD_INTERFACE_CFG=interface/stlink.cfg
-DOPENOCD_TARGET_CFG=target/stm32f4x.cfg
-DOPENOCD_ADAPTER_SPEED=4000
-DOPENOCD_FLASH_FILE=<默认是当前 Gimbal_um 目标文件>
```

如果你使用的是 `DAPLink / CMSIS-DAP + STM32F405xx`，可以改成：

```bash
-DOPENOCD_INTERFACE_CFG=interface/daplink.cfg
-DOPENOCD_TARGET_CFG=target/stm32f4x.cfg
```

示例：

```bash
cmake --preset Debug -DOPENOCD_ADAPTER_SPEED=2000
cmake --build --preset Debug --target flash
```

## 前置条件

Windows:

- 已安装 ST-Link USB 驱动，设备管理器可正常识别 ST-Link

Ubuntu 24:

- 当前用户可访问调试器设备
- 建议安装 `udev` 规则，例如将 `tools/openocd/linux/openocd/contrib/60-openocd.rules` 复制到 `/etc/udev/rules.d/`
- 重新加载规则后重新插拔 ST-Link

## 常见故障

`Error: open failed`

- 检查 ST-Link 是否已连接开发板
- 检查目标板是否上电
- 检查 USB 线是否仅供电、不传数据

`Error: libusb_open() failed` 或 Linux 下权限错误

- Ubuntu 24 上优先检查 `udev` 规则和当前用户权限
- 确认没有其他烧录工具占用 ST-Link

`target not halted` 或连接不稳定

- 适当降低 `OPENOCD_ADAPTER_SPEED`，例如 `2000` 或 `1000`
- 检查 SWD 接线与板上复位电路

`No such file or directory`

- 确认仓库内 `tools/openocd/<platform>/bin` 与 `openocd/scripts` 目录完整存在
- 确认已先执行对应 preset 的配置与构建
