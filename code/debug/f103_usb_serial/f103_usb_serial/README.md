# 测试板程序
本仓库是基于 USB 虚拟串口的云台通信联调测试板工程，用于验证上位机与下位机之间的串口协议。
协议、工作模式和系统行为以 `云台手册.md` 为准；本 README 主要说明当前仓库的使用方式，以及当前测试板工程与正式云台系统之间的差异。
当前测试板固件已同步到最新云台手册，主机侧 Python 联调工具仍按旧流程实现，阅读和联调时请先区分“固件现状”和“工具现状”。

## 1. 文档说明

- `云台手册.md`：正式双轴云台系统说明，定义工作模式、通信协议、灯语和故障处理
- `README.md`：本仓库测试板工程说明
- `通信调试指南.md`：通信联调与排障说明

## 2. 协议摘要

本节仅同步云台手册中的关键协议定义。若本节与其他内容存在差异，以 `云台手册.md` 尤其是工作模式章节为准。

### 2.1 适用对象

`云台手册.md` 面向正式双轴云台系统，硬件对象为 `STM32F405RGT6 + BMI088 + GM6020`。
本仓库是 `STM32F103` USB 联调测试板，复用其中一部分通信协议。

### 2.2 时间戳规定

- 协议中的时间戳统一使用 `uint32_t` 毫秒计数
- `0x01` 开机报文、`0x02` 连接成功报文、`0x03` 反馈报文均带时间戳
- `0x84` 自瞄指令和 `0x85` 在线心跳包由上位机携带时间戳

### 2.3 数据帧格式

| 字段 | 字节数 | 数据类型 | 说明 |
|------|--------|----------|------|
| **帧起始** `SOF` | 2 | `0xAA 0x55` | 固定起始标志 |
| **数据长度** `LEN` | 1 | `uint8_t` | 数据区字节长度，不含识别码、CRC、包尾 |
| **识别码** `CMD` | 1 | `uint8_t` | 指令类型或来源标识 |
| **数据区** `DATA` | n | 自定义结构体 | 实际负载数据 |
| **CRC校验** `CRC16` | 2 | `uint16_t` | 从 `SOF` 到 `DATA` 结束的 CRC16 校验，小端序 |
| **包尾** `EOF` | 2 | `0x5A 0xA5` | 固定结束标志 |

数据完整性要求如下：

- 校验范围：从帧起始 `0xAA` 开始到数据区结束
- 校验算法：CRC16 标准算法
- 字节序：CRC16 采用小端序存储
- 错误处理：校验失败的数据包将被丢弃

### 2.4 指令定义

1. 云台发送给视觉，识别码为 `0x01` ~ `0x09`

- `0x01` 云台开机报文

```C
typedef struct
{
    uint32_t time_stamp;	//时间戳
} GimbalBoot_t;
```

- `0x02` 云台连接成功报文

```C
typedef struct
{
    uint32_t time_stamp;	//时间戳
} GimbalHandshakeAck_t;
```

- `0x03` 云台发送反馈报文

```C
typedef struct
{
    int16_t yaw_target;		//yaw角度
    int16_t pitch_target;	//pitch角度
    int16_t roll_target;	//roll角度
    uint32_t time_stamp;	//时间戳
    uint8_t mode;			//当前模式
    uint8_t reserved;		//保留位
} GimbalFeedback_t;
```

- `0x08` 云台锁定报文

```C
typedef struct
{
    uint32_t time_stamp;	//时间戳
    uint8_t msg;			//锁定原因
} GimbalLock_t;
```

```C
typedef enum {
    LOCK_REASON_MANUAL = 1,
    LOCK_REASON_BOOT_TIMEOUT = 2,
    LOCK_REASON_HEARTBEAT_TIMEOUT = 3
} LockReason;
```

2. 视觉发送给云台，识别码为 `0x81` ~ `0x89`

- `0x81` 连接云台，`DATA` 区为空
- `0x82` 开启数据传输，`DATA` 区为空
- `0x83` 进入搜索模式，`DATA` 区为空
- `0x84` 自瞄指令

```C
typedef struct
{
    int16_t yaw_target;
    int16_t pitch_target;
    uint32_t time_stamp;
    uint8_t reserved[2];
} AimControl_t;
```

- `0x85` 在线心跳包

```C
typedef struct
{
    uint8_t mode;
    uint32_t time_stamp;
} Heartbeat_t;
```

- `0x87` 请求云台锁定，`DATA` 区为空
- `0x88` 请求退出锁定模式，进入稳定模式，`DATA` 区为空

### 2.5 工作模式

```C
typedef enum {
    GIMBAL_MODE_STABLE = 0,
    GIMBAL_MODE_SEARCH = 1,
    GIMBAL_MODE_AUTO_AIM = 2,
    GIMBAL_MODE_LOCK_PROTECT = 3,
    GIMBAL_MODE_DISABLE = 4
} GimbalMode;
```

- 稳定模式：开机自检完成后的默认工作状态，也是搜索模式超时后的回退状态，以及锁定保护模式解锁后的返回状态；心跳包中断超过 `1000ms` 时进入锁定保护模式
- 搜索模式：接收到 `0x83` 后进入，也是自瞄模式超时后的回退状态；超过 `500ms` 未收到通讯包时回退到稳定模式，心跳包中断超过 `1000ms` 时进入锁定保护模式
- 自瞄模式：接收到 `0x84` 后进入；上位机发送频率要求不低于 `200Hz`，推荐 `250Hz` 以上；连续 `200ms` 未收到通讯包时回退到搜索模式，心跳包中断超过 `1000ms` 时进入锁定保护模式
- 锁定保护模式：收到 `0x87` 主动锁定、开机后 `15s` 内未成功连接上位机，或心跳包中断超过 `1000ms` 时进入；发送 `0x88` 后退出锁定保护并回到稳定模式
- 失能保护模式：IMU 异常、电机反馈丢失或角度解算超限时进入；无法通过指令切出

## 3. 当前测试板实现现状

当前仓库不是正式云台固件，而是 `STM32F103` USB 联调测试板。当前固件实现位于 `Core/Src/usb.c`、`Core/Src/LED.c` 等文件，已按最新云台手册同步状态机和通信协议；主机侧 `tools/` 下的 Python 工具仍保留旧流程，后续需要再单独同步。

当前测试板工程与云台手册的主要差异如下：

| 项目 | 云台手册 | 当前测试板工程 |
|------|----------|----------------|
| 目标对象 | 正式云台系统 | `STM32F103` USB 测试板 |
| 固件协议与状态机 | 以手册为准 | 当前固件已同步 |
| 主机侧工具 | 应使用 `0x81 -> 0x82 -> 0x85` 的严格流程 | 当前 Python 工具仍按旧握手和旧保活流程实现 |
| 灯语 | 正式云台多色灯语 | 当前只有单个绿色 LED |
| 锁定原因 | `0x08.msg` 使用 `LockReason` | 当前固件已同步，现有 Python 工具尚未适配解析 |
| 失能保护 | 真实 IMU 或电机异常触发 | 当前测试板不主动模拟失能保护 |

当前测试板的实际行为如下：

- 上电后发送一次带时间戳的 `0x01`
- 收到 `0x81` 后返回带时间戳的 `0x02`，进入严格握手流程
- 收到 `0x82` 后才开始周期发送 `0x03`
- `STABLE` 模式下 `yaw/pitch/roll` 回零
- `SEARCH` 模式下 `yaw = 30deg * sin(t)`，`pitch = 15deg * cos(t)`，`roll = 0`
- `AUTO_AIM` 模式下直接回报最近一次 `0x84` 的目标角度
- `SEARCH` 模式连续 `500ms` 未收到通讯包时回退到 `STABLE`
- `AUTO_AIM` 模式连续 `200ms` 未收到通讯包时回退到 `SEARCH`
- 上电 `15s` 未成功连接、收到 `0x87`，或 `0x85` 心跳超时后进入 `LOCK_PROTECT`
- `LOCK_PROTECT` 状态下发送 `0x08 {time_stamp, msg}`，其中 `msg` 取值见 `LockReason`
- `0x88` 只用于从 `LOCK_PROTECT` 返回 `STABLE`

## 4. 当前板级指示

正式云台灯语以 `云台手册.md` 为准。当前测试板只实现了一个 `PC13` 绿色 LED，且为低电平点亮。

| LED 状态 | 当前测试板含义 |
|----------|----------------|
| 每 `1s` 亮 `500ms` | 等待连接 |
| 每 `200ms` 亮 `100ms` | USB 通信活跃 |
| 每 `1s` 亮 `100ms` | `LOCK_PROTECT` |

## 5. 文件架构

- 测试板固件核心逻辑位于 `Core/Src/usb.c` 和 `Core/Src/LED.c`
- `Core/Src/main.c` 负责初始化并轮询调用业务任务
- USB CDC 底层接收桥接位于 `USB_DEVICE/App/usbd_cdc_if.c`
- 主机侧联调工具位于 `tools/usb_host.py`、`tools/usb_session.py`、`tools/usb_protocol.py` 和 `tools/usb_debug_gui.py`

## 6. 构建与联调

### 6.1 固件构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

### 6.2 主机侧依赖

CLI 依赖：

```powershell
python -m pip install pyserial
```

GUI 依赖：

```powershell
python -m pip install -r tools\requirements-gui.txt
```

### 6.3 主机侧运行

CLI：

```powershell
python tools\usb_host.py --port COM5 --workflow semi
```

GUI：

```powershell
python tools\usb_debug_gui.py --port COM5
```

当前 CLI 工具已同步最新固件协议；GUI 本次未单独更新，仍需后续再整理其交互文档。

当前 CLI 工具启动后会自动发送 `0x81` 握手请求，并按 `--workflow` 选择后续行为：

- `--workflow semi`：收到 `0x02` 后等待用户显式发送 `stream`，适合逐步联调
- `--workflow auto`：收到 `0x02` 后自动发送 `0x82` 开启反馈流
- 两种工作流下，CLI 都会自动发送 `0x85` 心跳包，且 `mode` 字段跟随当前目标模式

CLI 可用命令如下：

- `handshake`：重新发送 `0x81`
- `status`：查看当前 workflow、握手状态、开流状态、心跳状态和目标模式
- `workflow semi|auto`：运行时切换工作流；切到 `auto` 且已握手时会立即补发 `0x82`
- `stream`：发送 `0x82` 开启反馈流
- `search`：发送 `0x83`，并持续保持搜索模式
- `auto <yaw_deg> <pitch_deg>`：发送 `0x84`，并持续保持自瞄模式
- `stop`：停止周期 `0x83/0x84` 发送，心跳模式切回 `STABLE`
- `lock`：发送 `0x87`
- `unlock`：发送 `0x88`
- `heartbeat on`：开启 `0x85` 心跳
- `heartbeat off`：关闭 `0x85` 心跳，用于验证心跳超时锁定
- `disable`：兼容旧命令名，等价于 `lock`
- `quit`：退出脚本

建议联调流程如下：

- 半自动：连接后等待 `RX 0x02`，执行 `stream`，再执行 `search` 或 `auto`
- 全自动：使用 `--workflow auto` 启动，等待 `RX 0x02` 和自动 `TX 0x82` 后再执行 `search` 或 `auto`
- 若收到 `RX 0x08` 锁定报文，先执行 `unlock`，再重新选择 `search` 或 `auto`

当前 GUI 工具包含以下能力：

- 串口选择、连接和断开
- 握手与模式切换
- `yaw`、`pitch`、`roll` 三轴数值显示与实时曲线
- 姿态反馈的 3D cube 可视化
- `TX`、`RX`、`system` 日志查看、清空与导出
- `search` 和 `auto` 保活周期配置

## 7. 相关文档

- `云台手册.md`
- `通信调试指南.md`

---

**文档版本**：v0.3.0
**最后更新**：2026-03-31
**适用工程**：`f103_usb_serial` 测试板工程
**更新内容**：同步最新锁定语义，并更新固件与主机工具的现状说明
