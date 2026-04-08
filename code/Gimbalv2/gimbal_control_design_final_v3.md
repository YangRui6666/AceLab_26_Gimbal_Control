# 云台控制软件设计整理稿（最终版 v3）

## 0. 最终确认项

你已确认并固定的核心思路如下：

- 系统按 **FreeRTOS** 组织
- 当前只有两个任务：
  - `ctrl_task`
  - `communicate_task`
- `ctrl_task` 以 **1ms 固定周期**运行
- ISR 使用**信号量唤醒** `communicate_task`
- `GM6020`、`PID`、`MotorManage` 用 **C++ 类**
- `imu_fusion` 保持 **C**
- `ctrl` 维护**对地目标**
- `motor_manage.set(yaw_target, pitch_target)` 接收的是**关节目标角**
- `STABLE / SEARCH / AUTO_AIM` 是并列业务模式
- `LOCK_PROTECT / DISABLE` 是保护态
- `SEARCH` 当前使用 **Lissajous** 占位扫描轨迹
- 所有固定参数统一放在 **config 头文件** 中
- **PID 参数支持运行时在线调参**，默认值由 `config` 提供，运行时落在可修改变量中
- `0x84` 自瞄命令是**增量误差输入**，不是绝对角度输入；手册中的模式、锁定与反馈语义可直接作为对外协议基准

---

## 1. 分层与语言边界

### 1.1 分层

系统分为四层：

1. **BSP 层**
   - CAN
   - IMU
   - 中断与接收缓冲

2. **Device 层**
   - `GM6020`

3. **Module 层**
   - `PID`
   - `MotorManage`
   - `imu_fusion`

4. **Task / Ctrl 层**
   - `communicate_task`
   - `ctrl_task`

### 1.2 语言划分

- `bsp_can`：C
- `bsp_imu`：C
- `imu_fusion`：C
- `GM6020`：C++
- `PID`：C++
- `MotorManage`：C++
- `communicate_task`：C
- `ctrl_task`：C

也就是说：

> **`GM6020`、`PID`、`MotorManage` 用 C++ 类，其余保持 C 写法。**

---

## 2. FreeRTOS 任务模型

### 2.1 总体结构

系统当前固定为 **两个任务 + 多个 ISR**：

#### 任务
- `ctrl_task`
- `communicate_task`

#### ISR
- `USB / UART ISR`
- `CAN ISR`

### 2.2 `ctrl_task`

`ctrl_task` 是系统控制主任务，要求如下：

- 固定周期：**1ms**
- 使用 FreeRTOS 周期调度
- 建议高优先级运行
- 不被通信事件驱动
- 周期内完成控制闭环

建议形式：

```c
void ctrl_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        // 1. 读取消息队列
        // 2. 更新 IMU / 融合
        // 3. 更新电机反馈
        // 4. 健康检查
        // 5. 模式控制
        // 6. 输出控制量

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}
```

**【不可或缺 3】`ctrl_task` 必须被定义为“1ms 固定周期任务”**  
原因：控制时基必须明确，否则搜索轨迹、PID、增量自瞄的时间语义都不稳定。

### 2.3 `communicate_task`

`communicate_task` 是通信处理任务，采用：

> **ISR + 信号量唤醒**

流程为：

```text
ISR 接收数据
-> 写入 buffer
-> 释放信号量
-> 唤醒 communicate_task
-> communicate_task 解析完整帧
-> 投递消息到 ctrl_task
```

建议逻辑：

```c
void communicate_task(void *argument)
{
    for (;;)
    {
        xSemaphoreTake(comm_sem, portMAX_DELAY);

        // 1. 从 rx buffer 中取完整帧
        // 2. 检查 SOF / EOF
        // 3. CRC 校验
        // 4. 解析命令
        // 5. 投递消息到 ctrl_task
        // 6. 发送反馈（如需要）
    }
}
```

### 2.4 ISR 职责边界

ISR 只负责：

- 收字节 / 收报文
- 写 buffer
- 释放信号量
- 立即退出

ISR 不负责：

- 协议解析
- 状态切换
- 模式切换
- 控制计算
- 电机输出

**【不可或缺 4】ISR 必须“快进快出”**  
原因：这是 FreeRTOS 结构下的核心实时边界。

---

## 3. 配置管理（最终敲定）

### 3.1 总体原则

系统所有**固定参数**统一放在 `config` 头文件中管理，不在实现文件内部散写常量。

配置内容包括但不限于：

- PID 默认参数
- PID 输出限幅
- 电机最大电流
- 机械限位
- 任务周期
- 通信超时
- 搜索轨迹参数
- IMU 初始化重试参数

### 3.2 运行时参数与默认参数的关系

这里正式敲定：

- `config` 头文件中的 PID 参数只作为**默认值**
- PID 真正运行时使用的是**RAM 中的可修改变量**
- 初始化时由 `config` 默认值装载到运行时变量中
- 调试时可通过 **J-Link / Ozone** 实时修改这些变量

**【不可或缺 5】PID 参数不能只做成纯宏**  
原因：纯宏是编译期常量，无法在运行时通过 Ozone/J-Link 实时修改。

### 3.3 配置示例

建议统一配置头文件形态如下：

```c
#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

/* Motor Limit */
#define YAW_LIMIT_POS_DEG              (60.0f)
#define YAW_LIMIT_NEG_DEG              (-60.0f)
#define PITCH_LIMIT_POS_DEG            (45.0f)
#define PITCH_LIMIT_NEG_DEG            (-15.0f)

/* Motor Current */
#define YAW_MAX_CURRENT_DEFAULT        (16000.0f)
#define PITCH_MAX_CURRENT_DEFAULT      (16000.0f)

/* PID Default Param - Yaw Position */
#define YAW_POS_KP_DEFAULT             (8.0f)
#define YAW_POS_KI_DEFAULT             (0.0f)
#define YAW_POS_KD_DEFAULT             (0.2f)
#define YAW_POS_OUT_MIN_DEFAULT        (-300.0f)
#define YAW_POS_OUT_MAX_DEFAULT        (300.0f)

/* PID Default Param - Yaw Speed */
#define YAW_SPD_KP_DEFAULT             (15.0f)
#define YAW_SPD_KI_DEFAULT             (0.5f)
#define YAW_SPD_KD_DEFAULT             (0.0f)
#define YAW_SPD_OUT_MIN_DEFAULT        (-16000.0f)
#define YAW_SPD_OUT_MAX_DEFAULT        (16000.0f)

/* PID Default Param - Pitch Position */
#define PITCH_POS_KP_DEFAULT           (8.0f)
#define PITCH_POS_KI_DEFAULT           (0.0f)
#define PITCH_POS_KD_DEFAULT           (0.2f)
#define PITCH_POS_OUT_MIN_DEFAULT      (-300.0f)
#define PITCH_POS_OUT_MAX_DEFAULT      (300.0f)

/* PID Default Param - Pitch Speed */
#define PITCH_SPD_KP_DEFAULT           (15.0f)
#define PITCH_SPD_KI_DEFAULT           (0.5f)
#define PITCH_SPD_KD_DEFAULT           (0.0f)
#define PITCH_SPD_OUT_MIN_DEFAULT      (-16000.0f)
#define PITCH_SPD_OUT_MAX_DEFAULT      (16000.0f)

/* Task / Timeout */
#define CTRL_TASK_PERIOD_MS            (1U)
#define COMM_TIMEOUT_MS                (500U)
#define LOCK_TIMEOUT_MS                (60000U)
#define MOTOR_OFFLINE_MS               (20U)

/* Search Trajectory */
#define SEARCH_YAW_CENTER_DEG          (0.0f)
#define SEARCH_PITCH_CENTER_DEG        (10.0f)
#define SEARCH_YAW_AMP_DEG             (25.0f)
#define SEARCH_PITCH_AMP_DEG           (8.0f)
#define SEARCH_W_RAD                   (2.5132741f)

/* IMU */
#define IMU_INIT_RETRY_CNT             (3U)
#define IMU_INIT_RETRY_DELAY_MS        (500U)

#endif
```

### 3.4 配置命名要求

建议统一使用明确前缀：

- `YAW_POS_KP_DEFAULT`
- `PITCH_SPD_KD_DEFAULT`
- `CTRL_TASK_PERIOD_MS`
- `SEARCH_YAW_AMP_DEG`

避免使用不清晰的名字，例如：

- `KP1`
- `KP2`
- `LIMIT1`
- `TIMEOUT1`

---

## 4. BSP 层

### 4.1 `bsp_can`

职责：

- CAN 初始化
- 报文发送
- 报文接收
- 中断写入 ring buffer
- 记录最近接收时间
- 在线检查

接口：

```c
bool bsp_can_init(void);
bool bsp_tx(uint16_t can_id, const uint8_t *data, uint8_t dlc);
bool bsp_rx(uint16_t can_id, CanRxFrame *frame);
void can_ISR(void);
bool can_check(uint16_t can_id);
```

约定：

- `bsp_rx()` 从对应 `can_id` 的 `ring_buffer` 中取**最新一帧**
- 若无新数据则返回失败
- `can_ISR()` 只收包、写 buffer、记时间
- `can_check()` 用于在线检查

### 4.2 `bsp_imu`

职责：

- IMU 初始化
- 获取原始六轴数据
- 基础健康检查

建议接口：

```c
bool bsp_imu_init(void);
bool bsp_imu_update(IMURawData *raw);
bool imu_check(void);
```

约定：

- `bsp_imu_update()` 输出的是**原始六轴数据**
- 不在这里直接输出姿态角
- 初始化最多重试 3 次，每次失败可延时 500ms

---

## 5. Device 层

### 5.1 `GM6020`（C++ 类）

`GM6020` 表达一个电机实例对象。

建议职责：

- 保存配置参数
- 保存当前状态
- 解析反馈
- 做在线检查

建议形式：

```cpp
class GM6020 {
public:
    struct State {
        float angle_deg;
        float speed_dps;
        int16_t current;
        uint16_t encoder_raw;
    };

public:
    GM6020(uint16_t can_id, int16_t max_current, float limit_pos, float limit_neg);

    bool init();
    bool update();
    State get_state() const;
    bool check(uint32_t now_ms) const;

private:
    uint16_t can_id_;
    int16_t max_current_;
    float limit_pos_;
    float limit_neg_;

    uint32_t last_rx_time_;
    State state_;
};
```

**【不可或缺 6】构造函数只保存配置，不做硬件初始化；硬件相关初始化必须放 `init()`**  
原因：避免全局/静态对象构造阶段碰硬件，导致启动顺序不可控。

---

## 6. Module 层

### 6.1 `PID`（C++ 类）

`PID` 正式改为 C++ 类，和 `GM6020`、`MotorManage` 保持一致。

职责：

- 保存 PID 参数
- 保存控制器内部状态
- 提供统一计算接口
- 支持复位、限幅等控制逻辑

形式：

```cpp
class PID {
public:
    struct Param {
        float kp;
        float ki;
        float kd;
        float out_min;
        float out_max;
    };

public:
    void init(const Param& p);
    void reset();
    void set_param(float kp, float ki, float kd);
    void set_limit(float out_min, float out_max);

    float calc(float ref, float fdb);

    Param& param();
    const Param& param() const;

private:
    Param param_;
    float err_;
    float last_err_;
    float integral_;
};
```

### 6.2 `PID` 的运行时调参方式

敲定如下：

- 每个 PID 使用一份独立的运行时参数变量
- 该变量在系统启动时由 `config` 默认值初始化
- 调试时可通过 Ozone/J-Link 直接修改

建议参数块形式：

```cpp
typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
} PIDParam_t;

extern PIDParam_t g_yaw_pos_pid_param;
extern PIDParam_t g_yaw_spd_pid_param;
extern PIDParam_t g_pitch_pos_pid_param;
extern PIDParam_t g_pitch_spd_pid_param;
```

初始化示例：

```cpp
g_yaw_pos_pid_param.kp = YAW_POS_KP_DEFAULT;
g_yaw_pos_pid_param.ki = YAW_POS_KI_DEFAULT;
g_yaw_pos_pid_param.kd = YAW_POS_KD_DEFAULT;
g_yaw_pos_pid_param.out_min = YAW_POS_OUT_MIN_DEFAULT;
g_yaw_pos_pid_param.out_max = YAW_POS_OUT_MAX_DEFAULT;
```

这样可以满足：

- 默认参数统一来源于 `config`
- 调试时可实时修改 `kp / ki / kd`

### 6.3 `MotorManage`（C++ 类）

职责：

- 持有两个 `GM6020` 实例
- 持有 yaw / pitch 的 PID
- 更新反馈
- 统一下发关节目标
- 实现保护输出

建议形式：

```cpp
class MotorManage {
public:
    bool init();
    void update_feedback();

    void set(float yaw_target, float pitch_target);
    void lock();

private:
    GM6020 yaw_;
    GM6020 pitch_;

    PID yaw_pid_speed_;
    PID yaw_pid_location_;
    PID pitch_pid_speed_;
    PID pitch_pid_location_;
};
```

### 6.4 `set(yaw_target, pitch_target)` 的最终语义

这里最终敲定为：

- `yaw_target`：**yaw 关节目标角**
- `pitch_target`：**pitch 关节目标角**

参考系为：

> **相对各自关节零位 / 标定零位的目标角**

不是：

- 对地目标角
- 视觉误差角
- 相机绝对朝向角

**【不可或缺 7】这个语义必须固定，不再模糊写成“相对机体/本体角”**  
原因：这是系统分层的边界定义。

### 6.5 `lock()` 的必要性

**【不可或缺 8】`MotorManage` 必须提供 `lock()` 一类的显式接口**  
原因：进入 `LOCK_PROTECT` 时，不能只是不再调用 `set()`；必须有明确动作让输出停下来或进入锁定保护状态。

### 6.6 `imu_fusion`（C）

`imu_fusion` 继续保留 C 写法。

职责：

- 输入 IMU 原始六轴数据
- 完成姿态融合 / 解算
- 输出 `yaw / pitch / roll`

建议接口：

```c
bool imu_fusion_update(const IMURawData *raw);
AttitudeAngle imu_fusion_get_angle(void);
```

---

## 7. C / C++ 边界

由于 `ctrl_task.c` 与 `communicate_task.c` 是 C 文件，而 `MotorManage`、`PID`、`GM6020` 是 C++ 类，因此需要一层 C API 包装。

建议形式：

```c
#ifdef __cplusplus
extern "C" 
{
#endif

bool motor_manage_init(void);
void motor_manage_update_feedback(void);
void motor_manage_set(float yaw_target, float pitch_target);
void motor_manage_lock(void);

#ifdef __cplusplus
}
#endif
```

**【不可或缺 9】这层 C 包装必须存在**  
原因：C 文件不能直接操作 C++ 类。

---

## 8. 状态设计

### 8.1 内部状态

内部状态拆成两部分：

#### 业务模式
```c
typedef enum {
    WORK_MODE_STABLE = 0,
    WORK_MODE_SEARCH,
    WORK_MODE_AUTO_AIM
} WorkMode_e;
```

#### 保护态
```c
typedef enum {
    PROTECT_NONE = 0,
    PROTECT_LOCK,
    PROTECT_DISABLE
} ProtectState_e;
```

### 8.2 业务模式语义

#### `STABLE`
- 默认业务模式
- 对地稳定

#### `SEARCH`
- 按预设轨迹自主搜索

#### `AUTO_AIM`
- 根据视觉误差进行增量自瞄

### 8.3 保护态语义

#### `PROTECT_LOCK`
- 锁定保护
- 停止正常扭矩输出

#### `PROTECT_DISABLE`
- 失能保护
- 电机断电失能 / 程序终止

### 8.4 对外协议映射

虽然内部拆成 `work_mode + protect_state`，但对外反馈仍可映射成手册中的单 `mode` 字段：  
`STABLE / SEARCH / AUTO_AIM / LOCK_PROTECT / DISABLE`

---

## 9. 通信任务与消息队列

### 9.1 通信协议语义

根据手册，控制与反馈可按以下方式理解：

- `0x83`：进入搜索模式
- `0x84`：自瞄增量误差输入
- `0x87`：请求进入锁定保护
- `0x88`：请求退出锁定保护
- `0x03`：云台状态反馈
- `0x08`：锁定反馈

### 9.2 内部消息结构

建议内部消息队列内容定义为：

```c
typedef enum {
    CTRL_MSG_NONE = 0,
    CTRL_MSG_ENTER_SEARCH,
    CTRL_MSG_AUTO_AIM_DELTA,
    CTRL_MSG_ENTER_LOCK,
    CTRL_MSG_EXIT_LOCK
} CtrlMsgType_e;

typedef struct {
    CtrlMsgType_e type;
    uint32_t time_stamp;
    float delta_yaw;
    float delta_pitch;
} CtrlMsg_t;
```

消息流为：

```text
ISR -> rx buffer -> communicate_task -> ctrl message queue -> ctrl_task
```

---

## 10. Ctrl 层职责

这里最终敲定：

> **`ctrl` 维护对地目标；`motor_manage` 只执行关节目标。**

`ctrl` 至少维护：

```c
float yaw_world_target;
float pitch_world_target;

float yaw_joint_target;
float pitch_joint_target;

WorkMode_e work_mode;
ProtectState_e protect_state;
```

### 10.1 `STABLE`

- 保持对地稳定
- 根据当前姿态计算关节目标
- 再调用 `motor_manage_set(...)`

### 10.2 `SEARCH`

- 按占位搜索轨迹更新对地目标
- 轨迹形式为 **Lissajous 扫描**

可暂写为：

```c
yaw_world_target   = yaw_center   + A_yaw   * sin(w * t);
pitch_world_target = pitch_center + A_pitch * sin(2 * w * t + phi);
```

说明：

- 当前只是占位扫描逻辑
- 后续可按实机效果调整
- 不在本稿中钉死具体参数

### 10.3 `AUTO_AIM`

- 从消息队列读取 `delta_yaw / delta_pitch`
- 以增量方式修正对地目标

```c
yaw_world_target   += delta_yaw;
pitch_world_target += delta_pitch;
```

### 10.4 对地目标到关节目标

由 `ctrl` 完成：

```c
ctrl_world_to_joint(attitude,
                    yaw_world_target,
                    pitch_world_target,
                    &yaw_joint_target,
                    &pitch_joint_target);
```

然后调用：

```c
motor_manage_set(yaw_joint_target, pitch_joint_target);
```

---

## 11. PID 在系统中的使用方式

PID 不直接暴露给 `ctrl_task` 使用，而是作为 `MotorManage` 的内部执行器，用于双环控制。

控制链路如下：

```text
ctrl_task
  -> motor_manage_set(yaw_joint_target, pitch_joint_target)
  -> 位置环 PID：角度误差 -> 目标速度
  -> 速度环 PID：速度误差 -> 目标电流
  -> GM6020 下发电流
```

典型逻辑：

```cpp
float yaw_speed_ref = yaw_pid_location_.calc(yaw_target, yaw_angle);
float yaw_current   = yaw_pid_speed_.calc(yaw_speed_ref, yaw_speed);

float pitch_speed_ref = pitch_pid_location_.calc(pitch_target, pitch_angle);
float pitch_current   = pitch_pid_speed_.calc(pitch_speed_ref, pitch_speed);
```

---

## 12. 保护逻辑

### `LOCK_PROTECT`
进入条件：

- 接收到 `0x87`
- 或长时间通信超时

行为：

- `protect_state = PROTECT_LOCK`
- 调用 `motor_manage_lock()`

### `DISABLE`
进入条件：

- IMU 异常
- 电机反馈丢失
- 电机角度解算越界
- 其他严重错误

行为：

- `protect_state = PROTECT_DISABLE`
- 电机失能
- 程序终止或不可恢复错误处理

---

## 13. `ctrl_task` 主流程

最终流程如下：

```c
void ctrl_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        // 1. 取消息队列
        while (msg_queue_pop(&msg))
        {
            ctrl_handle_msg(&msg);
        }

        // 2. 更新 IMU 原始数据
        bsp_imu_update(&imu_raw);

        // 3. 姿态融合
        imu_fusion_update(&imu_raw);
        attitude = imu_fusion_get_angle();

        // 4. 更新电机反馈
        motor_manage_update_feedback();

        // 5. 健康检查
        ctrl_check_health();

        // 6. 处理保护态
        if (protect_state == PROTECT_LOCK)
        {
            motor_manage_lock();
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
            continue;
        }

        if (protect_state == PROTECT_DISABLE)
        {
            ctrl_fatal_error();
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
            continue;
        }

        // 7. 按业务模式更新对地目标
        switch (work_mode)
        {
        case WORK_MODE_STABLE:
            ctrl_update_stable_target();
            break;

        case WORK_MODE_SEARCH:
            ctrl_update_search_target();
            break;

        case WORK_MODE_AUTO_AIM:
            ctrl_update_auto_aim_target();
            break;
        }

        // 8. 对地目标 -> 关节目标
        ctrl_world_to_joint(attitude,
                            yaw_world_target,
                            pitch_world_target,
                            &yaw_joint_target,
                            &pitch_joint_target);

        // 9. 执行关节目标
        motor_manage_set(yaw_joint_target, pitch_joint_target);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}
```

---

## 14. `communicate_task` 主流程

```c
void communicate_task(void *argument)
{
    for (;;)
    {
        xSemaphoreTake(comm_sem, portMAX_DELAY);

        while (rx_buffer_has_frame())
        {
            if (!protocol_parse_one_frame(&frame))
                continue;

            ctrl_msg = protocol_to_ctrl_msg(&frame);
            msg_queue_push(&ctrl_msg);
        }

        communicate_tx_process();
    }
}
```

---

## 15. 一句话最终总结

> **系统基于 FreeRTOS，采用“两个任务 + ISR 驱动”的结构：`ctrl_task` 以 1ms 固定周期运行并维护对地目标，`communicate_task` 由 ISR 通过信号量唤醒完成协议解析；`GM6020`、`PID` 与 `MotorManage` 用 C++ 类实现，其他层保持 C 写法；所有固定参数统一在 config 中管理，其中 PID 默认值由 config 提供，运行时参数落在可修改变量中，便于通过 Ozone/J-Link 在线调参；内部状态拆成“业务模式 + 保护态”，对外协议仍兼容单一 mode 字段。**
