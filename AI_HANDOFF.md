# Pipettor_Lab 项目交接文档

本文面向后续接手本工程的 AI 或工程师，用于快速建立项目上下文、定位核心代码、避免误改关键行为。

## 1. 项目一句话

`Pipettor_Lab` 是一套基于 `STM32F103xE + FreeRTOS + STM32 HAL + CMake` 的自动核素分药仪下位机程序。

它负责：

- 本机 12864 LCD 页面和矩阵键盘交互；
- 上位机 CAN 远控、授权、状态上报和流程命令解析；
- 两轴导轨、两台 ISC1000 定量泵、抽水泵、电磁三通阀、RAM-100 活度计的协调控制；
- 配药、发药、排气、冲洗、上电导轨回零、复位收尾等非阻塞状态机流程。

## 2. 当前工程结构

```text
Core/                         STM32CubeMX 生成的 main、外设初始化、中断入口
MyTask/                       FreeRTOS 实际任务函数和全局初始化入口
application/
  machine/                    整机业务状态机
  cmd/                        LCD/键盘/远控命令门面、中文文案、操作说明
modules/
  Communication/              CAN 协议、授权、控制权、ACK/状态/数据帧
  pump_drive/                 ISC1000 定量泵 RS485 协议
  step_motor/                 DM542 两轴导轨脉冲方向驱动
  activity_meter/             RAM-100 活度计 Modbus RTU
  Keyboard/                   矩阵键盘扫描
  LCD/                        ST7920/12864 LCD 驱动
  solenoid_valve/             电磁阀业务封装
  water_pump/                 抽水泵业务封装
  io_output/                  PC13~PC15 24V 输出 GPIO 封装
bsp/
  usart/                      USART DMA + IDLE 接收封装
  can/                        CAN 注册/收发基础封装
  dwt/                        DWT 时间轴
Middlewares/                  FreeRTOS
Drivers/                      STM32 HAL/CMSIS
cmake/                        STM32CubeMX CMake 子工程和工具链文件
```

## 3. 启动和任务调度

主入口：`Core/Src/main.c`

启动顺序：

1. `HAL_Init()`
2. `SystemClock_Config()`
3. `MX_GPIO_Init()`、`MX_DMA_Init()`、`MX_TIM1_Init()`、`MX_CAN_Init()`、`MX_USART2_UART_Init()`、`MX_USART3_UART_Init()`、`MX_TIM2_Init()`、`MX_USB_PCD_Init()`
4. `HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)`，用于蜂鸣器 PWM
5. `AllTaskInit()`
6. `osKernelInitialize()`
7. `MX_FREERTOS_Init()`
8. `osKernelStart()`

`Core/Src/freertos.c` 里创建 5 个任务，但任务函数是 `__weak` 默认实现；真正生效的任务在 `MyTask/MyTask.c` 中同名覆盖。

| 任务 | 周期 | 主要职责 |
| --- | --- | --- |
| `KeyboardTask` | 1 ms | `Keypad_Process()`，矩阵键盘扫描和消抖 |
| `MachineTask` | 2 ms | `MachineControl()`，整机业务流程推进 |
| `LCDTask` | 100 ms | `MachineCMD_LCDTask()`，LCD 页面刷新 |
| `MachineCMDTask` | 2 ms | `MachineCMD_Process()`，按键事件和远控命令分发 |
| `ModuleTask` | 2 ms | `StepMotor_Process()`、`ActivityMeter_Process()`、`PumpDrive_Process()`、`Communication_Process()`、`DWT_SysTimeUpdate()` |

全局初始化入口：`MyTask/MyTask.c::AllTaskInit()`

初始化顺序目前为：

```text
DWT_Init
DisplayLcd_Init
Keypad_Init
Communication_Init
StepMotor_Init
IoOutput_Init
SolenoidValve_Init
WaterPump_Init
ActivityMeter_Init
PumpDrive_BoardInit
MachineCMD_Init
MachineInit
```

## 4. 最核心文件

### `application/machine/machine.c`

整机真实动作都在这里。它是本工程最重要、也最需要谨慎修改的文件。

重点状态机：

- `MachineComboState_e`：配药/完整组合流程、排气、冲洗、复位收尾复用的大状态机；
- `MachineDirectDispenseState_e`：独立发药流程；
- `MachineStartupHomeState_e`：上电导轨物理回零；
- `MachineMotorResetState_e`：按软件位置导轨回 0。

核心公开入口：

- `MachineControl()`
- `Machine_StartRemotePrepareByBottle()`
- `Machine_StartRemoteDispense()`
- `Machine_StartRemoteFlush()`
- `Machine_StartRemoteExhaust()`
- `Machine_EmergencyStop()`
- `Machine_StartMotorReset()`
- `Machine_GetCommunicationStep()`
- `Machine_GetDispenseProgressPercent()`

修改原则：

- 保持非阻塞推进，不要在 `MachineTask` 路径里加长时间阻塞等待。
- 需要延时请使用状态进入时间和 `HAL_GetTick()` 判断。
- 新增流程步骤时同步检查 LCD 阶段、CAN 状态步骤、最终 ACK/结果帧。
- 泵、阀、导轨启动失败必须有错误路径，不要让状态机永久停在等待态。

### `application/cmd/MachineCMD.c`

本机 UI 和上位机命令门面。它不应成为实际业务动作的长期阻塞执行者。

主要职责：

- LCD 页面状态；
- 数字输入缓存；
- 矩阵键盘事件消费；
- 本机流程请求置位；
- 远控命令解析和 ACK；
- 周期状态上报；
- 调试模式。

注意：`MachineCMD` 通常只是置位请求，由 `MachineControl()` 消费后启动真实流程。

### `modules/Communication/Communication.h/.c`

CAN 协议层。

关键 CAN ID：

- `0x090~0x093`：授权握手；
- `0x100`：控制命令；
- `0x101`：参数设置；
- `0x102`：查询；
- `0x103`：控制权响应；
- `0x180`：ACK；
- `0x181`：周期状态；
- `0x182`：报警；
- `0x183`：查询数据/流程结果；
- `0x184`：控制权事件。

协议约定：

- 标准帧，11 bit CAN ID；
- DLC 固定 8 字节；
- 多字节整数小端；
- 动作类命令需要授权；
- 上位机远控和本机控制权切换由通信层维护上下文，但实际停机动作由业务层消费安全动作完成。

## 5. 硬件映射速查

| 功能 | 模块 | 资源/说明 |
| --- | --- | --- |
| 插针导轨，电机 A | `step_motor` | PA0 / TIM2_CH1 输出 PUL，PA7/PA6 DIR |
| 进罐导轨，电机 B | `step_motor` | PA1 / TIM2_CH2 输出 PUL，PA5/PA4 DIR |
| 导轨零点光电 | `step_motor` | 插针 PC4，进罐 PC5 |
| 泵1，300 ul 定量泵 | `pump_drive` | USART3 / RS485，ISC1000 ID=1 |
| 泵2，100 ul 定量泵 | `pump_drive` | USART3 / RS485，ISC1000 ID=2 |
| 泵1光电门 | `pump_drive` | PB15 下降沿计数 |
| 泵2光电门 | `pump_drive` | PB14 下降沿计数 |
| 活度计 | `activity_meter` | USART2，RAM-100 Modbus RTU，9600 8N1 |
| 阀1，水路阀 | `solenoid_valve` / `io_output` | PC13 |
| 阀2，药路阀 | `solenoid_valve` / `io_output` | PC14 |
| 抽水泵 | `water_pump` / `io_output` | PC15 |
| 蜂鸣器 | `main.c`/`tim.c` | TIM1_CH1 PWM |
| CAN | `Communication` + `bsp/can` | 上位机协议 |

## 6. 本机流程概要

### 上电

```text
上电 -> HAL/CubeMX 初始化 -> AllTaskInit -> FreeRTOS
     -> MachineInit -> 上电导轨回零 -> 默认进入远控/待机相关页面
```

### 本机配药

```text
待机
-> 选择 1/2 瓶
-> 输入药瓶体积
-> 准备确认
-> 配药前回吸/排气相关动作
-> 进罐导轨
-> 插针导轨
-> 抽水泵转移原液
-> 活度计等待/读数
-> 泵1分段补水 + 抽水泵转移
-> 导轨回原点
-> 两瓶时等待换罐确认
-> 最终活度稳定
-> 自动排气
-> 自动冲洗
-> 配药完成，等待发药
```

### 本机发药

```text
待机/配药完成
-> 输入发药体积
-> 泵2发药
-> 泵1自动冲洗
-> 返回待机
```

### 远控流程

远控入口在 `MachineCMD_HandleRemoteStartProcess()` 一带，最终调用 `machine.c` 的远控启动函数。

主要流程号：

- `COMMUNICATION_PROCESS_PREPARE = 0x01`
- `COMMUNICATION_PROCESS_DISPENSE = 0x02`
- `COMMUNICATION_PROCESS_FLUSH = 0x03`
- `COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY = 0x04`
- `COMMUNICATION_PROCESS_EXHAUST = 0x05`
- `COMMUNICATION_PROCESS_CONFIRM_BOTTLE_CHANGED = 0x06`

远控 STOP_PROCESS 按急停语义处理：立即停止输出并终止当前整体流程，不执行本机复位收尾。上位机后续发送 RESET_ERROR 清锁存。

## 7. LCD 和中文文案规则

LCD 使用 ST7920 12864 文本模式，单行最大 16 字节。

重要规则：

- 不要在 C 字符串中直接写 UTF-8 中文给 LCD。
- 中文显示文案统一放在 `application/cmd/MachineCMD_Text.c` 的 GB2312 字节表中。
- `MachineCMD_Text.h` 只暴露 `MachineCmdText_s` 符号。
- 新页面需要确认每行不超过 16 字节，否则会错位或乱码。

相关说明文档：

- `application/cmd/LCD按键使用说明.md`
- `application/cmd/矩阵键盘操作说明.md`

## 8. 构建和验证

工程使用 CMake Presets：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

也可手动指定：

```powershell
cmake -S . -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

注意：

- 当前 `build/Debug` 可能包含旧路径缓存。如果出现 `CMakeCache.txt directory is different`，不要在不确认的情况下删除用户构建目录；优先新建临时构建目录验证。
- 本项目需要 ARM GCC 和 Ninja。若工具链环境不完整，先修环境再判断代码问题。
- Git 可能因 Windows 用户所有权提示 `dubious ownership`。可在命令中临时使用：

```powershell
git -c safe.directory=C:/Users/15641/Desktop/Pipettor_Lab status
```

## 9. Git 和远端

当前仓库已经配置 GitHub 远端：

```text
origin https://github.com/ChenBoneng/Pipettor_Lab.git
```

当前主分支：

```text
master
```

`.gitignore` 已忽略：

```text
build/
tmp/
*.map
*.elf
*.hex
*.bin
```

接手修改前建议：

```powershell
git -c safe.directory=C:/Users/15641/Desktop/Pipettor_Lab status --short
git -c safe.directory=C:/Users/15641/Desktop/Pipettor_Lab log --oneline -5
```

不要随意清理 `build/`、`output/` 或用户生成文件，除非任务明确要求。

## 10. 修改前检查清单

开始改代码前先回答这些问题：

1. 改的是本机流程、远控协议、硬件驱动，还是显示文案？
2. 是否会改变 `machine.c` 的流程步骤？
3. 是否需要同步 `Machine_GetCommunicationStep()` 的 CAN 步骤映射？
4. 是否需要新增或调整 `MachineCMD` 页面阶段？
5. 是否影响泵/阀/导轨的急停、暂停、复位收尾？
6. 是否需要新增 ACK、结果帧或状态帧行为？
7. LCD 中文是否已放入 GB2312 文案表？
8. 是否会在任务里引入阻塞等待？
9. 是否需要现场硬件才能验证？
10. 修改后能否至少通过 CMake 编译？

## 11. 常见改动路线

### 改本机流程

优先看：

1. `application/machine/machine.c`
2. `application/cmd/MachineCMD.h`
3. `application/cmd/MachineCMD.c`
4. `application/cmd/LCD按键使用说明.md`

同步关注：

- LCD 阶段；
- 暂停/复位；
- 发药进度；
- 本机库存体积和浓度更新。

### 改远控协议

优先看：

1. `modules/Communication/Communication.h`
2. `modules/Communication/Communication.c`
3. `application/cmd/MachineCMD.c`
4. `application/machine/machine.c`

同步关注：

- ACK 是否立即返回还是流程结束返回；
- `0x181` 状态 Byte1 步骤；
- `0x183` 数据帧；
- 控制权和授权状态。

### 改定量泵行为

优先看：

1. `modules/pump_drive/pump_drive.h`
2. `modules/pump_drive/pump_drive.c`
3. `application/machine/machine.c`
4. `application/cmd/MachineCMD.c` 的调试模式

关键常量：

- `PUMP_DRIVE_PUMP1_FULL_STROKE_UL`
- `PUMP_DRIVE_PUMP2_FULL_STROKE_UL`
- `PUMP_DRIVE_FULL_STROKE_STEPS`
- `PUMP_DRIVE_STEPS_PER_TURN`
- `PUMP_DRIVE_COMMAND_MAX_STEPS`

### 改导轨行为

优先看：

1. `modules/step_motor/step_motor.h`
2. `modules/step_motor/step_motor.c`
3. `application/machine/machine.c`
4. `application/cmd/MachineCMD.c` 的调试模式

注意两轴共用 TIM2 计数周期，工程约定不同时运行两个步进电机。

## 12. 风险点

- `machine.c` 状态多、远控和本机流程复用较多，改一处状态跳转可能影响多条流程。
- `MachineCMD.c` 同时处理 UI 和远控命令，新增命令时要避免误触发本机动作。
- 泵2发药体积分段和 ISC1000 单条命令最大步数有关，不能只看 UI 体积。
- 活度计等待逻辑依赖 `update_count` 和通信状态，不要简单用固定延时替代。
- 电磁阀有 30 ms 响应保护，状态机切阀后应等待流路稳定。
- 上电导轨回零只执行一次，失败路径要保持安全输出关闭。
- `freertos.c` 是 CubeMX 生成风格文件，实际任务实现不要误以为 weak 空循环就是生效逻辑。

## 13. 建议的接手顺序

1. 读本文件。
2. 读 `MyTask/MyTask.c`，理解任务周期。
3. 读 `application/cmd/LCD按键使用说明.md`，理解用户可见行为。
4. 读 `application/machine/machine.h`，掌握业务层对外接口。
5. 按任务类型读 `machine.c` 或 `MachineCMD.c` 中相关状态。
6. 如涉及上位机，读 `Communication.h` 的协议枚举和帧定义。
7. 如涉及硬件，读对应 `modules/*/*.h`，再读 `.c`。
8. 修改后先编译，再根据硬件条件做现场验证。

