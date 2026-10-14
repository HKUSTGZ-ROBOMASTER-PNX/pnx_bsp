# BSP

BSP 在 CubeMX/HAL 之上提供外设接口，供应用直接调用。当前支持 H723 MC02 和 F407 CBoard。

## 职责与构建

- `boards/cubemx/<BOARD>`：生成的外设初始化、IRQ、HAL、ThreadX 和 USBX。
- `boards/wrapper/<BOARD>`：板级配置、外设映射、内存约束，以及 `src/init.cpp` 中的实际 `main()`。
- `bsp`：外设操作、异步状态和回调。依赖 `board` 与 `libs`，board 不依赖 BSP。
- 应用：创建线程、选择配置、组织收发和解码。

[CMakeLists.txt](CMakeLists.txt) 提供 `bsp` INTERFACE 目标，自动收集模块的 `include/*.hpp` 和 `src/*.cpp`。
CAN 按板型只编译一份：H723 使用 `fdcan.cpp`，F407 使用 `bxcan.cpp`。

构建时排除以下生成的应用文件，由 BSP 接管对应入口和 USB CDC 逻辑：

```text
AZURE_RTOS/App/app_azure_rtos.c
Core/Src/app_threadx.c
USBX/App/app_usbx_device.c
USBX/App/ux_device_cdc_acm.c
```

生成的 USB 描述符、PCD 初始化、IRQ、中间件及 ThreadX 底层汇编仍参与构建。
无需在生成文件中添加 `app_start()` 或 USB 回调。CubeMX 已配置为不生成 `main()`；根工程直接引用其
`cmake/stm32cubemx`，不使用生成工程的顶层 CMakeLists。

## 接口概览

| 模块 | 主要功能 | 使用前准备 |
| --- | --- | --- |
| [ADC](adc/include/bsp_adc.hpp) | 轮询、中断、DMA、校准和转换状态 | CubeMX 初始化后调用 `init(config)`；当前板级配置仅 H723 启用 ADC |
| [SPI](spi/include/bsp_spi.hpp) | 阻塞收发、中断或 DMA 单向传输、状态与中止 | CubeMX 初始化，无额外 BSP `init()` |
| [USART](usart/include/bsp_usart.hpp) | 阻塞或 DMA 发送、线路配置、接收到空闲 | `init(config, mode)` |
| [CAN](can/include/bsp_can.hpp) | 经典 CAN、CAN FD、接收回调、错误通知 | 注册回调后调用 `init(config)` 或 `init(fd_config)` |
| [GPIO](gpio/include/bsp_gpio.hpp) | 电平读写、有效态与翻转 | CubeMX 配置引脚；header-only |
| [PWM](pwm/include/bsp_pwm.hpp) | 启停、占空比、周期与脉宽 | CubeMX 配置定时器；header-only |
| [DWT](dwt/include/bsp_dwt.hpp) | 周期计数、微秒时间线、时间差与忙等待 | 时钟配置完成后调用 `init()`；header-only |
| [ThreadX](threadx/src/bsp_threadx.cpp) | 应用初始化入口及 USB 接收线程；src-only | wrapper 调用 `tx_kernel_enter()` |
| [USB](usb/include/bsp_usb.hpp) | CDC ACM 收发、连接状态、接收回调 | ThreadX 初始化时自动初始化 USBX |

多数操作返回 `pnxtypes::status`：`ok`、`error`、`not_configured`、`invalid_arg`、`busy`、
`not_initialized`、`timeout`。HAL 状态转换集中在 [bsp.hpp](bsp/include/bsp.hpp)。
异步启动返回 `ok` 只表示已启动，完成情况通过状态、回调或通知确认。

同一外设的前台操作由应用串行化。CAN、USART 的接收回调在 IRQ 中运行，不得阻塞；
USB 回调在调用 `poll()` 的线程中运行。回调绑定的对象必须活到注销或停止成功之后。

## DMA 与缓冲区

`memory::buffer<N>` 中的 `N` 是字节数，可通过 `data()` 或 `operator[]` 访问。
`logical_size()` 是有效长度，`capacity()` 包含对齐补齐空间。

```cpp
#include "bsp_spi.hpp"

// H723 示例；F407 选择已配置的 SPI 实例。
static memory::buffer<64> rx RAM_D1_BSS;

void start_receive()
{
    const auto result = bsp::spi::receive_dma(bsp::spi::handle::spi2, rx);
    // 检查 result；成功启动后等待 state() 报告完成，再访问 rx。
    (void)result;
}
```

H723 的 DMA 缓冲区放在 D1，使用 `RAM_D1_BSS`；该段不由启动代码清零，发送前必须写入有效数据。
F407 使用普通 SRAM，不能使用 CCM。缓存维护由驱动完成。

| 模块 | 缓冲区与生命周期 |
| --- | --- |
| ADC | 单通道、rank 1；每个采样占 4 字节。异步完成或出错后仍需对应 `stop_it()` / `stop_dma()`，停止成功后才能重启或复用 |
| SPI | IT/DMA 缓冲区保留到成功完成或 `abort()` 成功；错误不会自动释放所有权 |
| USART TX | DMA 发送复制到内部 256 字节暂存区；重叠提交返回 `busy` |
| USART RX | 缓冲区和回调目标保留到 `stop_rx()` 成功；循环 DMA 可覆盖旧数据，需延后处理时在回调内复制 |

USART 的 `start_rx_to_idle()` 支持接收回调和可选的 `TX_SEMAPHORE*` 通知；
`rx_status()` 查询状态，`restart_rx()` 重启接收，`abort_tx()` 中止发送。
9 位无校验串口每个字占两个对齐字节，接口长度仍以字节计。

## CAN

CAN 直接使用生成的 `hcan*` / `hfdcan*`，共用 `bsp_can.hpp` 接口。
经典帧使用 `config`；CAN FD 使用 `fd_config`，可配置 BRS 和 TDC。

```cpp
#include "bsp_can.hpp"

void init_can()
{
    // H723 经典 CAN；F407 改为 handle::can1 或 handle::can2。
    const bsp::can::config classic{bsp::can::handle::fdcan2};
    const auto result = bsp::can::init(classic);
    // 检查 result。
    (void)result;
}
```

- 波特率、时序、FIFO 和 FDCAN 消息 RAM 由 CubeMX 配置，BSP 校验使用条件。
- 经典帧最多 8 字节；FD 最多 64 字节，同时受消息 RAM 元素容量限制。FD 自动补零到合法 DLC 长度。
- `register_rx_callback()` 最多注册 8 个回调；仅在初始化前或 `stop()` 成功后注册、注销。
- 回调中的帧只在回调期间有效，`len` 是线上长度，可能包含 FD 补齐字节。
- `err_sem()` 提供错误信号量；bus-off 后由应用线程调用 `restart()` 恢复。
- F407 请求 FD 配置返回 `not_configured`；选择当前实现不支持的实例返回 `invalid_arg`。

F407 的 bus-off 通知需要 SCE 中断。目前 wrapper 补充了 CAN1/CAN2 的 SCE 使能和 IRQ 入口；
在 CubeMX 启用并生成这些中断后，应移除 wrapper 中对应补丁，避免重复定义。

## GPIO、PWM 与 DWT

GPIO 的 `read/write` 操作物理电平，`is_active/set_active` 根据配置的 `active_level` 转换逻辑有效态。
当前没有独立 EXTI 模块。

PWM 的 `set_duty()` 接受 `[0, 1]`，`set_period_us()` 设置整个定时器的周期，
`set_pulse_width_us()` 设置单通道脉宽。周期由同一定时器的所有通道共享；配置中的时钟频率必须准确，
超出计数器或比较寄存器范围返回 `invalid_arg`。

DWT 初始化后可调用 `cycles32()`、`cycles64()`、`timeline_us()`、`delta_s(last)`。
`delta_s()` 会更新传入的周期计数；首次使用前用 `cycles32()` 初始化 `last`。
64 位累计时间要求至少每个 CYCCNT 回绕周期采样一次，使用期间保持 CPU 时钟频率固定。
`delay_us/ms()` 是忙等待；线程休眠使用 `tx_thread_sleep()`，其参数单位是 tick。

## ThreadX 与 USB

启动顺序：

```text
wrapper main() → 硬件初始化 → tx_kernel_enter()
              → BSP tx_application_define() → USBX 初始化 → 创建 USB 接收线程 → app_start()
              → ThreadX 调度线程
```

应用实现 `extern "C" void app_start()`，在其中创建线程和同步对象。
此时尚未调度，可以注册接收回调，不能阻塞或收发 USB。默认弱实现为空；USB 初始化或接收线程创建失败进入 `Error_Handler()`。

USB 当前为 full-speed CDC ACM，PCD DMA 关闭，USBX 使用 16 KiB 静态内存。
描述符、VID/PID 和端点布局来自 CubeMX。`connected()` 表示 CDC 类已激活，不表示主机程序已经打开串口。
USBX 管理自身的协议线程，`bsp_threadx.cpp` 创建一个 USB 接收线程；发送由业务线程调用 `write()`。

### 接收回调与应用线程

接收线程循环调用 `poll()`，每次最多接收 64 字节，在该线程中执行回调。
栈大小 2048 字节、优先级 10、接收超时 20 ms 集中在 `bsp_threadx.cpp` 中，可按解码负载调整。
未注册回调、断开或接收失败时休眠 1 tick，避免空转。应用只需注册回调，工程中只保留一份 `app_start()`：

```cpp
#include "bsp_usb.hpp"
#include "main.h"

static void receive(const uint8_t* data, uint32_t size)
{
    // 在这里累积和解码字节流；跨线程保存时复制数据。
    (void)data;
    (void)size;
}

extern "C" void app_start()
{
    if (bsp::usb::on_rx(bsp::usb::rx_callback::bind<receive>()) != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }
}
```

回调数据只在调用期间有效；CDC 是字节流，回调边界不是消息边界。复杂处理可复制数据后通知业务线程。
`poll()` 即使返回错误也会分发已经收到的部分数据。`on_rx({})` 注销回调；在 `poll()` 等待或回调执行期间
修改注册会返回 `busy`，成功注销后才可销毁目标对象。

### 直接收发

也可以在线程中使用 `read(data, size, transferred, timeout_ms)` 和
`write(data, size, transferred, timeout_ms)`。默认由 BSP 接收线程负责 `poll()`，应用不要再调用它。
若应用选择直接 `read()`，保持接收回调未注册；切换前必须成功注销回调，避免争抢数据。

- `read()` 的长度必须是 64 字节的正整数倍。读取较大缓冲区可能等待填满或短包，低延迟接收优先用 64 字节。
- 一个读线程和一个写线程可以并行；同方向重叠调用返回 `busy`。
- 缓冲区保留到调用返回；失败时也要检查 `transferred`，避免忽略部分收发。
- 默认超时为 1000 ms，按每次 USB 传输计算，不是整个调用的总截止时间。
- `poll()` / `read()` / `write()` 只能在线程中调用。等待会让出 CPU；断开连接时立即返回，循环需要适当休眠。

## 编译与验证

在仓库根目录执行：

```powershell
cmake --preset h723_mc02
cmake --build --preset h723_mc02 --clean-first

cmake --preset f407_cboard
cmake --build --preset f407_cboard --clean-first
```

两块板已完成清理后的全量编译和链接，并检查 BSP 源码覆盖、CAN 实现选择和生成应用文件排除。
编译通过不代表硬件验证完成；USB 枚举、实际收发、插拔及外设时序需要上板测试。
