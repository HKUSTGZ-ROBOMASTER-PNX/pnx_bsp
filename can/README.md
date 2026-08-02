# CAN 通信质量诊断（BSP 层）

这是 `pnx_bsp/can` 的可选诊断模块，用于**定量**评价每条 CAN 总线的通信质量。它不改变任何收发行为，只在后台以固定周期（默认 1 秒）读取 FDCAN 硬件寄存器里的错误计数器，并把结果写入 C 链接的全局变量，方便你在调试器的 Watch 窗口直接观察，或做长线（分钟/小时级）趋势评价。

## 为什么需要它

CAN 控制器会在链路层自动丢弃坏帧，发送失败还会自动重发。所以应用层只能看到“成功到达的帧”，永远看不到“被打断/重发过多少次”——这就是“应用端显示能连，但总线其实很脏”的原因。本模块把这些硬件层面才有的数字暴露出来：

- **CEL**：总线协议错误总次数（硬件计数，255 封顶）
- **TEC / REC**：本节点发送/接收错误“扣分”（≥96 进 Warning，≥128 进 Passive，TEC>255 进 Bus Off）
- **LEC**：最近一次错误的类型（ACK/位/CRC/填充/格式）

## 如何开启 / 关闭

编辑 `configs/params.json`：

```json
{
  "can_diag": {
    "enabled": true,
    "sample_period_ms": 1000,
    "window_size": 60
  }
}
```

| 字段 | 含义 | 默认 |
| --- | --- | --- |
| `enabled` | 是否启用诊断 | `true` |
| `sample_period_ms` | 采样周期（毫秒），即“多久记一笔” | `1000` |
| `window_size` | 滑动平均窗口长度（采样周期数），同时决定历史数组长度，范围 `[1, 3600]` | `60` |

修改后重新运行 CMake configure / build（生成头文件会自动刷新，`#define CAN_DIAG_ENABLED` 与 `params::can_diag::*` 会随之更新）。

`enabled=false` 时：不创建采样定时器、所有统计钩子被编译剔除，程序行为与未加本模块完全一致（零运行时开销）。

## 调试器怎么读

把以下变量加入 Watch 窗口即可（下标 0/1/2 = FDCAN1/FDCAN2/FDCAN3）：

```
can_diag_bus[0]
can_diag_bus[1]
can_diag_bus[2]
can_diag_sample_count
can_diag_uptime_s

这几个变量在在bsp_can_diag.cpp

extern "C"
{
volatile can_diag_bus_t can_diag_bus[bsp::can::bus_count]{};
volatile uint32_t can_diag_sample_count = 0U;
volatile uint32_t can_diag_uptime_s = 0U;
}
```

`can_diag_bus[i]` 是一个结构体，展开后按下面表格逐项查看。建议先只看四个核心量：`cel_total`、`cel_rate_avg`、`tec_max`、`rec_max`。

## 指标含义

### 累计值（自系统启用以来，32 位不会饱和）

| 字段 | 含义 |
| --- | --- |
| `cel_total` | CEL 增量累计，即“总线总共发生过多少次协议错误（这个上限是255）” |
| `err_events_total` | 错误事件总数 = 状态转移次数（Warning/Passive/Bus Off）+ 采样到协议错误的周期数 |
| `ack_total` | 出现过 ACK 错误的采样周期数（发话没人应答） “采样周期”是一段时间，默认值是1000ms |
| `stuff_form_total` | 出现过位填充/格式错误的采样周期数 |
| `bit_total` | 出现过位错误的采样周期数 |
| `crc_total` | 出现过 CRC 错误的采样周期数 |
| `rx_frames_total` | 收到的帧总数 |
| `tx_attempts_total` | 尝试发送的帧总数（成功写入 TX FIFO） |
| `rx_overrun_total` | RX FIFO 溢出丢帧次数（CPU 处理不过来） |
| `warning_total` / `passive_total` / `busoff_total` | 进入 Error Warning / Error Passive / Bus Off 状态的次数 |
| `cel_overflow_total` | CEL 寄存器 255 饱和期间仍发生错误的次数 |

### 当前值（最近一次采样）

| 字段 | 含义 |
| --- | --- |
| `tec` / `rec` | 发送/接收错误计数（0~255；0 表示当前干净） |
| `cel` | CEL 寄存器当前值（0~255，1 秒窗口内封顶） |
| `lec` | 最近一次协议错误类型，编码见下表 |
| `state_ew` / `state_ep` / `state_bo` | 当前是否处于 Warning / Passive / Bus Off |
| `fifo0_fill` / `fifo1_fill` | 两个 RX FIFO 的水位（元素数） |

### LEC 编码

| 值 | 含义 | 通常指向的问题 |
| --- | --- | --- |
| 0 | 无错误 | 正常 |
| 1 | 位填充错误 | 信号脏/反射/干扰 |
| 2 | 格式错误 | 信号脏/协议异常 |
| 3 | ACK 错误 | 发话没人应答：终端缺失、断线、只有本节点在总线上 |
| 4 | 显性位错误（期望隐性） | 信号脏/电平不对 |
| 5 | 隐性位错误（期望显性） | 信号脏/负载过重 |
| 6 | CRC 错误 | 数据被干扰破坏 |
| 7 | 与上次读取无变化 | 不计数 |

### 窗口增量与滑动平均（最近 `window_size` 个采样周期）

| 字段 | 含义 |
| --- | --- |
| `cel_delta_last` | 上一周期 CEL 增量（本周期发生的协议错误数） |
| `err_events_delta_last` | 上一周期错误事件增量 |
| `rx_frames_delta_last` / `tx_attempts_delta_last` | 上一周期收/发帧数 |
| `cel_rate_avg` | 平均协议错误数/周期（**核心长线指标**） |
| `err_rate_avg` | 平均错误事件数/周期 |
| `rx_rate_avg` / `tx_rate_avg` | 平均收/发帧速率 |
| `tec_avg` / `rec_avg` | TEC/REC 在窗口内的平均值 |
| `window_count` | 已采样周期数（小于 `window_size` 时窗口未满，均值按已有样本计算） |

### 极值与历史环

| 字段 | 含义 |
| --- | --- |
| `tec_max` / `rec_max` | TEC/REC 历史峰值 |
| `cel_rate_max` | 单周期 CEL 增量峰值 |
| `fifo_fill_max` | FIFO 水位峰值（接近容量说明接收压力大） |
| `cel_hist` / `err_hist` / `tec_hist` / `rec_hist` / `rx_hist` / `tx_hist` | 最近 `window_size` 个周期的原始值环形数组，`hist_head` 是下一写入位置，可从 `hist_head` 起按循环顺序看趋势 |

## 判级参考（1Mbps classic，1 秒采样）

| 等级 | cel_rate_avg | TEC/REC | 状态 | 结论 |
| --- | --- | --- | --- | --- |
| 健康 | 0 | 恒为 0 | 始终 Active | 总线干净，无需处理 |
| 临界 | 偶发 >0（每分钟几次） | 短暂 >0 后归零 | 偶尔 Warning | 存在轻微问题，建议查终端/线缆 |
| 差 | 持续 >0 | 持续 >0 不归零 | 频繁 Passive / 出现 Bus Off | 总线有实际问题，必须修 |

配合 `ack_total`、`bit_total` 等字段判断方向：

- `ack_total` 高 → 发送侧问题：检查总线两端 120Ω 终端（万用表测 CANH-CANL 应为 **60Ω**）、线缆断点、节点供电。
- `bit_total` / `crc_total` / `stuff_form_total` 高 → 信号完整性问题：检查总阻值是否被压到 40/30Ω（终端过多）、线长/拓扑、屏蔽接地、收发器供电。
- `rx_overrun_total` 高 → CPU 处理不过来：降低采样中断负载或加大 RX FIFO。
- 总线完全静默但 `busoff_total` 增加 → 节点反复 Bus Off 重启，属于严重故障。

## 例：一次数据解读

以下为某次实机抓取 `can_diag_bus[0]`（FDCAN1，运行 22 秒）的逐字段解读。

### 累计值（自启用以来）

| 字段 | 值 | 解读 |
| --- | --- | --- |
| `cel_total` | 255 | CEL 增量累计。早期每秒错误 ≥255 把寄存器顶满后增量恒为 0，累计停在 255；实际错误数远大于此（见 `err_events_total`） |
| `err_events_total` | 18091 | 错误事件总数 = warning(4523) + passive(4523) + busoff(9045)，22 秒平均约 822 次/秒，总线持续高频犯错 |
| `ack_total` | 0 | 从未采到 ACK 错误：帧在到达应答位之前就失败，轮不到 ACK 错误 |
| `stuff_form_total` | 0 | 无位填充/格式错误 |
| `bit_total` | 22 | 22 个采样周期的 LEC 都是位错误（值 5），错误类型单一且持续 |
| `crc_total` | 0 | 无 CRC 错误：帧没有传完就失败 |
| `rx_frames_total` | 0 | 一帧都没收到：总线上没有可用的发送方，或发出的帧全部损坏 |
| `tx_attempts_total` | 13569 | 累计 13569 次尝试入队（约 617 帧/s）；这只是“写入 TX FIFO 成功”，不是“发送成功” |
| `rx_overrun_total` | 0 | 没有接收，自然没有溢出 |
| `warning_total` / `passive_total` | 4523 / 4523 | 错误状态中断里 EW、EP 同时置位，节点反复冲过 96/128 阈值 |
| `busoff_total` | 9045 | Bus Off 次数是 warning 的两倍，配合现有 Stop/Start 处理形成高频重启循环 |
| `cel_overflow_total` | 0 | 当前收不到：CEL 溢出标志（ELO）对应中断未使能，见文末“观测盲点” |

### 当前值（最近一次采样）

| 字段 | 值 | 解读 |
| --- | --- | --- |
| `tec` / `rec` | 0 / 0 | 采样瞬间刚被 Stop/Start 清零，不代表健康 |
| `cel` | 255 | CEL 寄存器已饱和到最大值 |
| `lec` | 5 | 最近一次错误 = 位 0 错误（期望隐性却读到显性），总线被钳在显性/电平异常 |
| `state_ew` / `state_ep` / `state_bo` | 0 / 0 / 0 | 采样瞬间刚重启，处于 Error Active 的短暂窗口 |
| `fifo0_fill` / `fifo1_fill` | 0 / 0 | RX FIFO 空 |

### 最近一周期增量

| 字段 | 值 | 解读 |
| --- | --- | --- |
| `cel_delta_last` | 0 | CEL 已饱和，增量恒为 0；“0”不代表没错误 |
| `err_events_delta_last` | 800 | 上一秒 800 次错误事件，故障仍在持续 |
| `rx_frames_delta_last` | 0 | 上一秒没有收到帧 |
| `tx_attempts_delta_last` | 600 | 上一秒尝试入队 600 帧（3 个电机 × 200Hz 使能帧） |

### 滑动平均与窗口

| 字段 | 值 | 解读 |
| --- | --- | --- |
| `window_size` / `window_count` | 60 / 22 | 窗口 60 秒，已采样 22 次（运行了 22 秒，窗口未满） |
| `cel_rate_avg` | 11.59 | “假低”：CEL 饱和后增量全 0 把均值拉平，真实速率看 `err_rate_avg` |
| `err_rate_avg` | 800 | 平均每秒 800 次错误事件 |
| `rx_rate_avg` / `tx_rate_avg` | 0 / 600 | 只发不收 |
| `tec_avg` / `rec_avg` | 0 / 0 | 每次采样都赶上重启清零后的瞬间，平均值为 0 |

### 极值与历史环

| 字段 | 值 | 解读 |
| --- | --- | --- |
| `tec_max` / `rec_max` | 0 / 0 | 观测盲点：TEC 在两次采样之间冲过 256 又被清零，1Hz 采样抓不到峰值（见文末） |
| `cel_rate_max` | 255 | 早期某个周期错误数 ≥255，CEL 一次顶满 |
| `fifo_fill_max` | 0 | 从没收到过帧 |
| `cel_hist` 等 6 个数组 | 未展开 | 历史环数组，`hist_head` 之前的 22 个元素有效 |
| `hist_head` | 22 | 已写入 22 个采样点，下一写入位置为下标 22 |

### 结论与排查方向

结论：FDCAN1 处于“只发不收 + 反复 Bus Off 重启”的严重故障循环，不是单纯终端电阻问题。应用层反馈（如 `joint_angle_buffer` 一直为 0）与 `rx_rate_avg=0` 互相印证。本次只展开了 `can_diag_bus[0]`，FDCAN2/3（`can_diag_bus[1]/[2]`）未展开，无法判断。

排查方向：终端电阻不对（单端 120Ω 或过载 40/30Ω）的典型表现是 ACK 错误（LEC=3）且通常还能收到帧；这里 LEC=5 持续且 RX 完全为 0，更符合总线被拉死在显性。优先检查 FDCAN1（PD0/PD1）外部接线与收发器：CANH/CANL 是否短路、RX 是否断开或接错、某节点收发器是否故障钳死总线、收发器供电是否正常；再用示波器量隐性共模（约 2.5V）与显性差分幅度。

### 两个观测盲点

- `tec_max/rec_max` 抓不到峰值：错误计数在两次采样之间冲顶并被 Stop/Start 清零。若需要峰值，应在错误回调（ISR）里同步记录 TEC/REC。
- `cel_overflow_total` 恒为 0：CEL 溢出标志（ELO）对应的中断 `FDCAN_IT_ERROR_LOGGING_OVERFLOW` 尚未使能，该字段目前收不到计数。

## 局限

- `LEC` 只保留“最近一次”错误类型，错误密集时各类型计数偏保守；**总次数以 `cel_total` / `cel_rate_avg` 为准**。
- CEL 寄存器单周期最大 255，`cel_overflow_total` 可提示发生过饱和；极高速率下请同时看 `err_rate_avg`。
- 本模块统计的是**本节点看到**的错误；如需全总线错误帧数，可与 USB-CAN 分析仪（PCAN、周立功等）的 error frame 计数交叉验证。
- 判断物理层是否合格（终端 60Ω、眼图、显性差分电压 ≥1.5V）仍需万用表/示波器，寄存器指标只能说明“有错误”，不能代替波形测量。

## 实现位置

- 采样与全局变量：`pnx_bsp/can/src/bsp_can_diag.cpp`、`pnx_bsp/can/include/bsp_can_diag.hpp`
- 统计钩子：`bsp_can.cpp` 的 `handle_error`、`receive`、`transmit_frame`、`HAL_FDCAN_RxFifo0/1Callback`
- 配置入口：`configs/params.json` 的 `can_diag` 节
