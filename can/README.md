# CAN 诊断

`bsp_can_diag` 是可选的 CAN 链路质量统计，不负责发送、接收或恢复策略。`can_diag.enabled: true` 时，首次成功 `bsp::can::init()` 会创建采样定时器，并统计 CAN 收发、错误和 FIFO 丢失。

配置在所选板型 `configs/boards/<board>/params.json` 的 `can_diag` 分组。调试器可查看 `can_diag_bus[]`、`can_diag_sample_count` 和 `can_diag_uptime_s`；主要关注 `cel_total`、`tec`、`rec`、`rx_frames_total`、`tx_attempts_total` 与 `busoff_total`。

这些值只反映本节点观察到的控制器状态；物理层终端、电平和波形仍需万用表或示波器确认。完整使用方式见 [CAN API](../../docs/api/bsp-can.md) 与 [配置参考](../../docs/configuration-reference.md)。
