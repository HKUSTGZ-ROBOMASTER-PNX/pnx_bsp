# pnx_bsp

板级硬件抽象（`namespace bsp`），封装 HAL 句柄，不向上层暴露 `huart` / `fdcan` 等。

## 模块

`can` `usart` `spi` `pwm` `adc` `dwt` `iic` `ws2812`

## 依赖

- `pnx_os`、`pnx_platform`
- `pnx_configs`（`bsp_board.hpp`）

## CMake 目标

`pnx_bsp`（静态库）

## Submodule

```bash
git submodule add <url> pnx_bsp
```

## 入口

`bsp::init()` — 见 `include/bsp.hpp`
