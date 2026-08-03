# pnx_bsp

Shared BSP contracts and MCU-family direct implementations for PnX firmware.

## Branch layout

This repository is split by MCU family. The branches are **not** intended to be
merged into each other.

| Branch            | Target                | Contains                                          |
| ----------------- | --------------------- | ------------------------------------------------- |
| `main`            | STM32H7 boards        | Public headers and direct H7 `src/` implementations |
| `F4_version_bsp`  | DJI C-board STM32F407 | F407 public contracts and direct F407 `src/` implementations |

### Why each branch has its own `src/`

On `main`, several modules ship a shared implementation that is written against
H7 peripherals (FDCAN, H7 DMA/MDMA geometry, H7 clock assumptions). Those files
cannot be reused on an F407 board, and making them multi-MCU would require
either conditional compilation inside shared code or an extra indirection layer.

`F4_version_bsp` therefore provides an F407 implementation in each applicable
`*/src/` directory. Those sources directly define the public `bsp::*` symbols
and call the F407 HAL/CMSIS surface; there is no `detail::backend_*` forwarding
layer. The consuming repository supplies its generated CubeMX handles, startup,
linker and middleware sources, then links exactly one BSP implementation per
image. A missing or duplicated implementation is a link-time error rather than
a runtime surprise.

Practical consequences:

- **Do not merge `F4_version_bsp` into `main`.** Its F407 sources cannot replace
  the H7 implementations that H7 boards depend on.
- **Do not merge `main` into `F4_version_bsp`.** It would replace the F407
  implementation with H7-specific sources.
- Header changes that are genuinely MCU-neutral (contract clarifications, new
  status codes, comment/documentation fixes) may be cherry-picked between
  branches. A source fix must be ported deliberately to the other MCU family,
  not merged mechanically.

### Contract rule for both branches

Public headers under `*/include/` must stay free of MCU-specific detail: no
vendor HAL types or includes, no CubeMX handle names, no peripheral instances,
no pin numbers, no DMA stream identifiers, no memory geometry. Those facts
belong in the selected branch's `*/src/` implementation or in generated Board
files owned by the consuming repository.
