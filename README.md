# pnx_bsp

Shared BSP contracts for PnX firmware.

## Branch layout

This repository is split by MCU family. The branches are **not** intended to be
merged into each other.

| Branch            | Target                | Contains                                          |
| ----------------- | --------------------- | ------------------------------------------------- |
| `main`            | STM32H7 boards        | Public headers **and** the H7 `src/` implementations |
| `F4_version_bsp`  | STM32F4 boards (DJI C-board F407) | Public headers only; no shared `src/` implementations |

### Why `F4_version_bsp` has no `src/`

On `main`, several modules ship a shared implementation that is written against
H7 peripherals (FDCAN, H7 DMA/MDMA geometry, H7 clock assumptions). Those files
cannot be reused on an F407 board, and making them multi-MCU would require
either conditional compilation inside shared code or an extra indirection layer.

`F4_version_bsp` takes the other option: the shared layer keeps only the
MCU-neutral contract (`*/include/*.hpp`), and each F4 board implements those
symbols directly in its own `boards/<board>/bsp/` directory. The consumer of
this branch (`pnx_f4_minimal`) links exactly one implementation per image, so a
missing or duplicated implementation is a link-time error rather than a runtime
surprise.

Practical consequences:

- **Do not merge `F4_version_bsp` into `main`.** It would delete the H7
  implementations that H7 boards still depend on.
- **Do not merge `main` into `F4_version_bsp`.** It would reintroduce
  H7-specific sources into the F4 graph.
- Header changes that are genuinely MCU-neutral (contract clarifications, new
  status codes, comment/documentation fixes) may be cherry-picked between
  branches. Anything touching `src/` must not be.

### Contract rule for both branches

Public headers under `*/include/` must stay free of MCU-specific detail: no
vendor HAL types or includes, no CubeMX handle names, no peripheral instances,
no pin numbers, no DMA stream identifiers, no memory geometry. Those facts
belong below the board boundary in the consuming repository.
