#  board 支持

`boards/` 提供 board 相关代码和公共库，由 `CMakeLists.txt` 统一接入。

## 目录

| 目录 | 作用与划分原因 |
| --- | --- |
| `cubemx/` | 按 board 保存 CubeMX 工程、生成代码和厂商依赖，便于独立配置与重新生成。 |
| `dsp/` | 公共 CMSIS-DSP 静态库，`include/` 放头文件，`src/` 放实现，供不同 board 共用。 |
| `memory/` | 按 board 保存链接脚本，集中维护 Flash、RAM 和堆栈布局。 |

每个 CubeMX 工程中：

- `Core/`：硬件初始化、中断和系统代码。
- `Drivers/`：CMSIS 与对应芯片系列的 HAL/LL 驱动。
- `Middlewares/`：ThreadX、USBX 等组件实现。
- `AZURE_RTOS/`、`USBX/`：RTOS 和 USB 的应用接入与平台配置。
- `cmake/`：生成代码的构建清单和工具链配置。
- `.ioc`：CubeMX 配置源文件；启动汇编和 `.ld` 文件提供启动与内存布局。

##  board 选择

```cmake
set(BOARD "h723_mc02" CACHE STRING "Target board")
```

支持 `h723_mc02`（默认）和 `f407_cboard`。外部通过 CMake 参数
`-DBOARD=f407_cboard`，或 Presets 的 `cacheVariables.BOARD` 选择。
选择会保存在构建目录的缓存中，无效名称会报错。

## 构建与维护

- 顶层工程先指定匹配的工具链、设置 C11 或更高标准，并创建与
  `${CMAKE_PROJECT_NAME}` 同名的固件 target，再调用 `add_subdirectory(boards)`。
- 使用公共 DSP 时，将 `dsp` 链接到固件 target；新增函数需加入其源码清单。
- `BOARD` 自动选择 board 目录及 `memory/${BOARD}.ld` 链接脚本；脚本修改后会触发重新链接。
- 工具链仍由顶层指定，需与 board 匹配；切换 board 建议使用独立构建目录。已有缓存不会随默认值改变，需显式传入 `-DBOARD=h723_mc02` 切换。
- 修改引脚、时钟和外设时，优先修改 `.ioc`；手写代码尽量保留在 `USER CODE` 区域。
- 新增 board 时，添加 `cubemx/<board>/` 和 `memory/<board>.ld`，更新 `_supported_boards`，并配置对应工具链。

