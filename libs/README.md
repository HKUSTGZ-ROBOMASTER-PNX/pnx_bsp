# libs

C++17 头文件库。父工程先添加 `boards`，再添加 `libs`，应用链接 `libs` target。
板型、工具链和 ThreadX 配置由 boards 管理；libs 通过 `stm32cubemx` 获取依赖。

```cmake
add_subdirectory(boards)
add_subdirectory(libs)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE libs)
```

## 文件职能

| 文件 | 主要职能 |
| --- | --- |
| `types/include/status.hpp` | 通用状态码，如 ok、busy、invalid_arg |
| `types/include/callback.hpp` | 不持有对象的函数/成员函数回调，无动态分配 |
| `msg/include/msg.hpp` | 基于 ThreadX mutex 的最新值发布订阅 |
| `crc/include/crc.hpp` | CRC8、CRC16、Modbus CRC16 的计算、校验和追加 |
| `math/include/constants.hpp` | 数学常量、角度转换、平方根倒数 |
| `math/include/constrains.hpp` | 限幅、循环限幅、绝对值限幅 |
| `math/include/geometry.hpp` | 三维向量、四元数运算与欧拉角转换 |
| `filters/include/biquad.hpp` | 二阶滤波状态及 Butterworth 高通 |
| `filters/include/iir.hpp` | 二阶 IIR 低通滤波器 |
| `filters/include/kalman_1d.hpp` | 当前为空，尚未实现 |

## callback

```cpp
#include "callback.hpp"

int increment(int value) { return value + 1; }

struct receiver
{
    int receive(int value) const { return value * 2; }
};

void example()
{
    using callback = pnxtypes::callback<int(int)>;
    auto function = callback::bind<increment>();
    receiver object;
    auto method = callback::bind<&receiver::receive>(&object);

    const int a = function(3); // 4
    const int b = method(3);   // 6
    (void)a;
    (void)b;
}
```

默认构造的回调为空，调用前用 `if (callback)` 或 `valid()` 检查。
成员函数绑定不拥有对象，对象必须比回调的使用期更长。
`==` / `!=` 比较绑定的上下文和调用入口。

## msg

```cpp
#include "msg.hpp"

struct sample { float value; };
msg::topic<sample> samples; // 生命周期覆盖所有订阅者

// 在允许创建 ThreadX mutex 的初始化阶段调用；发布和读取用于线程上下文。
void example()
{
    if (samples.init() != pnxtypes::status::ok)
        return;

    auto sub = samples.subscribe();
    if (sub)
    {
        if (samples.publish(sample{1.0f}) == pnxtypes::status::ok)
        {
            sample output{};
            if (sub.read(output) == pnxtypes::status::ok)
            {
                // 使用 output.value
            }
        }
    }

    // 所有发布者、订阅者停止使用后，显式释放 mutex。
    (void)samples.deinit();
}
```

消息类型必须可平凡复制；每个 topic 最多 8 个订阅槽，当前不支持退订。
连续发布会覆盖旧值，各订阅者只读取最新值，成功读取后清除自己的待读标记。
可用 `available()` 查询是否有新值，但 `read()` 的返回状态才是最终结果。
锁被占用或没有待读消息时返回 `busy`，不阻塞等待；这些接口不用于 ISR。
销毁 topic 前需显式 `deinit()`，重新初始化后应重新订阅。
