# MuJoCo机器人仿真实战

这是一部面向人形机器人、机械臂和机器人仿真工程师的系统教材。它不是 API 摘抄：每个主题从物理/数学原理出发，落到 `mjModel`、`mjData` 和 C API，再通过可独立运行的最小 C++ 实验形成证据。

书稿当前包含 38 章规划、7 篇知识递进、40 余个独立实验和两个综合项目。版本基线是 **MuJoCo 3.11.0**。

> 构建只使用仓库内 `mujoco-3.11.0/` 预编译 Linux x86-64 SDK，不编译、不链接上级 MuJoCo 源码仓库。其他 OS/architecture 需换入对应平台的 MuJoCo 3.11.0 官方 SDK。

## 学习地图

```mermaid
flowchart LR
  A[可信仿真<br/>1–7] --> B[机器人建模<br/>8–13]
  B --> C[动力学与接触<br/>14–20]
  C --> D[控制优化估计<br/>21–27]
  D --> E[程序化/批量/学习<br/>28–30]
  E --> F[可视化与生产<br/>31–35]
  F --> G[综合项目<br/>36–38]
```

| 篇 | 能力产出 |
|---|---|
| 第一篇 | 能加载、建模、步进，并证明 timestep/energy/复现可信 |
| 第二篇 | 能构建 actuator/sensor/tendon/闭链/柔性机器人模型 |
| 第三篇 | 能解释 M、bias、inverse dynamics、soft contact、solver 与 wrench |
| 第四篇 | 能实现多速率控制、IK、sysID、computed torque、LQR、EKF |
| 第五篇 | 能用 mjSpec、batch rollout、MJX/APG 组织生成与学习 |
| 第六篇 | 能做 headless RGB/depth、UI threading、plugin/VFS 和部署升级 |
| 第七篇 | 能完成 7-DoF arm 与 floating-base biped 的端到端验收 |

完整目录见[全书目录与知识递进](BOOK_OUTLINE.md)，主题验收见[官方文档覆盖矩阵](chapters/00_coverage_matrix.md)，章节质量要求见[书稿规范](BOOK_SPEC.md)。

## 独立实验

每个实验位于自己的目录，没有 `common.h` 或隐藏框架，不依赖其他示例。通常只有：

```text
examples/NN_topic/
├── main.cc
├── model.xml
└── CMakeLists.txt
```

纯程序化 `mjSpec`/VFS 实验不放置从不读取的 XML。每个目录单独 configure/build，不提供根目录 aggregate CMake，也不调用 `enable_testing()`：

```bash
cd examples/32_damped_ik
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

实验索引、知识点和预期现象见[examples/README.md](examples/README.md)。

## 环境

- CMake 3.16 或更高；
- 支持 C++17 的编译器；
- Linux x86-64，与仓库 SDK 匹配；
- 离屏渲染实验额外需要系统 `libEGL.so.1` 和可用 EGL/OpenGL driver。

普通 physics/controller 示例不需要 display server、Python 或 MuJoCo 源码。CMake 写入 SDK build RPATH，无需用户修改全局 `LD_LIBRARY_PATH`。

## 教材约定

- MJCF angle 默认 degree（可由 `compiler/angle` 改变）；C API 内部角度使用 radian。
- `nq` 不总等于 `nv`：ball/free joint 的 quaternion configuration 在 tangent space 少一维。
- 所有扁平数组通过 name→id→address/dimension 查询，不硬编码机器人规模。
- 示例刻意使用 C 风格资源生命周期，让 `load/make/delete` 清晰；生产应用可在其外增加 RAII。
- “跑起来”不是通过：每章要求输出误差、守恒、收敛、力/力矩或性能等可验证指标。
- online stable 可能已超出 3.11.0；编译事实以仓库 headers 与本地官方 docs 为准。

## 资料依据

- 本地官方文档：`../docs/html/`
- 官方源码与 notebooks（只作研究/核对）：`../mujoco/`
- 示例唯一构建依赖：`mujoco-3.11.0/`
- 官方 notebooks：Python tutorial、mjSpec、rollout、LQR、least-squares、MJX APG

SDK 许可证和第三方声明见 `mujoco-3.11.0/LICENSE` 与 `THIRD_PARTY_NOTICES`。模型、mesh、texture 和第三方 plugin 仍需分别检查其许可证。
