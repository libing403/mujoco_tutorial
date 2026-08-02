# 第 35 章　生产部署、生态互操作与版本迁移

一个模型在 `simulate` 中能动，只证明它能被当前机器加载。本章建立从研究模型到生产资产的交付链：SDK/ABI、模型包、plugin、平台、日志与回归；同时说明 Python、Unity、OpenUSD、Menagerie 在各自边界内怎样与本书 C++ 主线协作。

## 35.1 学习目标

- 为 Linux/Windows/macOS 交付正确 SDK、runtime library 与 plugins；
- 选择 MJCF、MJB、MJZ、USD 的权威源和转换方向；
- 在 C++、Python、Unity 间定义状态/control/sensor 接口；
- 用 changelog 驱动版本升级并建立行为回归；
- 处理 error/warning、NaN、solver failure 和 crash snapshot。

## 35.2 交付物清单

```text
application
├── executable
├── MuJoCo runtime library
├── plugin libraries + transitive dependencies
├── models (MJCF/MJZ + mesh/texture/license)
├── config (solver/control/sensor/deployment profile)
├── VERSION / build manifest / hashes
└── smoke tests + golden metrics
```

本仓库 vendored SDK 是 Linux x86-64，能让相同平台 clone 后直接构建。它不可能在 Windows、macOS、ARM 上通用；跨平台仓库应按 platform/architecture 提供对应官方 SDK artifact，或由 package manager/CI 下载锁定版本并校验 hash。

不要把 MuJoCo 源码仓库当示例 build dependency。若产品确实需要自定义构建，应在独立 toolchain job 产生 versioned SDK，再让应用只消费其 headers/library。

## 35.3 动态链接与 RPATH

本书极简 CMake 使用仓库相对 SDK 并写 BUILD_RPATH，适合 build tree 直接运行。安装产品时应设计 INSTALL_RPATH：Linux 常用 `$ORIGIN` 相对目录，macOS 用 `@rpath/@loader_path`，Windows 将 DLL 放 executable 搜索路径。

检查：

```bash
ldd ./demo
readelf -d ./demo
```

确认实际加载仓库/产品指定版本，而不是系统中另一个 `libmujoco.so`。plugin 的 transitive libraries 同样要检查。不要依赖用户手工设置全局 `LD_LIBRARY_PATH`，它会污染同 shell 的其他应用。

## 35.4 模型容器选择

| 格式 | 优点 | 风险/用途 |
|---|---|---|
| MJCF + assets | 可读、可 review、可程序生成 | 多文件路径/发布易漏 |
| MJB | 加载快、单个编译结果 | 与版本/平台兼容边界更强，不可作为长期作者源 |
| MJZ | 打包模型和依赖 | 仍需记录版本、license、hash |
| URDF | 机器人生态交换广 | actuator/contact/default 等 MuJoCo 语义不足 |
| OpenUSD | 大场景组合、DCC/数字孪生 | composition 与 physics schema 转换可能损失语义 |

推荐 MJCF/spec 是机器人动力学权威源，mesh/CAD 有独立资产流水线；发布可生成 MJZ/MJB，但能从源重新构建。若 USD 是企业场景权威源，建立单向自动导入 + audit，不要让工程师双向手工同步两份动力学。

## 35.5 Python 互操作

官方 Python bindings 覆盖 C API，并提供 NumPy view、named access、viewer、Renderer、mjSpec、rollout 和 minimize。适合研究迭代、可视化、数据处理与训练；C++ 适合嵌入实时产品和稳定服务。

跨语言接口应交换有 schema 的数据，而非“把整个 mjData 序列化”：

- model hash/version、`nq/nv/nu/nsensor`；
- state spec 与字段顺序；
- joint/actuator/sensor names + addresses；
- units/frame/time stamps；
- control period、hold、latency；
- warning/termination code。

NumPy array 往往是 model/data 内存 view，model/data 销毁后 view 失效。Python callback 跨语言 hot path 开销高且受解释器线程影响；高频控制/传感扩展更适合 native plugin。

旧 `mujoco-py` 不应作为新项目基础。迁移要区分 API 名称变化与 physics version 变化，先固定同一 MuJoCo model/trajectory 做对照。

## 35.6 Unity plug-in

Unity integration 将 MJCF importer、MuJoCo physics scene 与 Unity GameObjects/components 连接，适合 HMI、VR、可视化和游戏引擎内容。必须明确状态所有权：

- MuJoCo body pose 是 physics truth；
- Unity Transform 是显示/交互表示；
- 每 frame 同步方向只能有一个权威写入者；
- 用户 drag 应转成 mocap/perturb/control，不应任意覆盖 dynamic body transform。

编辑器 importer 与 runtime importer 的 asset path/lifecycle 不同。native plugin 与 Unity package version 要匹配；IL2CPP、platform architecture 和 rendering backend 都应进入 CI matrix。

Unity 的 hierarchy 不等于 MuJoCo kinematic tree，scale 也不能随意施加在动态父节点。导入后重新审计 joint axis、inertia、collision layers、sensor frame 和 mesh units。

## 35.7 OpenUSD

USD 的 layer、reference、variant、payload 为大型场景组合而设计；MJCF include/default/attach 服务机器人模型复用，语义不同。MuJoCo OpenUSD 支持围绕 physics schema、file-format plugin 和导入导出持续演进。

转换报告至少比较：

- body/joint tree 与 DoF；
- mass/inertia/units/up-axis；
- collision primitive/mesh approximation；
- material/texture；
- actuator/sensor/tendon/equality 是否有对应 schema；
- custom/plugin data；
- name/path 映射。

视觉看起来相同不代表动力学相同。用第 13、15、16、18 章实验对转换后模型做数值审计。

## 35.8 Menagerie 与 Model Gallery

官方 Menagerie 提供机械臂、夹爪、灵巧手、人形、四足等高质量模型，是学习真实资产组织的最佳素材之一。使用前检查 repository/model license、最低 MuJoCo version、mesh license、actuator interface 与 keyframes。

接入顺序：

1. 在指定 MuJoCo version compile，无 warning；
2. 打印规模、joint/actuator/sensor mapping；
3. 查看 inertia、visual/collision split；
4. keyframe 和静态 gravity/contact 测试；
5. actuator range/gear/ctrl semantics；
6. timestep/solver convergence；
7. 接入自己的 controller；
8. 保存 upstream commit 与本地 patch。

“官方模型”不等于对应你具体实机序列号的精确 digital twin；电机、摩擦、线缆、传感偏置仍需标定。

## 35.9 Changelog 驱动升级

从版本 A 到 B：

1. 冻结 A 的 SDK、MJCF、MJB 生成方式、初态/control 和 golden metrics；
2. 阅读跨越的每条 changelog，按 compiler/engine/solver/API/plugin/binding 分类；
3. 用 B 重新 compile XML，不复用旧 MJB；
4. 清除 warning/deprecation，比较 `nq/nv/nu/nbody/ngeom/nsensor`；
5. 比较 model constants、静态力矩、短轨迹、接触 wrench、solver 与性能；
6. 对行为变化归因：bug fix、default 改变、数值路径或模型依赖未声明；
7. 更新基线必须由工程 review 解释，而不是直接覆盖 golden files。

online stable 文档可能已超过 vendored 3.11.0。编译事实以 SDK headers、本地 3.11 docs 为准；online 用于发现新能力，不能在 3.11 示例中调用未来 API。

## 35.10 Error、warning 与 NaN

load/compile 使用 error buffer 或 spec error；runtime warning 记录在 data warning counters，并可通过 warning callback/handler 集中处理。建议策略：

- bad qpos/qvel/qacc：立即停止该 environment，保存 state/control/model hash；
- contact/constraint buffer full：模型 size capacity 或场景异常，不能忽略；
- solver warning：保存 ncon/nefc、iterations、min distance、质量尺度；
- repeated warning：rate limit 日志但保留首次和计数；
- plugin error：记录 plugin name/instance/config。

NaN 发生后继续 rollout 会污染 training batch。输出显式 termination reason 和有效步数，剩余 buffer 用约定值填充，不让下游误当正常数据。

## 35.11 Crash snapshot

最小复现 bundle：

- 原始或规范化 MJCF + 全部 assets/plugin；
- MuJoCo exact version/build/platform；
- `mj_getState` 指定 spec 的 state；
- control/applied force/mocap/equality/plugin/controller state；
- 随机 seed 与前若干 control；
- option/compiler overrides；
- warning/error text 与 step index。

只保存 qpos 常常无法复现 contact solver/plugin/activation 问题。snapshot 格式应 versioned，并验证 round-trip 后下一步结果一致。

## 35.12 Security

模型和 assets 是输入数据：限制 archive 解压路径穿越、文件大小/数量、mesh/texture 尺寸、resource provider URI 与网络权限。plugin 是 native code，只加载可信签名/白名单库；不能把不可信 `.so` 当普通模型资产。

服务端编译不可信模型应使用进程隔离、资源限额和 timeout。VFS/MJZ 提高便携性，不自动提供安全沙箱。

## 35.13 性能发布门槛

记录目标硬件上的：compile/load latency、first-step latency、steady steps/s、p50/p95/p99 step time、memory、最大 contact workload、render/readback（如需要）。绑定 CPU frequency/affinity 并区分 debug/release。

性能回归要有容忍区间和多次统计。单次快 5% 可能是 turbo/noise，单次慢 5% 也不应自动阻止正确性修复；报告 effect size 与任务影响。

## 35.14 常见误区

- vendored Linux SDK 被描述成跨平台；
- runtime 实际链接系统旧 MuJoCo；
- MJB 当长期唯一作者源；
- USD/MJCF 双向手工同步；
- Python NumPy view 超过 data 生命周期；
- Unity Transform 与 MuJoCo body 双向写；
- 升级后直接覆盖 golden trajectory；
- crash 只保存 qpos；
- 加载用户提供的 native plugin；
- online latest API 混入 3.11.0 示例。

## 35.15 习题与答案

1. 为什么本仓库 SDK 不能保证 ARM/Windows 可用？  
   **答案：**预编译 library 与 OS、architecture、ABI 绑定，需要对应平台官方发行包。

2. MJB 升级时为什么应从 XML 重编？  
   **答案：**MJB 是版本相关编译结果；XML/spec 才是可迁移的作者语义源。

3. Unity 与 MuJoCo 谁应拥有 dynamic body truth？  
   **答案：**使用 MuJoCo physics 时通常 MuJoCo 拥有，Unity Transform 消费同步结果；交互转成明确输入。

4. crash snapshot 为什么需要 plugin/controller state？  
   **答案：**未来 force/control 依赖其历史，仅 qpos/qvel 不足以重现下一步。

5. plugin 为什么比 MJCF 更高风险？  
   **答案：**plugin 是进程内 native executable code，可访问内存/系统资源，必须只加载可信代码。
