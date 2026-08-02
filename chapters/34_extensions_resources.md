# 第 34 章　Engine plugin、VFS、MJZ 与资源扩展

当原生 actuator、sensor、passive force 或 geom 不能表达任务时，MuJoCo 提供 engine plugin；当资产不来自普通磁盘路径时，提供 VFS、resource provider、decoder/encoder。本章重点是扩展协议和生命周期，而不是把所有业务逻辑塞进全局 callback。

## 34.1 学习目标

- 区分 callback、engine plugin、resource provider 和 model decoder；
- 实现并注册最小 sensor plugin；
- 管理 plugin instance config、state 与 per-data opaque storage；
- 用 VFS 从内存加载 MJCF/assets；
- 理解 MJZ bundle、content type 与 encoder/decoder；
- 处理动态库发现、全局注册和版本兼容。

## 34.2 选择扩展机制

| 需求 | 首选 |
|---|---|
| 简单进程内控制/被动力，只有一个应用 | global callback |
| 可在 MJCF 声明、每实例配置/状态、可复用 | engine plugin |
| `mem://`、网络、数据库读取 bytes | resource provider |
| 新模型文件格式 → `mjSpec` | decoder |
| `mjSpec/mjModel` → 新文件格式 | encoder |
| 少量内存文件覆盖普通路径 | VFS |

plugin 不是任意外部仿真器 RPC 框架。compute 位于 engine hot path，必须确定、快速、线程安全。

## 34.3 Plugin capability

`mjpPlugin.capabilityflags` 可组合：

- `mjPLUGIN_ACTUATOR`：产生 actuator force/activation dynamics；
- `mjPLUGIN_SENSOR`：填充 sensor output；
- `mjPLUGIN_PASSIVE`：向 passive force 累加；
- `mjPLUGIN_SDF`：signed distance、gradient、AABB 等隐式几何接口。

同一 plugin type 可有多个 instance，每 instance 由 MJCF config 区分。`needstage` 告诉 sensor 在 position/velocity/acceleration 哪个 pipeline stage 后才能计算，选得过晚会浪费计算，过早则依赖量尚未有效。

## 34.4 注册与身份

```cpp
mjpPlugin plugin;
mjp_defaultPlugin(&plugin);
plugin.name = "book.sensor.state";
...
int slot = mjp_registerPlugin(&plugin);
```

注册表是进程级全局、注册操作线程安全。相同 name 只有完全相同定义才可重复注册；不同函数指针/attribute 却同名会触发 error。plugin name 应使用组织命名空间，避免 `pid` 这类通用名称。

必须在解析/编译引用它的 MJCF 前注册或加载动态库。官方发布包的 plugin `.so` 通常由应用扫描/`dlopen`，其 constructor 完成注册；部署时要把 plugin libraries 和依赖一起打包并固定 ABI 版本。

## 34.5 Instance config

plugin 声明 attribute names，MJCF 通过 `<config key=... value=.../>` 为 instance 设置字符串值。运行时：

```cpp
const char* value = mj_getPluginConfig(m, instance, "scale");
```

在 init 中解析一次并验证范围，不要每个 compute 重复 `strtod`。错误 config 应使 compile/init 明确失败，不能静默用零。

config 是 model 常量；每个 `mjData` 的运行时对象放在 `d->plugin_data[instance]`，每个 instance 的积分状态放在 engine 管理的 `plugin_state`，并由 `nstate` 声明维数。

## 34.6 生命周期

```text
register type (process)
  → compile instances (model)
    → init per mjData
      → reset
      → compute / advance ...
      → copy when data copied
    → destroy per mjData
  → delete model
```

- `init` 为每个 data 分配 scratch/object；
- `destroy` 成对释放；
- `reset` 初始化 engine-managed plugin state 与 opaque object；
- `copy` 若 plugin_data 含可变对象，必须深复制；
- `compute` 按 capability bit 处理当前阶段；
- `advance` 在积分发生时更新离散 plugin state；
- `visualize` 可追加 decor geoms。

忽略 copy callback 会使 `mj_copyData` 后两个 data 共享一个可变指针，造成 double free 或 trajectory 串扰。

## 34.7 Sensor plugin 输出

`nsensordata` 返回特定 sensor 的输出维数。compute 中遍历 `m->sensor_plugin[sensor_id] == instance` 的 sensor，并写 `d->sensordata + sensor_adr`。不要假设一个 instance 只绑定一个 sensor。

plugin output 的 dimension、datatype、noise/cutoff 等 metadata 应与 sensor definition 一致。下游仍按第 11 章通过 `sensor_adr/sensor_dim` 解包。

## 34.8 SDF plugin

SDF 用 \(\phi(p)\) 的符号距离和梯度描述隐式表面。plugin 提供 distance、gradient、static distance、attribute conversion 与 AABB。gradient 应与 distance 数值一致；可用中心差分验证

\[
\nabla\phi(p)^Tv\approx\frac{\phi(p+\epsilon v)-\phi(p-\epsilon v)}{2\epsilon}.
\]

错误 AABB 会漏 broad-phase candidate，错误 gradient 会产生错误 contact normal。复杂 SDF 还影响 collision iteration 与 MJX/其他 backend feature parity。

## 34.9 VFS

VFS 是小型内存文件表：

```cpp
mjVFS vfs;
mj_defaultVFS(&vfs);
mj_addBufferVFS(&vfs, "model.xml", xml, size);
mjModel* m = mj_loadXML("model.xml", &vfs, error, sizeof(error));
mj_deleteVFS(&vfs);
```

compiler 查找资源时优先 VFS，再按规则访问磁盘/provider。buffer 添加后由 VFS 管理内部副本，可用于嵌入式部署、测试 fixture 和网络下载后的原子加载。

VFS name 必须与 XML asset path 解析后名称一致；目录、斜杠和 case 错误是最常见问题。不要用 VFS 绕开 untrusted model 安全检查：超大 mesh/texture 仍可能消耗大量内存和编译时间。

## 34.10 Resource provider

provider 通过 URI prefix（如 `asset://`）实现 open/read/close、modified timestamp，可选 mount/unmount/write。注册 prefix 不能与现有 prefix 互为前缀，避免路由歧义。

resource object 的 byte buffer 生命周期由 provider 协议决定；compiler 在 read 期间使用，close 后不得访问。网络 provider 应处理超时、缓存、一致性和凭据，且不要在实时 step 中触发资源读取——资源属于 parse/compile 阶段。

## 34.11 Decoder、encoder 与 MJZ

decoder 根据 content type/extension 把 resource 转为调用者拥有的 `mjSpec`；encoder 把 spec/model 写到 resource。URDF、USD 等导入链都可以从这一抽象理解。

MJZ 是可打包模型及依赖的归档工作流，解决“XML 能找到但 mesh/texture 丢失”的部署问题。无论容器格式如何，加载后仍应记录依赖清单、许可证、hash、MuJoCo version，并执行 model audit。

## 34.12 独立实验：单文件 sensor plugin

`examples/43_sensor_plugin/` 在 executable 中注册一个两维 sensor plugin，输出 simulation time 与第一个 joint position。MJCF 引用该 plugin；程序编译模型、step 并按标准 sensor address 读取结果。

```bash
cd examples/43_sensor_plugin
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

示例把 type 注册与应用放在一个源码文件，便于看清协议。生产 plugin 通常构建为独立 shared library，但那会引入额外源码/部署步骤，不适合作为本知识点的最小实验。

`examples/44_vfs_model/` 则把 MJCF 字符串放入 VFS，从 `embedded.xml` 这个内存资源名加载并仿真。它同样不放置无用的 `model.xml`：

```bash
cd examples/44_vfs_model
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo
```

## 34.13 常见误区

- MJCF compile 后才注册 plugin；
- compute 每步解析 config/分配内存/打印日志；
- plugin_data 在多个 data 间共享，未实现 copy/destroy；
- sensor instance 只写第一个绑定 sensor；
- SDF gradient 与 distance 不一致；
- provider prefix 冲突或 resource close 后 buffer 失效；
- VFS asset name 与 XML 相对路径不一致；
- 动态 plugin library 没随应用打包或 ABI 版本不同；
- 看到 C engine plugin 就假设 MJX/Warp 同样支持。

## 34.14 习题与答案

1. plugin config 为何应在 init 解析？  
   **答案：**它是 model 常量，解析一次可验证错误并避免 hot-path 字符串开销。

2. 为什么每个 `mjData` 要有独立 plugin_data？  
   **答案：**并行 trajectory 的运行时缓存/状态必须隔离，否则 data race 和历史串扰。

3. VFS 与 resource provider 的区别？  
   **答案：**VFS 是显式填充的小型内存文件表；provider 按 URI prefix 动态实现通用资源后端。

4. decoder 返回的 spec 谁负责销毁？  
   **答案：**调用者取得所有权，应最终 `mj_deleteSpec`。

5. SDF AABB 太小会怎样？  
   **答案：**broad phase 可能剔除真实相交对象，导致漏碰撞。
