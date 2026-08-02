# 第 34 章　Engine plugin、VFS、MJZ 与资源扩展

> 本书示例代码仓库：[libing403/mujoco_tutorial](https://github.com/libing403/mujoco_tutorial)

前面的章节都在 MuJoCo 已有能力范围内建模。本章讨论另一类工程问题：传感器模型、执行器动力学或碰撞形状超出了内置元素；模型资产来自内存、数据库或网络；团队还需要把模型和依赖可靠地交付到另一台机器。MuJoCo 分别用 engine plugin、VFS、resource provider、encoder/decoder 与 MJZ 解决这些问题。

扩展代码会进入模型编译或动力学流水线，接口虽少，责任却很重。本章不仅列 API，还从“类型注册—模型实例—运行时状态—动态库部署”完整实现一个可加载的传感器插件。

## 34.1 学习目标

完成本章后，你应能够：

- 判断何时使用 callback、engine plugin、resource provider 或 decoder；
- 解释 plugin type、instance、`mjModel` 和 `mjData` 的关系；
- 编写、构建、加载并验证一个真正的 `.so` engine plugin；
- 正确处理 attribute、`plugin_state`、`plugin_data`、复制和销毁；
- 理解 actuator、sensor、passive 与 SDF 四类能力的回调契约；
- 用 VFS 加载纯内存模型，并理解资源 URI 的路由过程；
- 为动态库、模型资产和 MuJoCo ABI 制定可复现的部署方案。

## 34.2 先选择正确的扩展层

| 需求 | 首选机制 | 原因 |
|---|---|---|
| 单一程序中的控制律或实验性被动力 | global callback | 接入最快，不需要 MJCF 扩展声明 |
| 可在 MJCF 中声明、可配置、可复用的执行器/传感器/被动力 | engine plugin | 有实例、状态和生命周期协议 |
| 新的隐式碰撞几何 | SDF engine plugin | 直接接入碰撞查询 |
| 少量已知文件来自内存 | VFS | 接口最小，适合嵌入和测试 |
| `asset://`、数据库、网络或内容寻址资源 | resource provider | 按 URI 前缀统一路由 |
| 新模型格式转换为 `mjSpec` | decoder | 进入 MuJoCo 的标准编译流程 |
| 将 spec/model 写为其他格式 | encoder | 对称的导出扩展点 |

不要只因为 plugin “更高级”就使用它。若控制算法只服务于一个 executable，callback 往往更清楚；若一个模型文件需要声明多个独立实例，并携带各自配置和状态，plugin 才真正产生价值。plugin 也不是通用 RPC 容器：`compute` 位于 engine 热路径，网络访问、阻塞锁、频繁分配和日志输出都会直接破坏仿真实时性。

## 34.3 四层对象：最容易混淆的核心

一个 plugin 的数据分布在四个层次：

| 层次 | 数量与寿命 | 典型内容 |
|---|---|---|
| plugin type | 进程中一个，全局注册 | 名称、attribute 列表、函数指针、capability |
| plugin instance | `mjModel` 中零到多个 | 某个编码器实例、PID 实例及其只读配置 |
| `plugin_state` | 每个 `mjData`、每个 instance 一段 | 必须随仿真状态复制/重置的数值状态 |
| `plugin_data` | 每个 `mjData`、每个 instance 一个指针槽 | 已解析参数、查找表、scratch 等 C++ 对象 |

因此，同一个 type 可以在模型里出现多次，同一个 `mjModel` 又可以创建许多 `mjData`。做并行 rollout 时，各 trajectory 共享只读模型，却必须拥有独立运行时状态。

```text
进程注册表
└── book.sensor.calibrated_joint（type）
    └── mjModel
        ├── encoder_left（instance + config）
        └── encoder_right（instance + config）
            ├── mjData A：plugin_state + plugin_data
            └── mjData B：plugin_state + plugin_data
```

把可变对象挂在全局变量上，会让不同模型和不同 rollout 相互污染；把模型常量每步重新解析，则会浪费热路径时间。

## 34.4 `mjpPlugin`：扩展协议本身

注册从零初始化结构体开始：

```cpp
mjpPlugin plugin;
mjp_defaultPlugin(&plugin);
plugin.name = "my_company.sensor.encoder";
plugin.capabilityflags = mjPLUGIN_SENSOR;
plugin.nstate = ...;
plugin.init = ...;
plugin.reset = ...;
plugin.compute = ...;
int slot = mjp_registerPlugin(&plugin);
```

`mjp_defaultPlugin` 很重要：它把当前版本新增的可选字段初始化为安全值。不要用未初始化的栈对象，也不要依赖自己记住结构体的全部字段。

名称是进程级身份，应使用组织命名空间，避免 `pid`、`sensor` 这类易冲突名字。注册表是全局的，`mjp_registerPlugin` 本身线程安全；同名且定义相同的重复注册会被忽略，同名但定义不同会触发错误。可用以下 API 检查注册结果：

```cpp
int slot = -1;
const mjpPlugin* definition = mjp_getPlugin("my_company.sensor.encoder", &slot);
const mjpPlugin* by_slot = mjp_getPluginAtSlot(slot);
int count = mjp_pluginCount();
```

slot 只适合当前进程内查询，不应写入文件或网络协议；稳定身份是名称。

## 34.5 从源码到动态插件

生产插件通常是 shared library。库被加载时，`mjPLUGIN_LIB_INIT` 生成的平台相关入口自动执行注册函数：

```cpp
mjPLUGIN_LIB_INIT(my_sensor_library) {
  register_sensor();
}
```

宿主必须在加载引用该插件的 XML **之前** 加载库：

```cpp
mj_loadPluginLibrary("./libmy_sensor_library.so");
mjModel* m = mj_loadXML("model.xml", nullptr, error, sizeof(error));
```

顺序不能反过来。MJCF 编译器需要先从全局注册表找到 type，才能验证 attribute、确定状态维数和布局。一个库可以注册多个 type；宿主也可用 `mj_loadAllPluginLibraries` 扫描一个受控目录。扫描方便，但生产系统更适合显式目录、白名单和启动日志，以免加载到同名或不受信任的二进制文件。

### 动态库边界的工程规则

- 插件和宿主应链接兼容的 MuJoCo ABI；发布时记录 `mj_versionString()`。
- plugin 结构体、名称字符串和 attribute 字符串在注册后必须长期有效，通常使用静态存储。
- 不要在卸载动态库后继续使用已注册的函数指针；最简单的策略是让插件库与进程同寿命。
- Linux 上同时检查插件自身及其依赖的 RPATH；“能找到插件”不等于“插件依赖都能找到”。
- 动态库是原生代码，加载即执行；只能加载可信产物。

## 34.6 MJCF：声明类型、创建实例、绑定消费者

下面声明一个带两个参数的校准实例：

```xml
<extension>
  <plugin plugin="book.sensor.calibrated_joint">
    <instance name="encoder_calibration">
      <config key="gain" value="57.2957795"/>
      <config key="offset" value="1.5"/>
    </instance>
  </plugin>
</extension>
```

传感器再通过 instance 名称绑定：

```xml
<sensor>
  <plugin name="joint_encoder" instance="encoder_calibration"/>
</sensor>
```

也可以在具体元素中内联 plugin 与 config。显式 instance 更适合多个元素共享同一配置，且能清楚地区分“插件类型名”和“模型实例名”。

插件先声明允许的 attribute：

```cpp
static const char* attributes[] = {"gain", "offset"};
plugin.nattribute = 2;
plugin.attributes = attributes;
```

然后在 `init` 中读取：

```cpp
const char* text = mj_getPluginConfig(m, instance, "gain");
```

返回的是 `mjModel` 拥有的字符串。正确做法是在 `init` 中解析一次，检查缺失值、非法字符、有限性和物理范围；错误应让 `init` 返回 `-1`，使 `mj_makeData` 明确失败。实验为突出主线，使用已知合法的固定配置；生产代码应完成全部校验。

## 34.7 生命周期：每个回调在何时发生

```text
load library → register type → parse/compile model
                                  │
                                  ├─ nstate / nsensordata
                                  ▼
                              mj_makeData
                                  │ init
                                  │ reset
                                  ├──────── compute
                                  ├──────── advance
                                  ├──────── visualize
                                  │
              mj_copyData ────────┤ copy
                                  │
                              mj_deleteData
                                    destroy
```

| 回调 | 责任 | 常见错误 |
|---|---|---|
| `nstate` | 声明每 instance 需要多少个 `mjtNum` | 维数依赖运行时数据 |
| `nsensordata` | 声明某 plugin sensor 的输出维数 | 与实际写入数量不同 |
| `init` | 为一个 `mjData` 创建对象、解析配置 | 每步才解析或分配 |
| `reset` | 重置数值状态和运行时对象 | 忘记可重复 reset |
| `compute` | 根据 capability 计算输出/力 | 阻塞、打印、越界写 |
| `advance` | 积分发生时推进离散内部状态 | 在 `compute` 中偷偷改变历史 |
| `copy` | 深复制 `plugin_data` 对象 | 浅复制导致串扰或 double free |
| `destroy` | 释放该 `mjData` 的对象 | 与 `init` 不成对 |
| `visualize` | 向场景追加装饰 geom | 超过场景容量或修改动力学状态 |

### `plugin_state` 还是 `plugin_data`

判断标准不是“是否属于插件”，而是“是否属于仿真状态”。积分器状态、延迟线中必须被保存的数值、可参与 rollout 快照的内部状态，应放入 engine 管理的 `plugin_state`。由参数构建的查找表、缓存对象和 C++ 容器放入 `plugin_data`。

若 `plugin_data` 只保存不可变对象，也仍要决定复制所有权：深复制最直观；共享则需要引用计数且必须保证线程安全。本章实验用 `copy` 创建独立 `Calibration`，让所有权关系一目了然。

## 34.8 四类 capability

`capabilityflags` 是位图，一个 type 可以组合能力，但只实现需要的接口通常更容易测试。

### Sensor

`nsensordata` 决定输出维数。`compute` 不能假设一个 instance 只绑定一个 sensor，应遍历：

```cpp
for (int sensor = 0; sensor < m->nsensor; ++sensor) {
  if (m->sensor_plugin[sensor] != instance) continue;
  mjtNum* output = d->sensordata + m->sensor_adr[sensor];
  // 写入恰好 m->sensor_dim[sensor] 个值
}
```

`needstage` 声明输出依赖的最早动力学阶段：

- `mjSTAGE_POS`：位置、位姿、几何等即可；
- `mjSTAGE_VEL`：还依赖速度量；
- `mjSTAGE_ACC`：还依赖加速度/力相关结果。

选得太早会读到未更新量，选得过晚会迫使引擎执行本不需要的阶段。

### Actuator

actuator plugin 可负责执行器力与 activation dynamics。若实现内部 activation 导数，使用 `actuator_act_dot` 写对应 `d->act_dot`；`compute` 负责当前动力学求值。控制输入仍来自 `d->ctrl`，不要在插件中隐藏上层控制器难以观测的全局状态。

### Passive force

passive plugin 在 `compute` 中向相应广义力累计。必须遵守“累加而非覆盖”的约定，并注意隐式积分器对速度相关力及其导数的要求。高刚度弹簧或阻尼不只是插件实现问题，也会改变数值稳定性。

### SDF

SDF 用有符号距离 \(\phi(p)\) 与梯度描述隐式表面，并提供 static distance、attribute conversion 和 AABB 等接口。梯度可用中心差分验证：

\[
\nabla\phi(p)^T v \approx
\frac{\phi(p+\epsilon v)-\phi(p-\epsilon v)}{2\epsilon}.
\]

AABB 太小会让 broad phase 漏掉真实接触；distance 与 gradient 不一致会产生错误法向；过于昂贵的距离函数会放大碰撞迭代成本。还要确认目标后端是否支持该插件，C engine 可用不代表 MJX 或其他后端自动具备等价实现。

## 34.9 实时性、并发与可微性

插件代码与 engine 同步执行，应把它当作实时内核的一部分：

- 在 `init` 完成解析、分配和索引查找；
- 在 `compute` 使用有界循环，避免 I/O、锁竞争和异常；
- 不用静态可变 scratch；每个 `mjData` 拥有自己的 scratch；
- 同一 `mjModel` 的多个 `mjData` 可能并行推进，回调必须可并发；
- 输出出现 NaN 时尽早定位，不要让其扩散到求解器。

插件还会影响优化与系统辨识。若插件内部有离散状态、饱和、死区或不可微分分支，有限差分得到的导数可能不稳定。至少应测试：重复运行的确定性、`mj_copyData` 后的轨迹独立性、reset 可重复性、输出有限性，以及对关键输入的有限差分。

## 34.10 独立实验：动态校准编码器插件

`examples/43_sensor_plugin/` 只有一个 `main.cc`，但 CMake 将它编译两次：定义 `BUILD_PLUGIN` 时得到动态库；未定义时得到宿主程序。动态库注册三维传感器，依次输出仿真时间、关节弧度值，以及

\[
y_{deg}=gain\,q+offset.
\]

模型设置 `gain=180/\pi`、`offset=1.5`，模拟带零偏的角度编码器。这比直接返回 `qpos` 多展示了四个关键知识点：attribute 声明、instance config、`plugin_data` 生命周期与动态装载。

```bash
cd examples/43_sensor_plugin
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml ./build/libbook_sensor_plugin.so
```

预期输出形式如下，具体角度取决于 100 步后的动力学状态：

```text
slot=0, dim=3, time=0.198 s, angle=..., calibrated=... deg
```

阅读源码时按下面顺序追踪，而不是从第一行机械读到最后一行：

1. CMake 如何用同一源码生成 `.so` 与宿主；
2. `mjPLUGIN_LIB_INIT` 如何在加载时注册 type；
3. 宿主为何先 `mj_loadPluginLibrary` 再 `mj_loadXML`；
4. MJCF 如何创建 instance 并绑定 sensor；
5. `init/copy/destroy` 如何管理每个 `mjData` 的 `Calibration`；
6. `compute` 如何通过 `sensor_adr` 写入三维结果。

### 用 `simulate` 查看模型

`view.xml` 不引用自编译插件，专门用于快速检查几何、关节和相机。这是因为独立宿主已显式加载实验 `.so`，而直接启动官方 `simulate` 时并不会自动知道该实验 build 目录。若要让 `simulate` 执行插件模型，应把经过审核的 `.so` 放入它扫描的插件目录或编写显式加载插件的 viewer 宿主；不要误以为一个进程加载的插件会自动出现在另一个进程中。

<!-- EMBEDDED_EXAMPLE_BEGIN: 43_sensor_plugin -->
### 可视化运行与效果

```bash
../../mujoco-3.11.0/bin/simulate view.xml
```

该命令使用发布包自带的官方 `simulate` 界面，可暂停、单步、施加扰动并开启接触等可视化标志。

![43_sensor_plugin 实验运行效果](../assets/experiments/43_sensor_plugin.png)

*43_sensor_plugin 的真实 MuJoCo 原生渲染结果。*

### 实验完整源码

以下文件与 `examples/43_sensor_plugin/` 中可直接编译的版本一致。

#### 模型文件：`model.xml`

```xml
<mujoco model="calibrated encoder plugin">
  <option timestep="0.002"/>
  <extension>
    <plugin plugin="book.sensor.calibrated_joint">
      <instance name="encoder_calibration">
        <config key="gain" value="57.2957795"/>
        <config key="offset" value="1.5"/>
      </instance>
    </plugin>
  </extension>
  <worldbody>
    <light pos="0 -2 3"/>
    <body pos="0 0 .8">
      <joint name="hinge" axis="0 1 0" damping=".1"/>
      <geom type="capsule" fromto="0 0 0 0 0 -.4" size=".03"
            mass="1" rgba=".3 .7 .9 1"/>
    </body>
  </worldbody>
  <sensor>
    <plugin name="joint_encoder" instance="encoder_calibration"/>
  </sensor>
</mujoco>
```

#### 可视化模型：`view.xml`

```xml
<mujoco model="calibrated encoder geometry">
  <worldbody>
    <light pos="0 -2 3"/>
    <body pos="0 0 .8">
      <joint name="hinge" axis="0 1 0" damping=".1"/>
      <geom type="capsule" fromto="0 0 0 0 0 -.4" size=".03"
            rgba=".3 .7 .9 1"/>
    </body>
  </worldbody>
</mujoco>
```

#### 程序源码：`main.cc`

```cpp
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#ifdef BUILD_PLUGIN

struct Calibration {
  mjtNum gain;
  mjtNum offset;
};

void register_sensor() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "book.sensor.calibrated_joint";
  static const char* attributes[] = {"gain", "offset"};
  plugin.nattribute = 2;
  plugin.attributes = attributes;
  plugin.capabilityflags = mjPLUGIN_SENSOR;
  plugin.needstage = mjSTAGE_POS;
  plugin.nstate = +[](const mjModel*, int) { return 0; };
  plugin.nsensordata = +[](const mjModel*, int, int) { return 3; };
  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* calibration = new Calibration{
        std::strtod(mj_getPluginConfig(m, instance, "gain"), nullptr),
        std::strtod(mj_getPluginConfig(m, instance, "offset"), nullptr)};
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(calibration);
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Calibration*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.copy = +[](mjData* dest, const mjModel*, const mjData* src, int instance) {
    auto* source = reinterpret_cast<Calibration*>(src->plugin_data[instance]);
    dest->plugin_data[instance] = reinterpret_cast<uintptr_t>(new Calibration(*source));
  };
  plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability) {
    if (!(capability & mjPLUGIN_SENSOR)) return;
    auto* calibration = reinterpret_cast<Calibration*>(d->plugin_data[instance]);
    for (int sensor = 0; sensor < m->nsensor; ++sensor) {
      if (m->sensor_plugin[sensor] != instance) continue;
      mjtNum angle = d->qpos[0];
      mjtNum* output = d->sensordata + m->sensor_adr[sensor];
      output[0] = d->time;
      output[1] = angle;
      output[2] = calibration->gain * angle + calibration->offset;
    }
  };
  mjp_registerPlugin(&plugin);
}

mjPLUGIN_LIB_INIT(book_sensor_plugin) { register_sensor(); }

#else

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "用法: %s model.xml plugin.so\n", argv[0]);
    return 1;
  }

  mj_loadPluginLibrary(argv[2]);
  int slot = -1;
  if (!mjp_getPlugin("book.sensor.calibrated_joint", &slot)) {
    std::fprintf(stderr, "动态库未注册预期的 plugin\n");
    return 1;
  }

  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "%s\n", error);
    return 1;
  }
  mjData* d = mj_makeData(m);
  d->qpos[0] = 0.7;
  for (int i = 0; i < 100; ++i) mj_step(m, d);

  int sensor = mj_name2id(m, mjOBJ_SENSOR, "joint_encoder");
  const mjtNum* value = d->sensordata + m->sensor_adr[sensor];
  std::printf("slot=%d, dim=%d, time=%.3f s, angle=%.3f rad, calibrated=%.3f deg\n",
              slot, m->sensor_dim[sensor], value[0], value[1], value[2]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}

#endif
```

#### 构建文件：`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(43_sensor_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(MUJOCO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../mujoco-3.11.0)

add_library(book_sensor_plugin SHARED main.cc)
target_compile_definitions(book_sensor_plugin PRIVATE BUILD_PLUGIN)
target_include_directories(book_sensor_plugin PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(book_sensor_plugin PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(book_sensor_plugin PRIVATE mujoco)
set_target_properties(book_sensor_plugin PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)

add_executable(demo main.cc)
target_include_directories(demo PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(demo PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(demo PRIVATE mujoco)
set_target_properties(demo PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
```
<!-- EMBEDDED_EXAMPLE_END: 43_sensor_plugin -->

## 34.11 调试插件：按边界逐层排错

动态插件失败时，用下面的顺序比盲目改 XML 更有效：

1. **二进制层**：`.so` 是否存在，架构是否匹配，依赖是否都能解析；
2. **注册层**：加载后 `mjp_getPlugin(name)` 是否成功，名称是否完全一致；
3. **编译层**：attribute 是否被声明，instance 和绑定关系是否正确；
4. **data 层**：`mj_makeData` 是否调用 `init` 成功，指针所有权是否正确；
5. **流水线层**：`needstage`、capability bit、地址和维数是否正确；
6. **数值层**：输出是否有限、量纲是否一致、reset/copy 后是否确定。

Linux 可用 `ldd libbook_sensor_plugin.so` 检查依赖，用 `nm -D` 检查导出入口；但最终是否注册成功应由 `mjp_getPlugin` 断言。插件中若需要日志，限制在加载和初始化阶段，并给出 instance 名称与配置；不要在每个 step 打印。

## 34.12 VFS：最小的内存资源层

VFS 是一张由文件名到字节数组的内存表：

```cpp
mjVFS vfs;
mj_defaultVFS(&vfs);
mj_addBufferVFS(&vfs, "model.xml", xml, size);
mjModel* m = mj_loadXML("model.xml", &vfs, error, sizeof(error));
mj_deleteVFS(&vfs);
```

添加 buffer 后，VFS 管理自己的内部副本。它适合嵌入式程序、单元测试 fixture、程序生成的小模型，以及“先完整下载并校验，再原子加载”的流程。编译器解析 include、mesh 和 texture 时，名称必须与 VFS 中的规范化名称相符；目录、斜杠和大小写不一致是最常见的失败原因。

VFS 不等于安全沙箱。来自不可信来源的超大 mesh、texture 或复杂 XML 仍会消耗内存和编译时间，应用需要在加载前限制大小、类型和资源数量。

## 34.13 独立实验：从 VFS 加载完整模型

`examples/44_vfs_model/` 把 MJCF 字符串加入 VFS，再以 `embedded.xml` 这个内存资源名编译和仿真。程序目录不需要真正的 `model.xml`，因此能直接证明读取发生在内存中。

```bash
cd examples/44_vfs_model
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo
```

把示例扩展为含 mesh 的模型时，应把 XML 与 mesh bytes 都加入同一个 VFS，并让 XML 的 `file` 名与 VFS key 完全匹配。

<!-- EMBEDDED_EXAMPLE_BEGIN: 44_vfs_model -->
### 可视化运行与效果

```bash
../../mujoco-3.11.0/bin/simulate view.xml
```

该命令使用发布包自带的官方 `simulate` 界面，可暂停、单步、施加扰动并开启接触等可视化标志。

![44_vfs_model 实验运行效果](../assets/experiments/44_vfs_model.png)

*44_vfs_model 的真实 MuJoCo 原生渲染结果。*

### 实验完整源码

以下文件与 `examples/44_vfs_model/` 中可直接编译的版本一致。

#### 可视化模型：`view.xml`

```xml
<mujoco model="VFS embedded model">
  <worldbody>
    <light pos="0 -2 3"/>
    <geom type="plane" size="2 2 .1" rgba=".2 .25 .3 1"/>
    <body pos="0 0 .5">
      <freejoint/>
      <geom type="sphere" size=".1" mass="1" rgba=".9 .4 .2 1"/>
    </body>
  </worldbody>
</mujoco>
```

#### 程序源码：`main.cc`

```cpp
#include <cstdio>
#include <cstring>
#include <mujoco/mujoco.h>

int main() {
  const char xml[] = R"(<mujoco model="memory model">
    <worldbody><body><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody>
  </mujoco>)";
  mjVFS vfs; mj_defaultVFS(&vfs);
  if (mj_addBufferVFS(&vfs, "embedded.xml", xml, std::strlen(xml)) != 0) {
    std::fprintf(stderr, "无法向 VFS 添加内存模型\n"); mj_deleteVFS(&vfs); return 1;
  }
  char error[1024] = {0};
  mjModel* m=mj_loadXML("embedded.xml", &vfs, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); mj_deleteVFS(&vfs); return 1; }
  mjData* d=mj_makeData(m);
  for (int k=0;k<100;++k) mj_step(m,d);
  std::printf("loaded entirely from VFS: model=%s nq=%lld z=%.6f\n",
              m->names, static_cast<long long>(m->nq), d->qpos[2]);
  mj_deleteData(d); mj_deleteModel(m); mj_deleteVFS(&vfs); return 0;
}
```

#### 构建文件：`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(44_vfs_model LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
add_executable(demo main.cc)

set(MUJOCO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../mujoco-3.11.0)
target_include_directories(demo PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(demo PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(demo PRIVATE mujoco)
set_target_properties(demo PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
```
<!-- EMBEDDED_EXAMPLE_END: 44_vfs_model -->

## 34.14 Resource provider：把 URI 接到数据源

VFS 适合应用主动准备一组文件；resource provider 则适合按 URI 动态打开资源。provider 注册一个前缀，例如 `asset`，随后 `asset://robot/arm.xml` 可被路由到数据库、对象存储或内容寻址缓存。

provider 通常实现 open、read、close 和 modified timestamp，并可选实现 mount、unmount、write。设计时要明确：

- open 返回的 resource 与 buffer 由谁拥有；
- close 后所有指针立即失效；
- timestamp/etag 如何驱动缓存失效；
- 网络超时、重试和离线缓存如何处理；
- 凭据不写入模型，也不进入日志；
- URI 规范化后仍不能越过允许的命名空间。

注册前缀不能与已有前缀互为前缀，否则 `asset:` 与 `asset-cache:` 之类路由会产生歧义。资源读取发生在 parse/compile 阶段；不要让仿真 step 临时访问网络。

## 34.15 Decoder、encoder 与 content type

resource provider 回答“字节从哪里来”，decoder 回答“这些字节如何成为模型”。decoder 根据 content type 或扩展名把资源转换为调用者拥有的 `mjSpec`，随后仍走标准编译器；encoder 则把 `mjSpec/mjModel` 写到 resource。

一个可靠的导入器不仅要创建 body 和 geom，还要定义坐标系、单位、惯量、材质、关节限制和名称冲突的处理规则。导入完成后应执行第 23 章的 model audit，而不是把“解析成功”等同于“物理正确”。decoder 返回的 `mjSpec` 由调用者最终通过 `mj_deleteSpec` 销毁。

content type 比文件后缀更可靠。网络响应、归档内部成员和无扩展名内容都可能没有可信后缀；后缀可以作为提示，但不应成为唯一协议。

## 34.16 MJZ：可交付的模型包

一个 MJCF 往往依赖 include、mesh、texture 和其他资产。只复制主 XML，常出现“作者电脑能运行，用户电脑资源丢失”。MJZ 的意义是把模型及依赖作为一个可携带资源包处理，并接入 resource/decoder 机制。

工程发布包还应包含：

- 入口模型与全部递归依赖；
- 每个资产的许可证和来源；
- 内容 hash 与构建版本；
- MuJoCo 版本、目标平台和所需插件列表；
- 加载后的 model audit 结果或回归基线。

插件动态库通常具有平台相关性，不能把一份 Linux `.so` 当作跨平台模型资产。更稳妥的做法是把模型资产与各平台插件产物分别版本化，并由 manifest 建立对应关系。

## 34.17 生产化检查清单

在把扩展交给机器人项目使用前，逐项确认：

- type 名称带组织命名空间，attribute 名称与单位有文档；
- 动态库在解析模型前加载，并检查了预期 type；
- `init/reset/copy/destroy` 经多 `mjData` 和重复 reset 测试；
- `compute` 不做 I/O、无界分配或高频日志；
- sensor 维数、地址、stage 和 capability 均有断言；
- SDF 的 distance/gradient/AABB 有数值与几何测试；
- 资源 URI 规范化、大小限制、缓存和凭据策略明确；
- 模型包记录 hash、许可证、MuJoCo ABI 与插件版本；
- CI 在干净机器上从仓库发布包独立构建并运行示例；
- 对不支持 plugin 的后端有明确的降级或拒绝策略。

## 34.18 常见误区

- MJCF 编译后才加载插件库；
- 把 type name、instance name 和 sensor name 混为一谈；
- attribute 字符串来自临时对象，注册后指针失效；
- `compute` 每步解析 config、分配内存或打印日志；
- `plugin_data` 在多个 `mjData` 间共享，却没有明确所有权；
- 实现 `destroy` 却遗漏 `copy`，导致浅复制和 double free；
- sensor instance 只写第一个绑定 sensor，或写入超过声明维数；
- SDF gradient 与 distance 不一致，或 AABB 过小；
- provider close 后仍保存 buffer 指针；
- VFS key 与 XML 相对路径不一致；
- 插件库随意扫描，未做来源控制和 ABI 版本约束；
- 看到 C engine plugin 可用，就假设 MJX/Warp 也支持。

## 34.19 习题与参考答案

1. **为什么必须先加载插件库，再编译引用它的 MJCF？**
   **答案：**编译器需要从全局注册表取得 type 定义，才能验证 attribute，并确定 instance 的状态和 sensor 输出布局。

2. **两个 `mjData` 能否共享同一个可变 `plugin_data` 对象？**
   **答案：**通常不能。并行 trajectory 会发生数据竞争和历史串扰；除非对象严格不可变且用清晰的共享所有权管理。

3. **低通滤波器的历史值应放在 `plugin_state` 还是 config？**
   **答案：**放在 `plugin_state`，因为它随时间变化，属于需要 reset、copy 和快照的仿真状态；截止频率等常量才属于 config。

4. **为什么传感器插件要遍历所有 `sensor_plugin[sensor] == instance` 的项？**
   **答案：**一个 instance 可以绑定多个 sensor；只写第一个会让其余输出保持陈旧值或零。

5. **VFS 与 resource provider 的关键区别是什么？**
   **答案：**VFS 是应用预先填充的有限内存文件表；provider 按 URI 前缀动态连接通用数据源。

6. **SDF 的 AABB 太小会发生什么？**
   **答案：**broad phase 可能提前剔除真实相交对象，最终漏掉接触。

7. **为什么插件动态库不能只记录文件名？**
   **答案：**同名文件可能对应不同 ABI、平台或内容；部署还应记录 hash、MuJoCo 版本、目标架构和依赖。

8. **修改实验，使同一 instance 同时绑定两个 plugin sensor，应验证什么？**
   **答案：**两个 sensor 的 `sensor_adr` 不同但三维结果一致，证明 `compute` 没有假设一实例只对应一个 sensor。
