# 第 10 章　有状态执行器、延迟与肌肉模型

理想 torque motor 让 control 在当前步直接变成力。真实电流环、液压缸、气动肌肉和生物肌肉都有内部响应时间：命令先改变 actuator 内部状态，再由状态生成力。加入 activation 后，机械系统从二阶动力学变成三阶，控制器的相位裕度和可实现带宽都会变化。

## 10.1 学习目标

- 理解 `ctrl`、`act`、`actuator_force` 的时序关系；
- 推导 integrator、filter、filterexact activation dynamics；
- 解释 `na`、`actuator_actadr` 和 `actuator_actnum`；
- 使用 actrange 和 `actearly`；
- 设计驱动延迟与饱和模型；
- 理解 MuJoCo muscle 的长度—速度—激活力生成结构。

## 10.2 为什么 activation 是真实状态

无状态 actuator 的力可直接依赖 `u`：

\[
p=p(q,v,u).
\]

有状态 actuator 引入 activation `w`：

\[
\dot w=g(u,w,l,\dot l),\qquad p=p(l,\dot l,w).
\]

机械状态已经包含 `q,v`，再加 `w` 后完整动力学为：

\[
\dot q=N(q)v,qquad
M(q)\dot v=\tau(q,v,w)+J^Tf-c(q,v),qquad
\dot w=g(u,w,l,\dot l).
\]

这不是“输出做一个显示滤波”。`w` 会影响真实力和后续轨迹，必须在状态快照、rollout 和线性化中保存。

## 10.3 act 数组和地址

`m->na` 是 activation state 总数，`d->act[na]` 保存状态。不是每个 actuator 都有一个 act：

- `actuator_actadr[i] == -1`：无状态 actuator；
- 非负地址：第一个 activation 的位置；
- `actuator_actnum[i]`：该 actuator activation 数量。

MuJoCo 3.11 支持 actuator 的输入/输出数量不再都假定为一。简单 SISO 模型中常有 `nactuator=nu=nout`，但通用程序应读取模型计数和地址。

```mermaid
flowchart LR
  C[ctrl 拼接数组] --> A[每个 actuator dynamics]
  A --> W[act 拼接数组]
  W --> F[actuator force outputs]
  F --> Q[qfrc_actuator]
```

## 10.4 integrator dynamics

最简单有状态模型：

\[
\dot w=u.
\]

control 是 activation 的变化率，而不是 activation 本身。离散后近似：

\[
w_{k+1}=w_k+h u_k.
\]

它用于积分速度 servo、累积阀门位置或自定义状态原型。没有 actrange 时，持续非零 control 会让 activation 无界增长。

`intvelocity` shortcut 将 control 解释为位置目标的变化率，activation 成为内部位置目标，再通过 position-like force law 作用于系统。它与直接 velocity actuator 不同：一个积分出目标位置，一个直接调节当前速度误差。

## 10.5 filter dynamics

一阶低通：

\[
\dot w=\frac{u-w}{t},
\]

`t` 是 time constant，通常位于 `actuator_dynprm`。连续阶跃响应：

\[
w(t)=u+(w(0)-u)e^{-t/t_c}.
\]

从 0 到 1：一个 time constant 达到约 63.2%，三个达到 95.0%，五个达到 99.3%。它可近似电流环、阀门或通信/执行延迟的低阶部分。

普通 `filter` 用 Euler 更新：

\[
w_{k+1}=w_k+h\frac{u_k-w_k}{t_c}.
\]

当 `t_c<h` 时会发散；稳定性和响应也依 timestep。

## 10.6 filterexact dynamics

同一连续方程使用解析离散：

\[
w_{k+1}=w_k+(u_k-w_k)(1-e^{-h/t_c}).
\]

对任意正 `t_c` 都稳定，并且在分段常值 control 假设下精确。`h→0` 时 filter 与 filterexact 收敛到同一结果。

机器人驱动的一阶响应通常优先 `filterexact`，因为更换物理 timestep 不会显著改变同一 time constant 的离散极点。普通 filter 仍有教学和特定离散模型用途。

## 10.7 actearly：减少一拍延迟

默认 actuator force 使用当前 activation `w_k`，本步积分得到的 `w_{k+1}` 在下一步影响力。因此 control 到 acceleration 之间带一个离散时序延迟。

`actearly="true"` 使用下一 activation 计算 actuator force，将影响提前一个 timestep。它改变系统离散阶次/相位，不只是性能开关。控制器设计、线性化和真机时序对齐时必须固定这个选择。

## 10.8 activation limit

`actlimited/actrange` 限制 activation，不是 control，也不是 force：

```text
ctrlrange → 输入命令
actrange  → actuator 内部状态
forcerange → 输出标量力
```

例如液压阀命令可在 `[-1,1]`，内部压力状态必须在 `[0,pmax]`，最终缸力还受面积与机械限制。三层饱和同时存在时，闭环会出现积分饱和和恢复延迟。

对 integrator actuator，actrange 尤其重要。若内部目标积到边界，上层 control 反向后需要先“卸载”累积量；应用控制器还可能需要 anti-windup。

## 10.9 独立实验：三种阶跃响应

```bash
cd examples/21_activation_filter
cmake -S . -B build
cmake --build build
./build/demo model.xml
```

模型包含三个相同 hinge：

1. stateless：无 activation，`ctrl=1` 立即产生 1 单位 actuator force；
2. filter：`t=0.05 s`，Euler 离散一阶滤波；
3. filterexact：同一 time constant，解析离散。

程序打印 `nactuator/nu/na`、每个 actuator 的 act address，并在 0、10、50、100、250 ms 记录 activation 和 force。5 ms timestep 下，50 ms 时 activation 应接近连续理论的 63.2%；filter 与 filterexact 有小的离散差异。

把 timestep 改成 60 ms，大于 time constant：普通 filter 会出现过冲/振荡甚至发散趋势，filterexact 仍单调稳定。这是非常直接的数值反例。

<!-- EMBEDDED_EXAMPLE_BEGIN: 21_activation_filter -->
### 可视化运行与效果

```bash
../../mujoco-3.11.0/bin/simulate model.xml
```

该命令使用发布包自带的官方 `simulate` 界面，可暂停、单步、施加扰动并开启接触等可视化标志。

![21_activation_filter 实验运行效果](../assets/experiments/21_activation_filter.png)

*21_activation_filter 的真实 MuJoCo 原生渲染结果。*

### 实验完整源码

以下文件与 `examples/21_activation_filter/` 中可直接编译的版本一致。

#### 模型文件：`model.xml`

```xml
<mujoco model="activation_filter">
  <compiler angle="radian"/>
  <option timestep="0.005" gravity="0 0 0"/>
  <default>
    <joint axis="0 1 0" damping=".1"/>
    <geom type="capsule" fromto="0 0 0 0 0 -.4" size=".03" mass="1" contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body pos="-.6 0 1"><joint name="stateless_joint"/><geom/></body>
    <body pos="0 0 1"><joint name="filter_joint"/><geom/></body>
    <body pos=".6 0 1"><joint name="exact_joint"/><geom/></body>
  </worldbody>
  <actuator>
    <general name="stateless" joint="stateless_joint" gainprm="1"/>
    <general name="filter" joint="filter_joint" dyntype="filter" dynprm=".05" gainprm="1"/>
    <general name="filterexact" joint="exact_joint" dyntype="filterexact" dynprm=".05" gainprm="1"/>
  </actuator>
</mujoco>
```

#### 程序源码：`main.cc`

```cpp
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "用法: %s model.xml\n", argv[0]);
    return EXIT_FAILURE;
  }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "无法加载 %s:\n%s\n", argv[1], error);
    return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m);
  std::printf("nactuator=%lld nu=%lld na=%lld\n",
              (long long)m->nactuator, (long long)m->nu, (long long)m->na);
  for (int i = 0; i < m->nactuator; ++i) {
    std::printf("%-11s actadr=%d actnum=%d\n",
                mj_id2name(m, mjOBJ_ACTUATOR, i),
                m->actuator_actadr[i], m->actuator_actnum[i]);
    d->ctrl[i] = 1.0;
  }

  const mjtNum sample[] = {0.0, 0.01, 0.05, 0.10, 0.25};
  std::printf("\n%7s %11s %11s %11s %11s %11s\n",
              "time", "force_none", "act_filter", "force_filter",
              "act_exact", "force_exact");
  for (int s = 0; s < 5; ++s) {
    while (d->time + 0.5*m->opt.timestep < sample[s]) mj_step(m, d);
    mj_forward(m, d);
    int af = m->actuator_actadr[1];
    int ae = m->actuator_actadr[2];
    std::printf("%7.3f %11.6f %11.6f %11.6f %11.6f %11.6f\n",
                d->time, d->actuator_force[0], d->act[af], d->actuator_force[1],
                d->act[ae], d->actuator_force[2]);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
```

#### 构建文件：`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(21_activation_filter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
add_executable(demo main.cc)

set(MUJOCO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../mujoco-3.11.0)
target_include_directories(demo PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(demo PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(demo PRIVATE mujoco)
set_target_properties(demo PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
```
<!-- EMBEDDED_EXAMPLE_END: 21_activation_filter -->

## 10.10 延迟不只有低通滤波

真实驱动链可能包含：

- 传感采样与时间戳延迟；
- 网络传输和队列；
- controller 计算时间；
- 零阶保持；
- 电流环/压力的一阶或高阶动态；
- 机械柔性、回差和摩擦；
- 命令速率限制。

一阶 filter 只表达平滑滞后，不等于纯时间延迟 `u(t-T)`。纯延迟更适合用历史 buffer/插件或应用层控制队列。控制验证应分别扫描 time constant 和 transport delay，因为它们的相位响应不同。

## 10.11 速度—力矩包络

理想 motor 在任意速度都能输出相同力矩，真实电机受电压和反电动势限制。简化 DC motor：

\[
\tau=k_t i,\qquad V=Ri+k_e\omega.
\]

在电压上限下：

\[
|\tau|\le \frac{k_t}{R}(V_{max}-k_e|\omega|),
\]

形成低速恒转矩、高速转矩下降的包络。可使用 dcmotor shortcut、gain/bias 参数、callback 或应用层限幅表达，具体取决于 3.11.0 功能和所需精度。

关节侧还要换算 gear、效率和电机速度。忽略速度包络会让仿真机器人在高速摆腿时拥有不现实的峰值力矩。

## 10.12 muscle 的三部分

MuJoCo muscle actuator 将 control 解释为神经激励，通过激活动力学得到 activation，再结合长度和速度生成力。典型结构：

\[
F=F_{max}\left[a\,F_L(L)F_V(V)+F_P(L)\right],
\]

其中：

- `F_L`：主动长度—张力曲线，最优长度附近最大；
- `F_V`：速度—张力曲线，缩短/伸长能力不同；
- `F_P`：被动长度张力，超过一定长度后增长；
- `a`：activation，受激活/去激活时间常数影响。

muscle 通过 tendon transmission 最自然：tendon 路径决定肌肉长度和 moment arm，肌肉力再映射到多个关节。

## 10.13 muscle length range 与自动计算

muscle 需要把物理 tendon length 归一化到工作区间。模型编译器可根据 joint range 和 actuator lengthrange 计算可达长度范围，但这一优化可能昂贵，也依赖 joint limit 和初态。

应检查：

- `actuator_lengthrange` 是否覆盖实际运动；
- 最优长度和 slack/工作范围是否合理；
- 在关键姿态 moment arm 是否接近零或变号；
- 最大等长力是否与生理/执行器数据一致；
- 被动力是否在初始姿态已异常巨大。

不合理 lengthrange 会导致 muscle 总处在曲线极端，控制再好也无法产生期望力。

## 10.14 液压、气动和柔性驱动

`cylinder` 等 shortcut 可表达与长度/速度相关的力生成，但真实液压系统还包含压力状态、阀流量、可压缩性和供压限制。教学路线：

1. 先用 motor 验证机械系统；
2. 加 filterexact 表达主要响应时间；
3. 加 force/activation range；
4. 用 gain/bias 表达长度相关机械优势；
5. 需要压力守恒、多腔耦合时使用 plugin 或外部联合仿真。

不要一开始就拟合所有非线性。先用实验说明哪些动态对任务性能重要。

## 10.15 状态保存、reset 与 keyframe

有状态 actuator 使以下操作必须包含 `act`：

- rollout 初态保存；
- episode reset；
- keyframe；
- 动力学线性化；
- 两次控制器公平对比。

只恢复 qpos/qvel、保留旧 activation，会让相同机械姿态产生不同初始力。`mjSTATE_PHYSICS` 包含相应物理状态，具体 3.11.0 state bitmask 仍应查头文件。

keyframe 的 act 长度是 `na`，不是 `nu`。无状态和有状态 actuator 混合时尤其不能按 actuator ID 直接索引 act。

## 10.16 线性化中的 actuator dynamics

无状态机械系统线性化状态通常是 `δq,δv`；有 activation 后必须加入 `δw`：

\[
\begin{bmatrix}
\delta x_{k+1}\\
\end{bmatrix}
=A
\begin{bmatrix}
\delta q_k\\\delta v_k\\\delta w_k
\end{bmatrix}
+B\delta u_k.
\]

忽略 `w` 得到的 LQR 可能假设力瞬时到达，实际闭环相位裕度不足。后续 LQR 章会显式比较平衡 control、activation 和 actuator force。

## 10.17 常见错误

| 错误 | 后果 | 修复 |
|---|---|---|
| 把 act 当 ctrl 的副本 | 状态恢复和线性化错误 | 按 dynamics 积分理解 |
| 假定 act[i] 对 actuator i | 混合无状态 actuator 时错位 | actuator_actadr/actnum |
| filter time constant 小于 timestep | Euler filter 不稳定 | filterexact 或减小 h |
| 降低 kp 模拟纯延迟 | 幅频/相频性质错误 | 单独建 transport/filter delay |
| 只限 ctrl 不限 activation | integrator windup | actrange + anti-windup |
| muscle 只设最大力 | 长度/速度曲线不合理 | 验证 lengthrange/moment arm |
| reset 只恢复 qpos/qvel | 初始 actuator force 不一致 | 恢复完整物理状态 |

## 10.18 本章小结

- activation 是真实动力学状态，使系统成为三阶。
- integrator、filter、filterexact 定义不同 control→activation 动态。
- filterexact 对正 time constant 无条件稳定，更适合连续一阶响应。
- `actearly` 改变 control 到 force 的离散时序。
- ctrlrange、actrange、forcerange 分别限制输入、内部状态和输出。
- muscle force 同时依赖 activation、长度和速度，并通过 tendon moment 映射。
- rollout、reset 和 LQR 必须把 act 纳入状态。

## 10.19 练习

1. filterexact 的 `t=0.1 s`，从 0 接受单位阶跃，0.2 s 时 activation 理论值是多少？
2. 普通 filter 的 `h=0.01,t=0.005`，离散极点是多少，为什么不稳定？
3. 模型有 5 个 actuator，其中第 2、4 个有一个 activation，`nu=5` 时 `na` 是多少？能否用 `act[3]` 表示第 4 actuator？
4. 为什么纯 transport delay 和一阶低通不能互相完全替代？
5. muscle 在某姿态 tendon moment arm 为零，即使 activation=1，会对该 joint 产生力矩吗？

## 10.20 参考答案

1. `1-exp(-0.2/0.1)=1-exp(-2)≈0.8647`。
2. Euler 极点 `1-h/t=1-2=-1`，处在稳定边界且会符号振荡；更小 t 时绝对值超过 1 而发散。
3. `na=2`；不能按 actuator ID 访问，必须读取第 4 actuator 的 `actuator_actadr`，它很可能是 1。
4. 纯延迟幅值不变而相位随频率线性下降；一阶低通同时衰减高频幅值且相位渐变，时域阶跃形状也不同。
5. 对该 joint 的广义力为 moment arm×muscle force，moment arm 为零时不产生该 joint 力矩，尽管可能作用于其他 joint。
