# 第 26 章　LQR：从平衡点到局部最优反馈

> 本书示例代码仓库：[libing403/mujoco_tutorial](https://github.com/libing403/mujoco_tutorial)

线性二次调节器（LQR）不是“自动选 PD 增益”的黑盒。它在一个自洽平衡点附近，利用离散线性模型和无限时域二次代价，得到局部最优状态反馈。本章沿官方 LQR notebook 的推理链展开，并在 C++ 中直接迭代离散 Riccati 方程。

## 26.1 学习目标

- 判断一个 state-control pair 是否真的是平衡点；
- 用逆动力学与 actuator moment arms 求控制前馈；
- 理解 controllability、Q/R 权重和离散 Riccati 方程；
- 在 MuJoCo 切空间中构造 LQR state error；
- 区分局部线性稳定、饱和后的非线性表现和接触 mode 有效域。

## 26.2 离散 LQR 问题

局部离散系统

\[
\delta x_{k+1}=A\delta x_k+B\delta u_k
\]

与无限时域代价

\[
J=\sum_{k=0}^{\infty}
(\delta x_k^TQ\delta x_k+\delta u_k^TR\delta u_k),
\]

其中 \(Q\succeq0,R\succ0\)。最优策略是

\[
\delta u_k=-K\delta x_k,
\]

最优 value function 为 \(V(x)=x^TPx\)。矩阵 \(P\) 满足离散代数 Riccati 方程（DARE）：

\[
P=Q+A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA,
\]

并有

\[
K=(R+B^TPB)^{-1}B^TPA.
\]

工程实现不应显式求逆大矩阵，而应用对称分解求解。教学示例是单输入 2-state，分母退化为标量，公式可完整写在一个源码文件中。

## 26.3 第一步不是 Riccati，而是平衡点

希望控制在 \((\bar x,\bar u)\) 附近调节，必须满足

\[
f(\bar x,\bar u)=\bar x.
\]

固定基机械臂的静态姿态可设 `qvel=qacc=0` 调用 `mj_inverse` 得到所需广义力，再映射到 actuator controls。unit motor 时近似直接对应；一般 actuator 应使用 moment-arm matrix 的伪逆或有限差分 \(\partial qfrc_{actuator}/\partial ctrl\)。

浮动基接触系统还要检查 root DoF 的 inverse force。官方 humanoid 示例中，足部接触高度偏差不到毫米就会要求巨大的竖直“魔法力”。只有调整 pose/contact，使未驱动 root force 接近零，内部关节力才是可实现的控制前馈。

最终控制律是

\[
u=\bar u-K(x\ominus\bar x),
\]

遗漏 \(\bar u\) 会让反馈长期承担重力或接触静载，工作点也不再是线性化点。

## 26.4 可控性

有限维线性系统的 controllability matrix：

\[
\mathcal C=[B,AB,A^2B,\ldots,A^{n_x-1}B].
\]

满行秩表示任意小状态可由有限控制序列到达。浮动基人形通常不是裸系统完全驱动，但接触约束会改变局部有效动力学。数值上不要仅用精确 rank；应看奇异值尺度，接近不可控的方向会需要巨大 gain 并迅速触发 saturation。

若某个不稳定 mode 不可控，DARE 可能无稳定解。增加 Q 权重不能创造 control authority。

## 26.5 Q 与 R 的工程含义

Q/R 的共同缩放不改变 K，因此真正重要的是相对权重。常见设计：

- configuration tangent error 权重按允许角度/位移的倒数平方；
- velocity 权重按允许速度的倒数平方；
- control 权重按允许力矩/输入的倒数平方；
- 对 humanoid 加入 CoM 相对支撑脚位置代价，可由 CoM/site Jacobian 映射到 state quadratic form。

Bryson rule 提供初值：若 \(|x_i|\le x_{i,max}\)，取 \(Q_{ii}=1/x_{i,max}^2\)；control 同理。但它只是无量纲化，不替代任务权衡。

官方单脚站立示例对支撑腿与躯干姿态给较高权重，对手臂等可用于平衡的 DoF 给较低权重；这体现“允许哪些关节移动来保护主要任务”。

## 26.6 切空间状态误差

```cpp
mj_differentiatePos(m, dx, 1.0, qpos_ref, d->qpos);
// dx[0:nv] 是从 reference 到 current 的 position tangent
// dx[nv:2*nv] = qvel - qvel_ref
```

注意函数参数顺序和符号必须用一个 hinge 正方向实验校准。free joint 的四元数差自然得到三维旋转向量；直接减 `qpos` 会使 state 维数错误并遇到四元数双覆盖。

若 actuator 有 activation，`act-act_ref` 追加在 state 尾部。估计器、延迟队列或控制滤波状态若影响闭环，也应纳入 augmented system 后重新线性化/设计。

## 26.7 求 DARE

一种教学用 fixed-point iteration：从 \(P_0=Q\) 开始反复代入 Riccati 右端，直到矩阵变化足够小。稳定/可稳定系统通常收敛，但正式项目应使用成熟 Schur/QZ DARE solver，检查对称性、残差和 closed-loop eigenvalues。

验证至少包括：

1. Riccati residual 范数；
2. \(A-BK\) 所有离散特征值在单位圆内；
3. 无饱和线性 rollout 收敛；
4. 非线性 MuJoCo 小扰动收敛；
5. 扫描初始扰动和 saturation，估计实际 region of attraction。

## 26.8 独立实验：倒立摆 LQR

`examples/36_lqr_balance/` 在倒立平衡点调用 `mjd_transitionFD`，迭代 2×2 DARE，比较无控制与 LQR 的三秒响应。

```bash
cd examples/36_lqr_balance
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

模型在精确直立处 `ū=0`，所以省略了前馈映射。把 geom 改成偏心或加入弹簧后，应重新用逆动力学确认工作点，而不是沿用零 control 假设。

<!-- EMBEDDED_EXAMPLE_BEGIN: 36_lqr_balance -->
### 可视化运行与效果

```bash
../../mujoco-3.11.0/bin/simulate model.xml
```

该命令使用发布包自带的官方 `simulate` 界面，可暂停、单步、施加扰动并开启接触等可视化标志。

![36_lqr_balance 实验运行效果](../assets/experiments/36_lqr_balance.png)

*36_lqr_balance 的真实 MuJoCo 原生渲染结果。*

### 实验完整源码

以下文件与 `examples/36_lqr_balance/` 中可直接编译的版本一致。

#### 模型文件：`model.xml`

```xml
<mujoco model="LQR inverted pendulum">
  <option timestep="0.005" integrator="implicitfast"/>
  <worldbody>
    <body>
      <joint name="hinge" axis="0 1 0" damping=".02" armature=".01"/>
      <geom type="capsule" fromto="0 0 0 0 0 .6" size=".035" mass="1"/>
    </body>
  </worldbody>
  <actuator><motor joint="hinge" ctrlrange="-10 10" ctrllimited="true"/></actuator>
</mujoco>
```

#### 程序源码：`main.cc`

```cpp
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

void dare(const double A[4], const double B[2], double K[2]) {
  const double Q[4] = {20, 0, 0, 1}, R = 0.1;
  double P[4] = {20, 0, 0, 1};
  for (int iter = 0; iter < 100000; ++iter) {
    double PB[2] = {P[0]*B[0]+P[1]*B[1], P[2]*B[0]+P[3]*B[1]};
    double s = R+B[0]*PB[0]+B[1]*PB[1];
    double PA[4] = {P[0]*A[0]+P[1]*A[2], P[0]*A[1]+P[1]*A[3],
                    P[2]*A[0]+P[3]*A[2], P[2]*A[1]+P[3]*A[3]};
    double btpa[2] = {B[0]*PA[0]+B[1]*PA[2], B[0]*PA[1]+B[1]*PA[3]};
    double next[4];
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) {
      double atpa = A[0*2+i]*PA[0*2+j] + A[1*2+i]*PA[1*2+j];
      next[2*i+j] = Q[2*i+j] + atpa - btpa[i]*btpa[j]/s;
    }
    double change = 0;
    for (int i = 0; i < 4; ++i) { change = mju_max(change, std::fabs(next[i]-P[i])); P[i]=next[i]; }
    if (change < 1e-12) break;
  }
  double PB[2] = {P[0]*B[0]+P[1]*B[1], P[2]*B[0]+P[3]*B[1]};
  double s = R+B[0]*PB[0]+B[1]*PB[1];
  K[0] = (B[0]*(P[0]*A[0]+P[1]*A[2]) + B[1]*(P[2]*A[0]+P[3]*A[2]))/s;
  K[1] = (B[0]*(P[0]*A[1]+P[1]*A[3]) + B[1]*(P[2]*A[1]+P[3]*A[3]))/s;
}

double simulate(const mjModel* m, const double K[2], bool controlled) {
  mjData* d = mj_makeData(m); d->qpos[0] = 0.15; mj_forward(m, d);
  for (int k = 0; k < 600; ++k) {
    if (controlled) d->ctrl[0] = mju_clip(-K[0]*d->qpos[0]-K[1]*d->qvel[0], -10.0, 10.0);
    mj_step(m, d);
  }
  double angle = d->qpos[0]; mj_deleteData(d); return angle;
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0}; mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* d = mj_makeData(m); mj_forward(m, d);
  mjtNum A[4], B[2]; mjd_transitionFD(m, d, 1e-6, 1, A, B, NULL, NULL);
  double K[2]; dare(A, B, K);
  std::printf("K = [%.6f, %.6f]\n", K[0], K[1]);
  std::printf("after 3 s: uncontrolled angle = %.6f rad\n", simulate(m, K, false));
  std::printf("after 3 s: LQR angle          = %.9f rad\n", simulate(m, K, true));
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
```

#### 构建文件：`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(36_lqr_balance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
add_executable(demo main.cc)

set(MUJOCO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../mujoco-3.11.0)
target_include_directories(demo PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(demo PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(demo PRIVATE mujoco)
set_target_properties(demo PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
```
<!-- EMBEDDED_EXAMPLE_END: 36_lqr_balance -->

## 26.9 非线性与接触边界

LQR 的 optimal 只针对线性模型、二次代价、无 hard saturation 的设定。实际中：

- 大角度下 A/B 改变，固定 K 可能失效；
- command clamp 破坏 Riccati 推导；
- 足部离地或滑移后接触 mode 改变；
- sensor delay、estimator dynamics 降低稳定裕量；
- model mismatch 改变闭环 pole。

可以用 gain scheduling、time-varying LQR、MPC 或非线性策略扩大工作范围。即便如此，LQR 仍是极有价值的局部基线、terminal controller 与线性化正确性检查。

## 26.10 常见误区

- 在非平衡 state-control pair 上做 regulation，却不处理 affine drift；
- 遗漏重力/接触前馈 `ū`；
- 对 `qpos` 直接相减构造浮动基 state；
- 混用连续 ARE 和离散 A/B；
- Q/R 未按单位尺度归一，某类状态意外支配代价；
- 只看 DARE 返回成功，不检查 closed-loop eigenvalues；
- 仿真中 control 无限，部署时突然加入 saturation；
- 单个小扰动站住就宣称人形稳定域足够大。

## 26.11 习题与答案

1. 同时把 Q、R 乘 100，K 是否变化？  
   **答案：**理论上不变，P 同比例缩放；数值舍入可能有极小差异。

2. 为什么工作点需要 control 前馈？  
   **答案：**反馈在零误差时为零，维持重力/静载的非零输入必须由 `ū` 提供。

3. 离散 closed-loop 稳定条件是什么？  
   **答案：**`A-BK` 的全部特征值模小于 1。

4. actuator saturation 后 LQR 还最优吗？  
   **答案：**不是；标准推导假设 unconstrained control，饱和还可能缩小稳定域。

5. 人形 root inverse force 很大说明什么？  
   **答案：**所谓静态 pose/contact 不自洽，需要不可驱动外力维持；应先调整高度/姿态/接触或求可行平衡。
