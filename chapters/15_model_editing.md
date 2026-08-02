# 第 15 章　质量矩阵、偏置力与关节空间动力学

机器人动力学的核心不是“重力加速度乘质量”，而是构型相关的质量矩阵、速度耦合、重力、被动力和约束共同决定广义加速度。本章从标准方程进入 MuJoCo 数组，验证质量矩阵的对称正定性和动能意义。

## 15.1 学习目标

- 解释 `M(q)`、`c(q,v)` 和各种 `qfrc_*`；
- 理解质量矩阵为何构型相关、对称、正定；
- 正确使用 `mj_fullM`、`mj_mulM`、`mj_solveM`；
- 用 `½vᵀMv` 验证动能；
- 区分 bias、passive、actuator、applied 和 constraint force；
- 理解 armature 如何改变质量矩阵。

## 15.2 连续动力学方程

MuJoCo 采用广义坐标动力学：

\[
M(q)\dot v+c(q,v)=\tau+J(q)^Tf.
\]

其中：

- `q∈Q`：配置，长度 nq；
- `v,\dot v∈R^{nv}`：广义速度/加速度；
- `M(q)∈R^{nv×nv}`：质量矩阵；
- `c(q,v)`：偏置力，包含重力、科氏、离心等；
- `τ`：执行器、用户外力和被动力等非约束广义力；
- `Jᵀf`：contact、limit、equality、frictionloss 等约束力。

```mermaid
flowchart LR
  Q[q] --> M[M(q)]
  Q --> C[c(q,v)]
  V[v] --> C
  U[actuator/applied/passive] --> R[右端广义力]
  F[constraint force] --> R
  M --> A[qacc]
  C --> A
  R --> A
```

## 15.3 质量矩阵的物理意义

对任意广义速度 v，系统动能：

\[
T=\frac12v^TM(q)v.
\]

对物理合法、无质量退化的系统：

\[
M=M^T,\qquad v^TMv>0\quad(v\ne0).
\]

对角元素不是简单“对应 link 质量”，而是沿该 DOF 运动时整个受影响子树的有效惯量。非对角元素表达关节速度耦合：两个关节同时运动时，动能包含交叉项。

## 15.4 为什么 M 随 q 变化

二连杆机械臂屈伸时，第二 link 质心相对肩关节的距离和速度方向改变，因此 shoulder 感受到的有效惯量及 shoulder-elbow 耦合改变。

典型平面二连杆质量矩阵含：

\[
M_{11}=a+2b\cos q_2,
\quad M_{12}=d+b\cos q_2,
\quad M_{22}=d.
\]

即使 link 质量不变，`cos q₂` 让 M 随构型变化。这也是固定 PD 增益在不同姿态阻尼比不同的原因。

## 15.5 MuJoCo 如何存储 M

`d->qM` 是利用运动学树稀疏结构的压缩表示，长度不是 `nv*nv`。禁止：

```cpp
// 错误：qM 不是普通 dense matrix
mjtNum value = d->qM[row*m->nv + col];
```

MuJoCo 3.11.0 展开完整矩阵：

```cpp
std::vector<mjtNum> dense(m->nv*m->nv);
mj_fullM(m, d, dense.data());
```

旧版本常见签名可能不同，教材代码以仓库内 3.11.0 头文件为准。

## 15.6 不展开矩阵的乘法和求解

高频算法优先：

```cpp
mj_mulM(m, d, result, vector);       // result = M * vector
mj_solveM(m, d, solution, rhs, 1);   // solution = M^{-1} * rhs
```

`mj_solveM` 使用 forward pipeline 已计算的质量矩阵分解。调用前必须确保 qpos 对应的动力学已更新，例如执行 `mj_forward`。

多右端项的布局和参数 `n` 应查 API Reference。不要为了求 `M⁻¹y` 显式构造逆矩阵：求解线性系统更快、更稳定。

## 15.7 armature 对 M 的影响

joint/tendon armature 为相应广义/传动速度增加惯性能量。对简单 joint DOF，它直接增加质量矩阵对角项：

\[
M_{ii}\leftarrow M_{ii}+a_i.
\]

armature 可改善极轻 link 的数值条件，也表达反射转子惯量；但不能为了让 solver 好看随意增大，否则加速度、自然频率和控制带宽失真。

比较模型前后 M 的对角元素，是验证 armature 是否生效的直接方法。

## 15.8 qfrc_bias

`d->qfrc_bias` 对应标准方程左侧除 `M qacc` 外的偏置项，主要含重力、科氏和离心作用。常写：

\[
c(q,v)=C(q,v)v+g(q).
\]

但 `C` 矩阵表示不唯一，MuJoCo 直接提供组合后的向量更实用。

- 当 `v=0`，qfrc_bias 主要是保持当前姿态需克服的重力项（符号需按方程核对）；
- 当 gravity=0、v非零，剩下速度相关偏置；
- free base 上 bias 包含全身惯性/重力对基座 DOF 的作用。

## 15.9 广义力账本

| 字段 | 来源 |
|---|---|
| `qfrc_actuator` | actuator force 经 moment 映射 |
| `qfrc_passive` | spring、damping、部分流体/插件被动力 |
| `qfrc_applied` | 用户直接写入的广义力 |
| `xfrc_applied` | body 笛卡尔外力，经内部映射 |
| `qfrc_bias` | 重力、科氏、离心等左侧偏置 |
| `qfrc_constraint` | 约束 solver 广义力 |
| `qfrc_inverse` | inverse dynamics 所需净广义力结果 |

调试应建立力平衡残差，而不是只看某一项。具体 forward dynamics 的符号组合以 Computation 文档和 `mj_compareFwdInv` 等诊断为准。

## 15.10 独立实验：M 的四重验证

```bash
cd examples/25_mass_matrix_properties
cmake -S . -B build
cmake --build build
./build/demo model.xml
```

二连杆设置非零 q 和 v，程序验证：

1. `mj_fullM` 展开的 M 对称；
2. 二阶主子式为正，2×2 M 正定；
3. `0.5*vᵀMv` 与 `mj_energyVel` 得到的 kinetic energy 一致；
4. 随机 rhs 先 `mj_solveM` 再 `mj_mulM`，残差接近机器精度。

若这四项失败，应先检查 API 版本/布局和 forward 时序，再怀疑物理引擎。

## 15.11 正定性与数值条件

正定不代表条件良好。若最大/最小特征值比值很大：

\[
\kappa(M)=\frac{\lambda_{max}}{\lambda_{min}},
\]

求解会放大浮点和模型误差。来源包括：

- 极端质量比；
- 接近无质量 link；
- 错误单位导致惯量跨很多数量级；
- 冗余坐标/不合理 armature；
- 非常细长且惯量近退化的 body。

MuJoCo 编译器会拒绝某些非法惯量，但工程仍需检查尺度。条件数诊断可使用外部线性代数库；教学 2×2 模型可由解析特征值计算。

## 15.12 重力补偿

固定基座机械臂静止时设 `v=0,qacc=0`，逆动力学给出保持姿态所需广义力。可近似用 qfrc_bias 或 `mj_inverse` 结果构造前馈：

\[
\tau=g(q)+K_pe+K_d\dot e.
\]

但 actuator ctrl 与广义力不是总相等。需要通过 actuator moment/gear 求能实现目标 τ 的 control；欠驱动系统可能无解，多 actuator 系统可能有无穷解。

## 15.13 科氏与离心作用的对照实验

固定 q，分别设置：

1. v=0；
2. v；
3. -v。

读取 qfrc_bias。重力项不随 v 变；许多速度二次项对整体速度反号保持不变，但不同耦合项的结构需结合方程分析。再设置 gravity=0 可隔离速度偏置。

不要尝试从少量样本唯一恢复 C 矩阵，因为 `C(q,v)v` 的矩阵 C 表示本身不唯一。控制通常只需 bias 向量。

## 15.14 浮动基座质量矩阵

free base 的前 6 个速度 DOF与全身关节耦合，M 可分块：

\[
M=
\begin{bmatrix}
M_{bb}&M_{bj}\\
M_{jb}&M_{jj}
\end{bmatrix}.
\]

关节运动会通过 `M_bj` 影响基座动量；这正是空中姿态调整和人形 whole-body dynamics 的基础。固定基座模型删除了这部分耦合，不能用于验证浮动基座动量控制。

基座 qpos 有 7 个数，但 M 的基座块是 6×6，因为 M 位于速度/力的 nv 空间。

## 15.15 质量矩阵与任务空间惯量

在满秩条件下，末端任务空间惯量：

\[
\Lambda=(JM^{-1}J^T)^{-1}.
\]

它描述某方向末端力与加速度的关系。奇异附近需要阻尼/子空间处理。直接用 `(JJ^T)⁻¹` 忽略 M，只反映运动学而非动态惯量。

操作空间控制会进一步构造动态一致伪逆和 nullspace projector，后续控制篇详细展开。

## 15.16 稀疏性和性能

树形多体系统的 M 虽以 dense 数学矩阵描述，但其分解和相关运算可利用树稀疏结构。每个控制周期展开 dense M 再用通用逆矩阵是常见性能浪费。

建议：

- 只需 `Mv`：`mj_mulM`；
- 只需 `M⁻¹y`：`mj_solveM`；
- 教学、日志或小系统分析才 `mj_fullM`；
- 多 RHS 复用当前 q 下的分解；
- q 改变后重新 forward，不能复用旧 M。

## 15.17 常见错误

| 错误 | 后果 | 修复 |
|---|---|---|
| 把 qM 按 nv² 索引 | 越界/错误矩阵 | mj_fullM |
| 显式计算 M inverse | 慢且数值差 | mj_solveM |
| qpos 改后直接读 M | 陈旧惯量 | mj_forward |
| M 使用 nq 维度 | quaternion 模型错维 | M 是 nv×nv |
| qfrc_bias 当纯重力 | 高速时补偿错误 | 它还含速度偏置 |
| ctrl 直接等于 τ | 传动/饱和错误 | actuator moment mapping |
| 只检查 M 对称 | 忽略条件数和物理尺度 | 正定/条件/实验审计 |

## 15.18 本章小结

- `M(q)` 把广义加速度与惯性力联系起来，位于 nv 空间。
- M 对称正定，动能为 `½vᵀMv`，非对角项表达耦合。
- qM 是压缩存储；使用 fullM、mulM、solveM 公共 API。
- qfrc_bias 组合重力、科氏和离心作用。
- 广义力应按 actuator/passive/applied/constraint 分账诊断。
- 浮动基座 M 的基座—关节耦合是全身动力学核心。

## 15.19 练习

1. 为什么 M 是 nv×nv，而不是 nq×nq？
2. 对 `M=[[2,0.5],[0.5,1]]`、`v=[1,2]`，动能是多少？
3. 给 shoulder DOF 增加 armature 0.2，M 哪个元素直接变化？
4. 为什么 `qfrc_bias(q,0)` 可用于重力补偿，但高速运动时只用它的静态缓存不正确？
5. `mj_solveM` 后残差大，列出三个首先检查的事项。

## 15.20 参考答案

1. M 映射切空间加速度到广义力；quaternion 配置有冗余分量，独立速度维度是 nv。
2. `Mv=[3,2.5]`，`T=0.5*(1*3+2*2.5)=4`。
3. 对应 shoulder dof 的对角元素 `M_ii` 增加 0.2。
4. qfrc_bias 随 q 和 v 变化，高速时含科氏/离心项；静态缓存既姿态旧又缺少当前速度效应。
5. 是否先对当前 q 调 forward；rhs/solution 长度和多 RHS 布局；是否使用同一个 model/data；还应检查模型惯量条件和 NaN。

