# 第 25 章　离散动力学线性化与有限差分

LQR、EKF、局部 MPC 和 trajectory optimization 都需要动力学导数。MuJoCo 的真实一步是非线性映射

\[
x_{k+1}=f(x_k,u_k),

\]

`mjd_transitionFD` 高效有限差分整个 `mj_step`，直接给出离散时间 Jacobian。本章解释这些矩阵究竟对什么 state 求导，以及怎样证明它们可用。

## 25.1 学习目标

- 区分连续时间动力学 Jacobian 与离散一步 transition Jacobian；
- 理解 MuJoCo 切空间 state 的维数 `2*nv+na`；
- 使用 `mjd_transitionFD` 获取 A、B、sensor C、D；
- 选择 epsilon、中心/前向差分和 solver 设置；
- 用独立扰动的一步预测误差验证导数。

## 25.2 局部模型

在工作点 \((\bar x,\bar u)\) 附近，定义扰动

\[
\delta x=x\ominus\bar x,
\qquad
\delta u=u-\bar u.
\]

一阶离散模型为

\[
\delta x_{k+1}=A\delta x_k+B\delta u_k+O(\|\delta\|^2),
\]

其中

\[
A=\left.\frac{\partial f}{\partial x}\right|_{\bar x,\bar u},
\qquad
B=\left.\frac{\partial f}{\partial u}\right|_{\bar x,\bar u}.
\]

它们已经包含一个 physics timestep 的积分效果。不要再把连续模型用 `I+hA` 离散一次，否则会重复离散化。

## 25.3 state 为什么不是 `nq+nv`

MuJoCo transition state 的切空间维数为

\[
n_x=2n_v+n_a.
\]

顺序是 position tangent、velocity、activation。ball/free joint 的 quaternion configuration 用三维旋转扰动表示，所以 position tangent 是 `nv` 而不是 `nq`。比较状态必须用 `mj_differentiatePos`；施加 position perturbation 用 `mj_integratePos`。

时间、applied force、mocap、user data、控制器内部状态等不自动进入这个线性 state。若它们影响一步映射，应用必须扩充自己的状态或确保它们固定。

## 25.4 API 输出

```cpp
int nx = 2*m->nv + m->na;
std::vector<mjtNum> A(nx*nx), B(nx*m->nu);
mjd_transitionFD(m, d, 1e-6, 1, A.data(), B.data(), NULL, NULL);
```

矩阵采用 row-major：`A[row*nx+col]`、`B[row*nu+col]`。可选 sensor matrices：

\[
C=\partial s/\partial x\in\mathbb R^{n_s\times n_x},
\qquad
D=\partial s/\partial u\in\mathbb R^{n_s\times n_u}.
\]

它们是下一步 sensor output 对当前 state/control 的导数，与控制教材常把 `y_k` 定义为当前瞬时观测的 convention 可能不同，使用前要按 API 的一步语义核对。

## 25.5 工作点必须自洽

线性化前，`d->qpos/qvel/act/ctrl` 应是目标工作点，并调用 `mj_forward` 填充派生量。平衡控制还需要 \(f(\bar x,\bar u)=\bar x\)：

- 固定基机械臂可用逆动力学求重力前馈；
- 浮动基人形必须先找到与接触一致的高度和 actuator setpoint；
- 若 root inverse force 很大，说明所谓“平衡点”依靠不可驱动的魔法外力维持。

官方 LQR notebook 正是先调节单脚站立高度，使 inverse dynamics 的 root force 接近零，再将内部广义力映射到 actuator control。

## 25.6 数值精度

前向差分每列多一次计算；中心差分每列两次，截断误差更小但成本更高。epsilon 过大破坏局部性，过小则放大舍入和 solver 终止噪声。

约束模型建议：

1. 先 `mj_forward` 让 warmstart 收敛；
2. 暂时降低 Newton iterations，因为扰动点接近已收敛解；
3. 将 tolerance 设为 0 固定迭代路径，避免提前终止造成列间噪声；
4. 线性化后恢复 option；
5. 扫描多个 epsilon，检查 A/B 和预测误差的平台区间。

`mjd_transitionFD` 不支持 RK4。控制建模时应选受支持积分器，并保证训练/验证采用相同 timestep 和 integrator。

## 25.7 接触处的“导数”

固定接触模式内部通常可获得有用局部导数；接触建立/断开、stick/slip 切换处则不光滑。有限差分的正负扰动可能落在不同 mode，得到的矩阵强烈依赖 epsilon。

这不代表导数毫无价值：站立 LQR 可在稳定承载的接触 mode 内工作。但它是局部控制器，足部即将离地或滑移时模型失效，需要 gain scheduling、MPC、接触切换逻辑或非线性策略。

## 25.8 独立实验：一步预测验证

`examples/35_transition_fd/` 在线性化点获取 A、B，然后施加一个未用于差分验证的 state/control perturbation。程序比较真实下一步状态差和 `A dx+B du`。

```bash
cd examples/35_transition_fd
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

把验证扰动从 `1e-4` 逐次缩小，若实现正确，一阶模型误差应近似二次下降，直到浮点与有限差分噪声占主导。

## 25.9 常见误区

- 把 A/B 当连续时间矩阵又离散一次；
- 以 `nq+nv+na` 分配 A，浮动基模型维数错误；
- 直接相减两个 quaternion；
- 线性化前手工改 qpos 却未 forward；
- 工作点并非平衡点，却直接求无限时域 regulation LQR；
- epsilon 只试一个值；
- 在接触切换边界期待平滑 Jacobian；
- control callback 带随机数或隐藏历史，使有限差分不再测量同一个函数。

## 25.10 习题与答案

1. `nq=7,nv=6,na=2` 时 transition state 多大？  
   **答案：**`2*6+2=14`，不是 15。

2. timestep 改变后 A、B 能否复用？  
   **答案：**不能；它们是特定积分器和 timestep 的离散一步导数。

3. 怎样检验 B 的某一列？  
   **答案：**从同一 state 分别用 `u` 和 `u+eps e_j` 步进，比较下一状态切空间差与 `B[:,j]*eps`。

4. 为什么 tolerance=0 有助有限差分？  
   **答案：**固定 solver 迭代数，减少不同扰动列因提前终止路径不同产生的数值噪声。

5. 一步预测准确是否保证长时线性 rollout 准确？  
   **答案：**不保证；非线性余项会累计，状态离开工作点后 A/B 失效。
