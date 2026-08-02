# 第 30 章　MJX、策略梯度与 APG

官方 `training_apg.ipynb` 展示了 MJX 可微物理怎样用于 analytical/first-order policy gradients（APG/FoPG）。本章先在原生 C++ 小系统中手工写出 through-time sensitivity recursion，使链式法则可见；再解释 MJX/JAX 如何把同一计算图批量化、自动微分并放到 accelerator。

## 30.1 学习目标

- 区分 zeroth-order 与 first-order policy gradient；
- 推导动力学、policy 和 reward 通过时间展开的梯度；
- 理解接触 softness、horizon、梯度爆炸/消失和截断；
- 判断 CPU rollout、MJX/JAX 与 MuJoCo Warp 的适用边界；
- 解释 APG 的 sample efficiency、探索局限和 residual learning。

## 30.2 策略学习问题

策略 \(a_t=\pi(x_t;\theta)\)，动力学 \(x_{t+1}=f(x_t,a_t)\)，希望最大化

\[
J(\theta)=\mathbb E\left[\sum_{t=0}^{T-1}r(x_t,a_t)\right].
\]

也可最小化 cost \(L=-J\)。策略参数可以只是 PD gains，也可以是神经网络数百万权重。关键差异不是参数规模，而是如何估计 \(\partial J/\partial\theta\)。

## 30.3 Zeroth-order policy gradient

ZoPG 只调用 simulator 得到函数值，不需要 \(\partial f/\partial x\)。有限扰动的 antithetic estimator 是最直观例子：

\[
\nabla_\theta J\approx
\frac1N\sum_i
\frac{J(\theta+\sigma\epsilon_i)-J(\theta-\sigma\epsilon_i)}{2\sigma}\epsilon_i.
\]

PPO 等 stochastic policy 方法使用 \(\nabla\log\pi\) 与 sampled returns。它们能处理不可微、随机环境并有较强探索，但 gradient variance 高，需要大量 rollout。第 29 章的 CPU batch 是此类方法的基础设施。

## 30.4 First-order / analytical policy gradient

FoPG 假设确定性可微 transition。定义 state sensitivity

\[
S_t=\frac{\partial x_t}{\partial\theta}.
\]

policy 对参数的总导数：

\[
U_t=\frac{\partial a_t}{\partial\theta}
=\frac{\partial\pi}{\partial x}S_t+rac{\partial\pi}{\partial\theta}.
\]

动力学递推：

\[
S_{t+1}=A_tS_t+B_tU_t,
\quad A_t=\frac{\partial f}{\partial x},
\quad B_t=\frac{\partial f}{\partial a}.
\]

累计 cost gradient：

\[
\frac{\partial L}{\partial\theta}
=\sum_t\left(
\frac{\partial l_t}{\partial x}S_t+
\frac{\partial l_t}{\partial a}U_t
\right).
\]

这就是 forward sensitivity 形式。JAX 通常用 reverse-mode backpropagation through time（BPTT），对参数很多、scalar objective 更高效。官方图中的多条颜色路径，本质都是上述 chain rule 展开的不同依赖路径；随意 `stop_gradient` 会删除真实的控制影响。

## 30.5 接触梯度为何困难

硬接触下，落地前一瞬对高度的小扰动可能让 trajectory 一条碰撞、一条未碰撞，导数不连续或没有提供“怎样提前减速”的信息。MuJoCo 默认 soft contact 使穿入深度与接触力连续变化，常能提供更有用的局部梯度。

但更软并非永远更真实：学习使用的 softness、timestep 和 solver 会塑造 gradient。策略可能利用过软地面、深穿入或非真实滑移。训练后应在更真实参数分布、CPU MuJoCo 和扰动条件下验证。

## 30.6 Through-time 的 sharp bits

递推反复乘闭环 Jacobian。若谱半径长期大于 1，gradient 可能指数爆炸；小于 1 的方向可能消失。常用措施：

- 短 horizon 在线更新；
- truncated BPTT；
- gradient clipping；
- downstream gradient decay；
- 状态/reward 正确归一化；
- 从稳定 baseline policy 开始；
- 必要时使用 double precision 诊断 NaN。

截断会引入 bias，但一个稳定、有用的近似 gradient 通常胜过数学上完整却溢出的 gradient。

## 30.7 APG 与 residual learning

官方教程中的 APG 以短窗口展开当前 policy，直接通过 MJX step 求 gradient，更新 policy 后从当前过程继续。相对高方差 ZoPG，它往往 sample efficient，但探索能力弱，对 reward shaping 和初值更敏感。

四足 locomotion 案例采用 residual policy：

\[
a_t=g(x_t;\phi)+f(g(x_t;\phi),x_t;\theta),

\]

冻结已有 baseline \(g\)，只学习小修正 \(f\)。这把搜索限制在一个已有可行 gait 附近的深谷中，正适合低方差、精确的 FoPG。简单地用 baseline 参数初始化一个全新网络，训练中仍可能迅速破坏原行为；显式 residual connection 始终保留 baseline。

## 30.8 Reward design

FoPG 不擅长从“摔倒扣 100 分”这类不连续事件得到提前避免跌倒的 gradient。官方 imitation 示例组合：

- minimal coordinate reference tracking，提高动作精度；
- maximal Cartesian coordinate tracking，提高训练稳定性；
- feet height shaping，明确摆动足几何目标。

locomotion 示例还用 gait schedule 和 Raibert-style foot placement target

\[
p^*=h_0+\frac{\Delta T}{2}v_0.
\]

这不是“作弊”，而是把任务结构编码成连续、局部有信息的目标。但每项 reward 都可能产生副作用，必须做 ablation 和真实任务指标验证。

## 30.9 MJX 数据与性能

MJX 是 JAX 生态中的批量、可微 MuJoCo 实现，不是把 C `mj_step` 原样搬到 GPU。关键工程规则：

- model/data 是 JAX pytree，更新是函数式的；
- batch shape 固定有利于 JIT；结构字段变化可能触发重新编译；
- 避免每步 host-device copy；policy、reward、reset 和 physics 全留在 accelerator；
- batch 必须足够大才能摊薄 JIT/dispatch；
- 对照官方 feature parity 检查 collision、constraint、actuator、plugin 等支持；
- 第一次 JIT 时间与稳态 step time 分开报告。

MuJoCo Warp 同样面向 accelerator batch，但数据布局、支持矩阵和优化路径不同。版本演进很快，本书不虚构 3.11.0 之外的固定 API；迁移时以随 SDK/在线版本提供的 parity 文档为准。

## 30.10 独立实验：手写 FoPG

`examples/40_policy_gradient/` 的 policy 是

\[
u=-k_pq-k_dv.
\]

程序每一步调用 `mjd_transitionFD` 得到 A/B，显式传播 \(\partial(q,v)/\partial(k_p,k_d)\)，对 finite-horizon quadratic cost 求精确的一阶 gradient，然后更新两个 gains。

```bash
cd examples/40_policy_gradient
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

这个例子刻意不用自动微分，让 BPTT 中最容易漏掉的“action 依赖 state，而 state 又依赖旧 action”路径全部显示在一个源码文件里。MJX/APG 是相同数学在大 batch、复杂 policy 上的自动化实现。

## 30.11 验证 gradient

任何可微训练前都应做 directional derivative check。随机方向 \(p\)：

\[
g^Tp\quad\text{vs}\quad
\frac{L(\theta+\epsilon p)-L(\theta-\epsilon p)}{2\epsilon}.
\]

扫描 epsilon，寻找相对误差平台。若失败，先检查 state ordering、policy hidden state、contact mode、reward time index 和 stop-gradient，再谈优化器。

## 30.12 常见误区

- 把 sample efficient 等同 wall-clock 快；反向传播常比 forward 慢且更耗内存；
- 大量环境并行就一定帮助 FoPG；其低 variance 不一定需要巨大 batch；
- 用稀疏 fall penalty 期待有用 gradient；
- 只验证训练 softness，不做接触参数随机化；
- 梯度 NaN 后直接改成 float64，却不查接触、归一化和 horizon；
- residual policy 实际覆盖 baseline 而不是相加；
- 比较 PPO/APG 时 sample、wall time、reward shaping 和初始化不公平；
- 未做 finite-difference gradient check。

## 30.13 习题与答案

1. FoPG 与 ZoPG 的根本差异是什么？  
   **答案：**FoPG 显式使用 transition Jacobian 并通过时间传播；ZoPG 只需 simulator 输出/trajectory samples。

2. 为什么停止 `action→future state` gradient 会伤害学习？  
   **答案：**删除了控制对未来 reward 的主要因果路径，策略只能看到不完整局部影响。

3. soft contact 为何有助 FoPG？  
   **答案：**接触力随穿入连续增长，在接触附近提供较平滑、有信息的局部导数。

4. residual learning 为什么适合 locomotion FoPG？  
   **答案：**baseline 保证基本可行 gait，低探索的 FoPG 只需学习局部精确修正。

5. 何时 CPU rollout 可能优于 MJX？  
   **答案：**batch 较小、模型 feature 复杂、horizon 长或 policy 不在 GPU 时，JIT/传输开销可能超过 accelerator 收益。
