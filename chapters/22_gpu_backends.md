# 第 22 章　逆运动学与阻尼最小二乘

逆运动学（IK）回答：为了让末端到达目标位姿，关节配置应怎样变化？它不涉及质量和力，却是机械臂规划、人形足端约束、遥操作和模型标定的基础。本章从速度映射推导 Jacobian transpose、伪逆与阻尼最小二乘，并把算法放回 MuJoCo 的配置流形中。

## 22.1 学习目标

- 建立位置与姿态残差，并明确表达坐标系；
- 从线性化 \(e(q+\delta q)\) 推导三类 IK 更新；
- 解释奇异值、可操作度和阻尼的关系；
- 对 free/ball joint 使用 `mj_integratePos`，避免直接相加四元数；
- 处理步长、关节限位、权重、零空间和不可达目标。

## 22.2 IK 是逐次局部线性化

末端任务 \(x=f(q)\)，目标 \(x_d\)，定义位置误差

\[
e=x_d-f(q).
\]

在当前配置附近：

\[
f(q\oplus\delta q)\approx f(q)+J(q)\delta q,
\]

所以希望解

\[
J\delta q\approx e.
\]

这里 \(\delta q\) 是 `nv` 维切空间增量，不一定是 `nq` 维数组差。只有全是 slide/hinge joint 时两者维数和局部加法才恰好一致。

## 22.3 Jacobian transpose

最简单更新：

\[
\delta q=\alpha J^Te.
\]

它可视作最小化 \(\tfrac12\|e\|^2\) 的梯度下降，因为梯度为 \(-J^Te\)。优点是便宜、在奇异处不需求逆；缺点是不同奇异方向尺度差异大，一个固定 \(\alpha\) 往往收敛慢。

不要把它与操作空间力控制混淆：两者都有 `Jᵀ`，但这里把误差变成**配置更新方向**，力控制则由虚功把末端 wrench 映射为**广义力**，单位和增益完全不同。

## 22.4 伪逆

最小二乘问题

\[
\min_{\delta q}\|J\delta q-e\|^2
\]

的最小范数解是 \(\delta q=J^+e\)。当任务维数小于 DoF 且满行秩时，

\[
J^+=J^T(JJ^T)^{-1}.
\]

SVD 解释最清楚：若 \(J=U\Sigma V^T\)，伪逆把每个可达方向除以奇异值 \(\sigma_i\)。当 \(\sigma_i\to0\)，微小任务误差会变成巨大关节更新，这就是伸直机械臂附近的奇异放大。

## 22.5 阻尼最小二乘（DLS）

加入 Tikhonov 正则：

\[
\min_{\delta q}\|J\delta q-e\|^2+\lambda^2\|\delta q\|^2.
\]

解为

\[
\delta q=J^T(JJ^T+\lambda^2I)^{-1}e
          =(J^TJ+\lambda^2I)^{-1}J^Te.
\]

前一种形式求逆矩阵大小为任务维数，适合高 DoF、低维末端任务；后一种适合 DoF 较少。DLS 在奇异方向使用 \(\sigma_i/(\sigma_i^2+\lambda^2)\)，不再无限放大。

`lambda` 越大越稳健但收敛越慢、稳态残差越大。实用策略是按最小奇异值自适应阻尼，或先用固定小阻尼建立可靠基线。

## 22.6 姿态误差

不能直接把四元数四个分量相减。设当前单位四元数 \(q_c\)、目标 \(q_d\)，误差旋转

\[
q_e=q_d\otimes q_c^{-1}.
\]

把它映射为三维旋转向量 \(e_R=\theta\hat u\)，再与 `mj_jacSite` 的转动 Jacobian 对应。MuJoCo 提供 `mju_subQuat` 计算切空间姿态差；同时要处理 \(q\) 与 \(-q\) 表示同一姿态的双覆盖。

六维位姿任务常写为

\[
e=\begin{bmatrix}w_p(p_d-p)\\w_R e_R\end{bmatrix},
\qquad
J=\begin{bmatrix}w_pJ_p\\w_RJ_R\end{bmatrix}.
\]

位置单位是米，姿态单位是弧度，权重决定二者折中，不能不加说明地直接拼接。

## 22.7 在配置流形上更新

求得 `nv` 维 `dq` 后：

```cpp
mj_integratePos(m, d->qpos, dq, step_size);
mj_forward(m, d);
```

`mj_integratePos` 对 hinge/slide 做加法，对 ball/free joint 做四元数指数映射并保持单位范数。直接执行 `qpos[i] += dq[i]` 在浮动基座机器人上维数就不匹配。

大步更新会让线性化失效。可限制 `||dq||`，或使用 line search：尝试 \(\alpha=1,1/2,1/4,...\)，只接受降低代价的步长。

## 22.8 关节限位与零空间

简单 clip 关节角虽然实用，但它破坏了求解方向，目标靠近限位时容易震荡。更系统的方法包括 box-constrained least squares、active set 或在代价中加入限位 barrier。

冗余机械臂可利用零空间：

\[
\delta q=J^+e+(I-J^+J)z,
\]

其中 \(z\) 可用于远离限位、保持舒适姿态或提高 manipulability。数值阻尼下 \(I-J^+J\) 不再是严格投影，主次任务会有小耦合，工程上应验证主任务误差。

## 22.9 独立实验：二维机械臂 DLS

`examples/32_damped_ik/` 用 2-DoF 平面臂到达二维目标。每轮调用 `mj_forward` 和 `mj_jacSite`，显式求解 2×2 的 \(JJ^T+\lambda^2I\)，再由 `mj_integratePos` 更新配置。

```bash
cd examples/32_damped_ik
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

把目标改为 `(1.1, 0)`，超过 0.9 m 最大臂展，观察算法不会“报错退出”，而是停在最接近配置并保留不可消除残差。这是迭代优化器的正常行为，应用层必须自行定义可达性阈值。

## 22.10 故障诊断

- 误差前几轮下降、随后上升：步长过大，使用 line search 或限幅；
- 伸直姿态关节更新爆炸：阻尼过小或使用了裸逆；
- 位置正确但姿态乱转：姿态残差符号、frame 或四元数顺序错误；
- 浮动基座四元数逐渐失真：没有用 `mj_integratePos`；
- 末端停在错误方向：检查 `Jp` 的行布局和世界/局部目标坐标；
- 靠近关节限位抖动：clip 与主任务相互打架，改用约束最小二乘；
- 多任务单位不一致：显式写出位置/姿态权重及其物理意义。

## 22.11 习题与答案

1. 为什么 DLS 在奇异处比伪逆稳定？  
   **答案：**奇异方向增益从 `1/sigma` 变成 `sigma/(sigma²+lambda²)`，不会无限放大。

2. `nq=7,nv=6` 的 free joint 能否把 6 维更新直接加到 qpos？  
   **答案：**不能；平移三维加四元数四维组成 qpos，更新在六维切空间，应使用 `mj_integratePos`。

3. 不可达目标下“最终误差非零”是否说明实现错误？  
   **答案：**不一定。最小二乘只找到局部最佳可达点，应用应结合阈值、限位和多初值判断。

4. 为什么位置和姿态残差需要权重？  
   **答案：**二者单位和任务容差不同，权重定义优化器如何在冲突时折中。

5. 零空间项怎样避免破坏主任务？  
   **答案：**将次任务方向投影到 `I-J⁺J`；使用阻尼近似时仍需监测主任务误差。
